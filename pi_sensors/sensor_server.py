"""Argos sensor sidecar.

Runs alongside the Node dashboard server on the Pi and exposes:

  GET  /camera/stream  — multipart MJPEG of the RealSense color stream
  WS   /sensors        — JSON frames: {type:"imu",...} and {type:"thermal",...}

Two threads do the actual work — RealSense pipeline and MLX90640 reader —
so the Flask request handler never blocks on hardware. Failures in either
sensor leave the other working; the dashboard handles the absent stream.
"""

from __future__ import annotations

import base64
import json
import math
import os
import threading
import time
from collections import deque
from typing import Optional

from flask import Flask, Response, abort
from flask_sock import Sock

LOG = lambda *a: print("[argos-sensors]", *a, flush=True)

CAM_FPS = int(os.environ.get("ARGOS_CAM_FPS", "15"))
CAM_W   = int(os.environ.get("ARGOS_CAM_W",   "640"))
CAM_H   = int(os.environ.get("ARGOS_CAM_H",   "480"))
JPEG_Q  = int(os.environ.get("ARGOS_JPEG_Q",  "70"))
PORT    = int(os.environ.get("ARGOS_SIDECAR_PORT", "8788"))
HOST    = os.environ.get("ARGOS_SIDECAR_HOST", "0.0.0.0")

# ─── RealSense ────────────────────────────────────────────────────────────

class RealSenseWorker:
    """Captures color frames + IMU samples in a background thread."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_imu: Optional[dict] = None
        self.imu_subscribers: list[threading.Event] = []
        self.imu_subscriber_queues: list[deque] = []
        self.running = False
        self.error: Optional[str] = None

    def start(self) -> None:
        self.running = True
        threading.Thread(target=self._run, name="realsense", daemon=True).start()

    def _run(self) -> None:
        try:
            import pyrealsense2 as rs
            from PIL import Image
            import io
        except ImportError as e:
            self.error = f"pyrealsense2/PIL missing: {e}"
            LOG(self.error)
            return

        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, CAM_FPS)
        # IMU streams. D435i / D455 only — D435 (no i) raises here.
        try:
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
            cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
            has_imu = True
        except Exception:
            has_imu = False
            LOG("RealSense IMU not available (D435 without 'i'?)")

        try:
            pipeline.start(cfg)
        except Exception as e:
            self.error = f"realsense start failed: {e}"
            LOG(self.error)
            return

        LOG(f"RealSense started @ {CAM_W}x{CAM_H}@{CAM_FPS} imu={has_imu}")
        last_accel = (0.0, 0.0, 0.0)
        last_gyro  = (0.0, 0.0, 0.0)
        # Gyro-z integrated yaw. No magnetometer / fusion, so this drifts
        # slowly — fine for a short demo, replace with Madgwick/Mahony if a
        # longer run starts to look obviously skewed.
        yaw_rad: float = 0.0
        prev_imu_ts: Optional[float] = None

        while self.running:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
            except Exception as e:
                LOG(f"RealSense wait error: {e}")
                continue

            color = frames.get_color_frame()
            if color:
                import numpy as np
                arr = np.asanyarray(color.get_data())
                # BGR → RGB then JPEG-encode via Pillow.
                rgb = arr[..., ::-1]
                buf = io.BytesIO()
                Image.fromarray(rgb).save(buf, format="JPEG", quality=JPEG_Q)
                with self.lock:
                    self.latest_jpeg = buf.getvalue()

            if has_imu:
                accel = frames.first_or_default(rs.stream.accel)
                gyro = frames.first_or_default(rs.stream.gyro)
                if accel:
                    motion = accel.as_motion_frame().get_motion_data()
                    last_accel = (motion.x, motion.y, motion.z)
                if gyro:
                    motion = gyro.as_motion_frame().get_motion_data()
                    last_gyro = (motion.x, motion.y, motion.z)
                now_ts = time.time()
                if prev_imu_ts is not None:
                    # Clamp dt against frame stalls (camera hiccups, USB
                    # disconnect retries) so a single ~second-long gap
                    # doesn't lurch yaw by hundreds of degrees.
                    dt = max(0.0, min(0.1, now_ts - prev_imu_ts))
                    yaw_rad += last_gyro[2] * dt
                    if yaw_rad > math.pi:
                        yaw_rad -= 2.0 * math.pi
                    elif yaw_rad < -math.pi:
                        yaw_rad += 2.0 * math.pi
                prev_imu_ts = now_ts
                imu_msg = {
                    "type": "imu",
                    "ts": now_ts,
                    "accel": list(last_accel),
                    "gyro": list(last_gyro),
                    "quaternion": _quaternion_from_accel(last_accel, yaw_rad),
                }
                with self.lock:
                    self.latest_imu = imu_msg
                    for q in self.imu_subscriber_queues:
                        q.append(imu_msg)
                        while len(q) > 16:
                            q.popleft()
                    for ev in self.imu_subscribers:
                        ev.set()

    def stop(self) -> None:
        self.running = False

    def subscribe(self) -> tuple[threading.Event, deque]:
        ev = threading.Event()
        q: deque = deque()
        with self.lock:
            self.imu_subscribers.append(ev)
            self.imu_subscriber_queues.append(q)
        return ev, q

    def unsubscribe(self, ev: threading.Event, q: deque) -> None:
        with self.lock:
            if ev in self.imu_subscribers: self.imu_subscribers.remove(ev)
            if q in self.imu_subscriber_queues: self.imu_subscriber_queues.remove(q)


def _quaternion_from_accel(
    accel: tuple[float, float, float], yaw: float = 0.0
) -> list[float]:
    """Roll/pitch from accel + caller-supplied yaw (gyro-integrated upstream).
    No magnetometer fusion — yaw drifts slowly. Swap in Madgwick/Mahony if
    that becomes a problem."""
    ax, ay, az = accel
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-6:
        # Pure-yaw quaternion when accel is unusable (free-fall / unplugged).
        return [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]
    ax, ay, az = ax / norm, ay / norm, az / norm
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


# ─── MLX90640 thermal ────────────────────────────────────────────────────

class ThermalWorker:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest: Optional[dict] = None
        self.subscribers: list[threading.Event] = []
        self.subscriber_queues: list[deque] = []
        self.running = False
        self.error: Optional[str] = None

    def start(self) -> None:
        self.running = True
        threading.Thread(target=self._run, name="thermal", daemon=True).start()

    def _run(self) -> None:
        try:
            import board
            import busio
            import adafruit_mlx90640
            import numpy as np
        except ImportError as e:
            self.error = f"mlx90640 deps missing: {e}"
            LOG(self.error)
            return

        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
            mlx = adafruit_mlx90640.MLX90640(i2c)
            mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ
        except Exception as e:
            self.error = f"mlx90640 init failed: {e}"
            LOG(self.error)
            return

        LOG("MLX90640 started @ 8 Hz")
        frame = np.zeros(32 * 24, dtype=np.float32)
        while self.running:
            try:
                mlx.getFrame(frame)
            except Exception as e:
                LOG(f"MLX90640 read error: {e}")
                time.sleep(0.5)
                continue
            grid = frame.reshape(24, 32).tolist()
            msg = {
                "type": "thermal",
                "ts": time.time(),
                "rows": 24,
                "cols": 32,
                "min_c": float(frame.min()),
                "max_c": float(frame.max()),
                "frame": grid,
            }
            with self.lock:
                self.latest = msg
                for q in self.subscriber_queues:
                    q.append(msg)
                    while len(q) > 8:
                        q.popleft()
                for ev in self.subscribers:
                    ev.set()

    def subscribe(self) -> tuple[threading.Event, deque]:
        ev = threading.Event()
        q: deque = deque()
        with self.lock:
            self.subscribers.append(ev)
            self.subscriber_queues.append(q)
        return ev, q

    def unsubscribe(self, ev: threading.Event, q: deque) -> None:
        with self.lock:
            if ev in self.subscribers: self.subscribers.remove(ev)
            if q in self.subscriber_queues: self.subscriber_queues.remove(q)


# ─── Flask app ────────────────────────────────────────────────────────────

app = Flask(__name__)
sock = Sock(app)
realsense = RealSenseWorker()
thermal = ThermalWorker()


@app.route("/health")
def health():
    return {
        "ok": True,
        "realsense_running": realsense.running and realsense.error is None,
        "thermal_running":   thermal.running and thermal.error is None,
        "errors": {
            "realsense": realsense.error,
            "thermal":   thermal.error,
        },
    }


@app.route("/camera/stream")
def camera_stream():
    boundary = b"argosframe"

    def gen():
        # If RealSense is unavailable, refuse fast so the proxy can show 502.
        if realsense.error:
            return
        while True:
            with realsense.lock:
                jpeg = realsense.latest_jpeg
            if jpeg is None:
                time.sleep(1.0 / max(1, CAM_FPS))
                continue
            yield (
                b"--" + boundary + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" +
                jpeg + b"\r\n"
            )
            time.sleep(1.0 / max(1, CAM_FPS))

    if realsense.error:
        abort(503, description=realsense.error)
    return Response(
        gen(),
        mimetype="multipart/x-mixed-replace; boundary=" + boundary.decode(),
    )


@sock.route("/sensors")
def sensors_ws(ws):
    imu_ev, imu_q = realsense.subscribe()
    th_ev, th_q = thermal.subscribe()
    try:
        # Initial snapshot.
        if realsense.latest_imu:
            ws.send(json.dumps(realsense.latest_imu))
        if thermal.latest:
            ws.send(json.dumps(thermal.latest))
        while True:
            triggered = imu_ev.wait(timeout=0.05) or th_ev.wait(timeout=0.05)
            imu_ev.clear()
            th_ev.clear()
            while imu_q:
                ws.send(json.dumps(imu_q.popleft()))
            while th_q:
                ws.send(json.dumps(th_q.popleft()))
            if not triggered:
                # Heartbeat keeps the proxy from timing out idle connections.
                ws.send(json.dumps({"type": "ping", "ts": time.time()}))
    except Exception as e:
        LOG(f"sensors WS closed: {e}")
    finally:
        realsense.unsubscribe(imu_ev, imu_q)
        thermal.unsubscribe(th_ev, th_q)


def main():
    realsense.start()
    thermal.start()
    LOG(f"listening on {HOST}:{PORT}")
    app.run(host=HOST, port=PORT, threaded=True)


if __name__ == "__main__":
    main()
