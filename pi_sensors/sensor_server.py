"""Argos sensor sidecar.

Runs alongside the Node dashboard server on the Pi and exposes:

  GET  /camera/stream        — multipart MJPEG of the RealSense color stream
  GET  /camera/depth-stream  — multipart MJPEG of the RealSense depth map
  WS   /sensors              — JSON frames: imu, depth, and thermal samples

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
DEPTH_VIS_MAX_M = float(os.environ.get("ARGOS_DEPTH_VIS_MAX_M", "4.0"))
DEPTH_ROI_PX = int(os.environ.get("ARGOS_DEPTH_ROI_PX", "24"))

# ─── RealSense ────────────────────────────────────────────────────────────

class RealSenseWorker:
    """Captures color/depth frames + IMU samples in a background thread."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_depth_jpeg: Optional[bytes] = None
        self.latest_imu: Optional[dict] = None
        self.latest_depth: Optional[dict] = None
        self.imu_subscribers: list[threading.Event] = []
        self.imu_subscriber_queues: list[deque] = []
        self.depth_subscribers: list[threading.Event] = []
        self.depth_subscriber_queues: list[deque] = []
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
            import numpy as np
        except ImportError as e:
            self.error = f"pyrealsense2/PIL/numpy missing: {e}"
            LOG(self.error)
            return

        def make_config(include_imu: bool):
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, CAM_FPS)
            cfg.enable_stream(rs.stream.depth, CAM_W, CAM_H, rs.format.z16, CAM_FPS)
            if include_imu:
                cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
                cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
            return cfg

        pipeline = rs.pipeline()
        # IMU streams. D435i / D455 only. Some devices accept the config call
        # but fail at pipeline.start(), so retry without IMU before giving up.
        has_imu = True
        try:
            cfg = make_config(include_imu=True)
            profile = pipeline.start(cfg)
        except Exception as imu_start_error:
            LOG(f"RealSense IMU not available, retrying color/depth only: {imu_start_error}")
            has_imu = False
            try:
                pipeline = rs.pipeline()
                cfg = make_config(include_imu=False)
                profile = pipeline.start(cfg)
            except Exception as e:
                self.error = f"realsense start failed: {e}"
                LOG(self.error)
                return

        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
            align = rs.align(rs.stream.color)
        except Exception as e:
            self.error = f"realsense depth init failed: {e}"
            LOG(self.error)
            return

        LOG(f"RealSense started @ {CAM_W}x{CAM_H}@{CAM_FPS} imu={has_imu} depth_scale={depth_scale}")
        last_accel = (0.0, 0.0, 0.0)
        last_gyro  = (0.0, 0.0, 0.0)
        # Gyro-z integrated yaw. No magnetometer / fusion, so this drifts
        # slowly — fine for a short demo, replace with Madgwick/Mahony if a
        # longer run starts to look obviously skewed.
        yaw_rad: float = 0.0
        prev_imu_ts: Optional[float] = None

        while self.running:
            try:
                raw_frames = pipeline.wait_for_frames(timeout_ms=1000)
                frames = align.process(raw_frames)
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

            depth = frames.get_depth_frame()
            if depth:
                depth_arr = np.asanyarray(depth.get_data()).astype(np.float32) * depth_scale
                depth_msg, depth_jpeg = _depth_measurement_and_jpeg(depth_arr, Image, io, np)
                with self.lock:
                    self.latest_depth = depth_msg
                    self.latest_depth_jpeg = depth_jpeg
                    for q in self.depth_subscriber_queues:
                        q.append(depth_msg)
                        while len(q) > 8:
                            q.popleft()
                    for ev in self.depth_subscribers:
                        ev.set()

            if has_imu:
                accel = raw_frames.first_or_default(rs.stream.accel)
                gyro = raw_frames.first_or_default(rs.stream.gyro)
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

    def subscribe_depth(self) -> tuple[threading.Event, deque]:
        ev = threading.Event()
        q: deque = deque()
        with self.lock:
            self.depth_subscribers.append(ev)
            self.depth_subscriber_queues.append(q)
        return ev, q

    def unsubscribe_depth(self, ev: threading.Event, q: deque) -> None:
        with self.lock:
            if ev in self.depth_subscribers: self.depth_subscribers.remove(ev)
            if q in self.depth_subscriber_queues: self.depth_subscriber_queues.remove(q)


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


def _depth_measurement_and_jpeg(depth_m, Image, io, np) -> tuple[dict, bytes]:
    """Build a compact dashboard depth sample plus a false-color JPEG map."""
    h, w = depth_m.shape
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    cx, cy = w // 2, h // 2
    half = max(1, DEPTH_ROI_PX // 2)
    roi = depth_m[max(0, cy - half):min(h, cy + half + 1), max(0, cx - half):min(w, cx + half + 1)]
    roi_valid = roi[np.isfinite(roi) & (roi > 0.0)]

    def finite_or_none(value) -> Optional[float]:
        return float(value) if np.isfinite(value) else None

    center_m = float(depth_m[cy, cx]) if valid[cy, cx] else None
    if roi_valid.size:
        roi_mean_m = finite_or_none(roi_valid.mean())
        roi_min_m = finite_or_none(roi_valid.min())
        roi_max_m = finite_or_none(roi_valid.max())
    else:
        roi_mean_m = roi_min_m = roi_max_m = None
    valid_depth = depth_m[valid]
    nearest_m = finite_or_none(valid_depth.min()) if valid_depth.size else None

    msg = {
        "type": "depth",
        "ts": time.time(),
        "width": int(w),
        "height": int(h),
        "units": "m",
        "center": {"x": int(cx), "y": int(cy), "m": center_m},
        "roi": {
            "x": int(cx),
            "y": int(cy),
            "size_px": int(DEPTH_ROI_PX),
            "mean_m": roi_mean_m,
            "min_m": roi_min_m,
            "max_m": roi_max_m,
        },
        "nearest_m": nearest_m,
        "valid_pct": float(valid.mean() * 100.0),
        "vis_max_m": DEPTH_VIS_MAX_M,
    }

    norm = np.clip(depth_m / max(0.1, DEPTH_VIS_MAX_M), 0.0, 1.0)
    norm[~valid] = 0.0
    # Lightweight black -> blue/cyan -> yellow palette without OpenCV.
    r = np.clip((norm - 0.35) / 0.65, 0.0, 1.0)
    g = np.clip(norm / 0.55, 0.0, 1.0)
    b = np.clip((0.75 - norm) / 0.75, 0.0, 1.0)
    rgb = np.stack([r, g, b], axis=-1)
    rgb[~valid] = 0.0
    rgb8 = (rgb * 255).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(rgb8).save(buf, format="JPEG", quality=JPEG_Q)
    return msg, buf.getvalue()


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
        "depth_running":     realsense.running and realsense.error is None and realsense.latest_depth is not None,
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


@app.route("/camera/depth-stream")
def depth_stream():
    boundary = b"argosdepth"

    def gen():
        if realsense.error:
            return
        while True:
            with realsense.lock:
                jpeg = realsense.latest_depth_jpeg
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
    depth_ev, depth_q = realsense.subscribe_depth()
    th_ev, th_q = thermal.subscribe()
    try:
        # Initial snapshot.
        if realsense.latest_imu:
            ws.send(json.dumps(realsense.latest_imu))
        if realsense.latest_depth:
            ws.send(json.dumps(realsense.latest_depth))
        if thermal.latest:
            ws.send(json.dumps(thermal.latest))
        while True:
            triggered = imu_ev.wait(timeout=0.03) or depth_ev.wait(timeout=0.03) or th_ev.wait(timeout=0.03)
            imu_ev.clear()
            depth_ev.clear()
            th_ev.clear()
            while imu_q:
                ws.send(json.dumps(imu_q.popleft()))
            while depth_q:
                ws.send(json.dumps(depth_q.popleft()))
            while th_q:
                ws.send(json.dumps(th_q.popleft()))
            if not triggered:
                # Heartbeat keeps the proxy from timing out idle connections.
                ws.send(json.dumps({"type": "ping", "ts": time.time()}))
    except Exception as e:
        LOG(f"sensors WS closed: {e}")
    finally:
        realsense.unsubscribe(imu_ev, imu_q)
        realsense.unsubscribe_depth(depth_ev, depth_q)
        thermal.unsubscribe(th_ev, th_q)


def main():
    realsense.start()
    thermal.start()
    LOG(f"listening on {HOST}:{PORT}")
    app.run(host=HOST, port=PORT, threaded=True)


if __name__ == "__main__":
    main()
