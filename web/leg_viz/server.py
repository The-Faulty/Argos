"""HTTP bridge for the single-leg web viz.

Runs on the Pi. Serves the static viz files AND exposes a small JSON API
that drives the PCA9685 directly, so the browser sliders / preset buttons
can move the real servos.

Self-contained: does not depend on the ROS workspace or single_leg_test.py.
The PWM constants and servo channel assignments mirror those in
ros2_ws/argos_control/single_leg_test.py so the calibration matches.

Run:
    python3 server.py                     # default port 8000
    python3 server.py --port 8080         # custom port
    python3 server.py --sim               # no PCA9685, just log requests

Endpoints:
    GET  /                 index.html (and every other static asset)
    GET  /api/status       {connected: bool, sim: bool}
    POST /api/servo        body {"hip": 90, "top": 90, "bot": 90}
    POST /api/release      zeroes every servo duty cycle (lets them go limp)
"""

import argparse
import json
import logging
import sys
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# ── PCA9685 setup (mirrors single_leg_test.py) ────────────────────────────

PCA9685_I2C_ADDRESS = 0x40
HIP_CH = 2
TOP_CH = 1
BOT_CH = 0
PWM_MIN_US = 370
PWM_MAX_US = 2400
PWM_FREQ = 50
SERVO_RANGE_DEG = 270.0

log = logging.getLogger("legviz")

_pca = None
_sim_mode = False


def _servo_deg_to_us(servo_deg):
    servo_deg = max(0.0, min(SERVO_RANGE_DEG, float(servo_deg)))
    return PWM_MIN_US + (servo_deg / SERVO_RANGE_DEG) * (PWM_MAX_US - PWM_MIN_US)


def _us_to_duty(pulse_us):
    period_us = 1_000_000.0 / PWM_FREQ
    return int(round(pulse_us / period_us * 65535))


def init_hardware(force_sim=False):
    """Initialize the PCA9685. Falls back to sim mode on any error."""
    global _pca, _sim_mode
    if force_sim:
        log.info("sim mode forced (--sim)")
        _sim_mode = True
        return False
    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        i2c = busio.I2C(board.SCL, board.SDA)
        _pca = PCA9685(i2c, address=PCA9685_I2C_ADDRESS)
        _pca.frequency = PWM_FREQ
        log.info("PCA9685 OK (addr=0x%02X, channels hip=%d top=%d bot=%d)",
                 PCA9685_I2C_ADDRESS, HIP_CH, TOP_CH, BOT_CH)
        home_servos()
        return True
    except Exception as exc:
        log.warning("PCA9685 unavailable (%s) -- running in sim mode", exc)
        _sim_mode = True
        return False


def home_servos():
    """Drive a 90 -> 45 -> 90 wake-up sequence on every channel.

    The PCA9685 channels sometimes latch at a leftover duty cycle from
    the previous run, and some servos won't respond to a static setpoint
    that matches where they already are. Sweeping through a known
    sequence forces every servo to move, so by the time init returns
    the leg is reliably sitting at the 90/90/90 stand position.

    Raw servo degrees, not control-space angles -- this is a hardware
    home, not a kinematic pose. Each leg of the sweep is interpolated
    in small steps rather than jumped in one shot, so the mechanism
    doesn't slam against the bell-crank linkage on startup.
    """
    if _pca is None:
        return
    log.info("homing servos: 90 -> 45 -> 90 (interpolated)")
    waypoints = (90.0, 45.0, 90.0)
    cur = 90.0
    for target in waypoints:
        _sweep_all(cur, target, duration_s=0.35, steps=28)
        cur = target
        time.sleep(0.15)   # settle


def _sweep_all(start_deg, end_deg, duration_s, steps):
    """Drive all three channels from start_deg to end_deg in small steps."""
    if steps < 1:
        steps = 1
    dt = duration_s / steps
    for i in range(1, steps + 1):
        u = i / steps
        d = start_deg + (end_deg - start_deg) * u
        for ch in (HIP_CH, TOP_CH, BOT_CH):
            set_servo(ch, d)
        time.sleep(dt)


def set_servo(channel, servo_deg):
    pulse_us = _servo_deg_to_us(servo_deg)
    if _pca is None:
        return pulse_us
    _pca.channels[channel].duty_cycle = _us_to_duty(pulse_us)
    return pulse_us


def release_all():
    if _pca is None:
        return
    for ch in (HIP_CH, TOP_CH, BOT_CH):
        _pca.channels[ch].duty_cycle = 0


# ── HTTP handler ──────────────────────────────────────────────────────────

STATIC_DIR = Path(__file__).resolve().parent


class Handler(SimpleHTTPRequestHandler):
    # Serve files relative to this directory regardless of cwd.
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, fmt, *args):
        log.info("%s - %s", self.address_string(), fmt % args)

    def _json(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        # Loose CORS so a viz served from somewhere else can still talk to us.
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):   # preflight
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        if self.path == "/api/status":
            return self._json(200, {"connected": _pca is not None, "sim": _sim_mode})
        return super().do_GET()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(length) if length else b""
        try:
            body = json.loads(raw or b"{}")
        except json.JSONDecodeError as exc:
            return self._json(400, {"error": f"bad json: {exc}"})

        if self.path == "/api/servo":
            try:
                hip = float(body["hip"])
                top = float(body["top"])
                bot = float(body["bot"])
            except (KeyError, TypeError, ValueError):
                return self._json(400, {"error": "expected {hip, top, bot} numbers"})
            p_hip = set_servo(HIP_CH, hip)
            p_top = set_servo(TOP_CH, top)
            p_bot = set_servo(BOT_CH, bot)
            return self._json(200, {
                "hip": hip, "top": top, "bot": bot,
                "pulse_us": {"hip": round(p_hip, 1), "top": round(p_top, 1), "bot": round(p_bot, 1)},
            })

        if self.path == "/api/release":
            release_all()
            return self._json(200, {"released": True})

        if self.path == "/api/home":
            # 90 -> 45 -> 90 wake-up sweep. Same sequence init_hardware uses
            # at startup; exposed here so the browser can re-run it when the
            # servos get stuck from the PCA9685 channel latching at a
            # leftover duty cycle. The browser is expected to turn its own
            # 50 Hz poster off first (see /urdf_viewer.html Home button),
            # otherwise the sweep will fight the poser's setpoints.
            home_servos()
            return self._json(200, {"homed": True})

        return self._json(404, {"error": "unknown endpoint"})


# ── Entry point ───────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--sim", action="store_true", help="no hardware, log requests only")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    init_hardware(force_sim=args.sim)

    with ThreadingHTTPServer((args.host, args.port), Handler) as srv:
        mode = "SIM" if _sim_mode else "HARDWARE"
        log.info("serving %s on http://%s:%d  [%s]", STATIC_DIR, args.host, args.port, mode)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            log.info("shutting down")
            release_all()


if __name__ == "__main__":
    main()
