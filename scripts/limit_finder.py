"""Argos joint limit finder.

Drives one servo at a time via the dashboard server's /api/command endpoint
so you can sweep angles by hand, mark min/max where the linkage binds, and
print a summary you can paste into DEFAULT_JOINT_LIMITS_RAD.

Bypasses both clamps that normally cap the servo command:
  * dashboard-side servo_cal min/max — uses the `raw_servo_angles` command,
    which only pins to the physical 0..180 horn domain.
  * firmware-side joint limits — at startup the script POSTs wide-open
    limits (±89 deg joint-space) so the firmware accepts the full sweep.
    Original limits are restored on exit.

Stdlib only — no requests/numpy needed. The dashboard server must be
running (default http://localhost:8787 on the Pi). Run from anywhere that
can reach the Pi:

    python scripts/limit_finder.py
    python scripts/limit_finder.py --host 192.168.1.42

Servo angles are in 0..180 deg space (90 = neutral). To convert to the
joint-space limits used in robot-config.js, subtract 90 — e.g. servo=50
means joint = -40 deg.

Commands at the prompt:
    <angle>        set selected joint to absolute servo angle
    +<step>        nudge up    (e.g. +5)
    -<step>        nudge down  (e.g. -5)
    select <name>  switch joint, e.g. 'select FR_femur' or 'select 4'
    list           show all 12 joints with their current angles
    mark min|max   record this angle as min/max for the selected joint
    marks          show every recorded mark
    center         all 12 servos back to 90
    release        cut PWM (limp) so you can move the linkage by hand
    lock           re-engage at last commanded angles
    done           exit and print summary
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request

LEG_IDS = ["FR", "FL", "RR", "RL"]
JOINT_ROWS = ["coxa", "femur", "tibia"]
JOINT_NAMES = [f"{leg}_{row}" for leg in LEG_IDS for row in JOINT_ROWS]

CENTER = 90.0
SERVO_MIN = 0.0
SERVO_MAX = 180.0


def joint_index(name: str) -> int:
    """Resolve 'FR_femur' / 'fr femur' / '4' to a 0..11 index."""
    s = name.strip().lower().replace("_joint", "").replace(" ", "_")
    if s.isdigit():
        i = int(s)
        if 0 <= i < 12:
            return i
        raise ValueError(f"joint index {i} out of range")
    for i, jn in enumerate(JOINT_NAMES):
        if jn.lower() == s:
            return i
    raise ValueError(f"unknown joint '{name}'")


class DashboardClient:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        data = json.dumps(body).encode("utf-8") if body is not None else None
        headers = {"Content-Type": "application/json"} if data is not None else {}
        req = urllib.request.Request(
            f"{self.base_url}{path}", data=data, headers=headers, method=method,
        )
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            raw = resp.read().decode("utf-8") or "{}"
            return json.loads(raw)

    def send_raw_servo_angles(self, angles_deg: list[float]) -> None:
        # raw bypasses dashboard's servo_cal clamp; only 0..180 horn limit applies.
        self._request("POST", "/api/command", {"type": "raw_servo_angles", "angles_deg": angles_deg})

    def release(self) -> None:
        self._request("POST", "/api/command", {"type": "disconnect"})

    def get_joint_limits(self) -> dict:
        return self._request("GET", "/api/settings/joint_limits")

    def set_joint_limits(self, limits: dict) -> dict:
        return self._request("POST", "/api/settings/joint_limits", limits)


def clamp(v: float) -> float:
    return max(SERVO_MIN, min(SERVO_MAX, v))


def print_state(angles: list[float], selected: int) -> None:
    name = JOINT_NAMES[selected]
    val = angles[selected]
    rel = val - CENTER
    print(f"  >> [{selected:2d}] {name:<10}  servo={val:6.1f}  joint={rel:+6.1f} deg")


def print_all(angles: list[float], selected: int) -> None:
    for i, name in enumerate(JOINT_NAMES):
        marker = "*" if i == selected else " "
        rel = angles[i] - CENTER
        print(f"   {marker} [{i:2d}] {name:<10}  servo={angles[i]:6.1f}  joint={rel:+6.1f}")


def print_marks(marks: dict[str, dict[str, float]]) -> None:
    if not marks:
        print("    (none yet)")
        return
    for name in JOINT_NAMES:
        if name not in marks:
            continue
        m = marks[name]
        lo = m.get("min")
        hi = m.get("max")
        lo_s = f"{lo:6.1f} (joint {lo - CENTER:+6.1f})" if lo is not None else "      —"
        hi_s = f"{hi:6.1f} (joint {hi - CENTER:+6.1f})" if hi is not None else "      —"
        print(f"    {name:<10}  min={lo_s}   max={hi_s}")


def summarize(marks: dict[str, dict[str, float]]) -> None:
    print()
    print("=" * 64)
    print("  SUMMARY")
    print("=" * 64)
    if not marks:
        print("  No marks recorded.")
        return
    print_marks(marks)
    print()
    print("  Joint-space (subtract 90 from servo angle):")
    print()
    # Aggregate by joint row across all four legs — DEFAULT_JOINT_LIMITS_RAD
    # is per-row, not per-leg. Take the *intersection* (max of mins, min of
    # maxes) so the resulting envelope is safe on every leg.
    for row in JOINT_ROWS:
        lows: list[float] = []
        highs: list[float] = []
        for leg in LEG_IDS:
            jn = f"{leg}_{row}"
            m = marks.get(jn, {})
            if "min" in m: lows.append(m["min"] - CENTER)
            if "max" in m: highs.append(m["max"] - CENTER)
        if lows and highs:
            lo = max(lows)
            hi = min(highs)
            print(f"    {row:<6}  [{lo:+6.1f}, {hi:+6.1f}]  (DEG2RAD intersection across legs)")
        else:
            print(f"    {row:<6}  (incomplete)")
    print()
    print("  Drop these into robot-config.js DEFAULT_JOINT_LIMITS_RAD as")
    print("    coxa/femur/tibia: [LO * DEG2RAD,  HI * DEG2RAD]")


WIDE_LIMITS_DEG = {
    f"{leg}_{row}_joint": {"min_deg": -89.0, "max_deg": 89.0}
    for leg in LEG_IDS for row in JOINT_ROWS
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="localhost", help="dashboard host (default: localhost)")
    parser.add_argument("--port", default=8787, type=int, help="dashboard port (default: 8787)")
    parser.add_argument("--start", default="FR_coxa", help="joint to start with (default: FR_coxa)")
    args = parser.parse_args()

    client = DashboardClient(f"http://{args.host}:{args.port}")
    angles = [CENTER] * 12
    selected = joint_index(args.start)
    marks: dict[str, dict[str, float]] = {}

    print("=" * 64)
    print("  Argos Joint Limit Finder")
    print("=" * 64)
    print(f"  Talking to {client.base_url}/api/command")
    print(f"  All 12 servos at 90 deg. Selected: {JOINT_NAMES[selected]}")
    print()

    # Snapshot the saved joint limits so we can restore on exit, then widen
    # firmware limits to ±89 deg so the firmware accepts the full sweep.
    saved_limits: dict | None = None
    try:
        saved_limits = client.get_joint_limits()
        client.set_joint_limits(WIDE_LIMITS_DEG)
        print("  Widened firmware joint limits to ±89 deg (will restore on exit).")
    except urllib.error.URLError as e:
        print(f"  ERROR: cannot reach dashboard server: {e}")
        return 1
    except Exception as e:
        print(f"  WARN: could not widen joint limits: {e}")
        print("        Sweeps may be clamped firmware-side.")

    try:
        client.send_raw_servo_angles(angles)
    except urllib.error.URLError as e:
        print(f"  ERROR: cannot reach dashboard server: {e}")
        return 1
    except Exception as e:
        print(f"  ERROR: bad response from dashboard: {e}")
        return 1

    print_state(angles, selected)
    print()
    print("  Type 'help' for commands, 'done' to exit.")

    try:
        exit_code = _run_loop(client, angles, selected, marks)
    finally:
        if saved_limits is not None:
            try:
                client.set_joint_limits(saved_limits)
                print("  Restored original firmware joint limits.")
            except Exception as e:
                print(f"  WARN: could not restore joint limits: {e}")
                print(f"        Saved snapshot: {json.dumps(saved_limits)}")

    summarize(marks)
    return exit_code


def _run_loop(client: DashboardClient, angles: list[float], selected: int, marks: dict) -> int:
    engaged = True
    while True:
        try:
            raw = input(f"  [{JOINT_NAMES[selected]}]> ").strip()
        except (KeyboardInterrupt, EOFError):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in ("done", "quit", "exit"):
            break

        if cmd in ("help", "?"):
            print(__doc__)
            continue

        if cmd == "list":
            print_all(angles, selected)
            continue

        if cmd == "marks":
            print_marks(marks)
            continue

        if cmd == "center":
            angles = [CENTER] * 12
            try:
                client.send_raw_servo_angles(angles)
                engaged = True
                print_state(angles, selected)
            except Exception as e:
                print(f"  send failed: {e}")
            continue

        if cmd == "release":
            try:
                client.release()
                engaged = False
                print("  Released — servos limp. Move the linkage by hand.")
            except Exception as e:
                print(f"  release failed: {e}")
            continue

        if cmd == "lock":
            try:
                client.send_raw_servo_angles(angles)
                engaged = True
                print("  Re-engaged at:")
                print_state(angles, selected)
            except Exception as e:
                print(f"  lock failed: {e}")
            continue

        if cmd == "select" and len(parts) >= 2:
            try:
                selected = joint_index(" ".join(parts[1:]))
                print_state(angles, selected)
            except ValueError as e:
                print(f"  {e}")
            continue

        if cmd == "mark" and len(parts) >= 2:
            label = parts[1].lower()
            if label not in ("min", "max"):
                print("  usage: mark min   |   mark max")
                continue
            jn = JOINT_NAMES[selected]
            marks.setdefault(jn, {})[label] = angles[selected]
            print(f"  Marked {jn} {label} = {angles[selected]:.1f} (joint {angles[selected] - CENTER:+.1f})")
            continue

        try:
            if raw.startswith("+") or raw.startswith("-"):
                step = float(raw)
                angles[selected] = clamp(angles[selected] + step)
            else:
                angles[selected] = clamp(float(raw))
        except ValueError:
            print("  Unknown command. Type 'help'.")
            continue

        if engaged:
            try:
                client.send_raw_servo_angles(angles)
            except Exception as e:
                print(f"  send failed: {e}")
                continue
        print_state(angles, selected)

    return 0


if __name__ == "__main__":
    sys.exit(main())
