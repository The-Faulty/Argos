"""Single-leg bench test for Argos.

Talks straight to the PCA9685 over I2C so you can check one leg without
bringing up ROS.  Run with --sim if no hardware is attached.
"""

import argparse
import sys
import time
from math import pi, acos, atan2, sqrt, sin, cos, fmod, asin, radians, degrees

import numpy as np

try:
    from .Config import Configuration
except ImportError:
    from Config import Configuration


PCA9685_I2C_ADDRESS = 0x40   # default I2C address for the PCA9685 servo driver

# PCA9685 channel assignments for this test leg.
HIP_CH = 2   # hip abductor servo
TOP_CH = 1   # upper leg servo
BOT_CH = 0   # lower/bell-crank servo

# Pulse limits from the servo datasheet (microseconds).
# Adjust if the servos hit their physical endpoints before these values.
PWM_MIN_US = 370
PWM_MAX_US = 2400
PWM_FREQ = 50
SERVO_RANGE_DEG = 270.0
SERVO_CENTER_DEG = SERVO_RANGE_DEG / 2.0

# Flip a multiplier to -1 if a servo runs backward.
# Adjust an offset (degrees) if the servo neutral doesn't match the IK zero.
HIP_OFFSET = 0.0
HIP_MULTIPLIER = -1

TOP_OFFSET = 0.0
TOP_MULTIPLIER = +1

BOT_OFFSET = 0.0
BOT_MULTIPLIER = +1

# Which leg to test: 0=FR, 1=FL, 2=RR, 3=RL
LEG_INDEX = 0
LEG_NAMES = ("FR", "FL", "RR", "RL")

# Pull the bench geometry from Config.py so the script stays aligned with the
# latest measured linkage values.
_CONFIG = Configuration()
PARAMS = dict(_CONFIG.leg_params)
LEG_LATERAL_OFFSET = float(_CONFIG.LEG_ORIGINS[1, LEG_INDEX])
DEFAULT_BODY_Z_M = float(_CONFIG.default_z_ref)

# Motion settings.
STEPS_PER_MOVE = 30
STEP_DELAY_S = 0.018
STARTUP_SAG = (0, int(round(abs(DEFAULT_BODY_Z_M) * 1000.0)))
STARTUP_SERVO_DEG = (90.0, 90.0, 90.0)
MOCK_WALK_DWELL_S = 0.12
MOCK_WALK_SERVO_WAYPOINTS = [
    STARTUP_SERVO_DEG,
    (94.0, 82.0, 76.0),
    (100.0, 80.0, 72.0),
    (96.0, 86.0, 82.0),
    STARTUP_SERVO_DEG,
    (84.0, 98.0, 104.0),
    (80.0, 94.0, 100.0),
    STARTUP_SERVO_DEG,
]


# ── Unit-converting wrappers around Kinematics.py ─────────────────────────────
# The ROS nodes work in radians and meters.  This script uses degrees and
# millimeters so the interactive REPL stays easy to read.

def _point_to_rad(p1, p2):
    return (atan2(p2, p1) + 2 * pi) % (2 * pi)


def _rotx(angle):
    c, s = cos(angle), sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ci(P1, r1, P2, r2):
    dx, dy = P2[0] - P1[0], P2[1] - P1[1]
    d = sqrt(dx * dx + dy * dy)
    if d < 1e-9 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return None, None
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = sqrt(max(0, r1 * r1 - a * a))
    mx = P1[0] + a * dx / d
    my = P1[1] + a * dy / d
    px, py = -dy / d, dx / d
    return np.array([mx + h * px, my + h * py]), np.array([mx - h * px, my - h * py])


def hip_ik(r_foot, leg_index, params):
    """Solve the hip abductor and return (theta_1_deg, x_sag_m, y_sag_m)."""
    is_right = leg_index in (0, 2)
    x, y, z = float(r_foot[0]), float(r_foot[1]), float(r_foot[2])
    if is_right:
        y = -y

    R1 = pi / 2 - params["phi"]
    rf = _rotx(-R1) @ np.array([x, y, z])
    x, y, z = rf

    len_A = sqrt(y * y + z * z)
    if len_A < 1e-9:
        len_A = 1e-9

    asin_arg = sin(params["phi"]) * params["L1_hip"] / len_A
    asin_arg = max(-1.0, min(1.0, asin_arg))
    a1 = _point_to_rad(y, z)
    a2 = asin(asin_arg)
    a3 = pi - a2 - params["phi"]
    t1 = a1 + a3
    if t1 >= 2 * pi:
        t1 = fmod(t1, 2 * pi)

    offset = np.array([0.0, params["L1_hip"] * cos(t1), params["L1_hip"] * sin(t1)])
    translated = np.array([x, y, z]) - offset
    R2 = t1 + params["phi"] - pi / 2
    sag = _rotx(-R2) @ translated
    x_sag, _, z_sag = sag
    return degrees(t1), x_sag, -z_sag


def sag_ik(x_m, y_m, params):
    """Solve the sagittal linkage. Returns (theta_top_deg, theta_bot_deg) or None."""
    L1, L2 = params["L1"], params["L2"]
    origin = np.zeros(2)

    d = sqrt(x_m * x_m + y_m * y_m)
    max_reach = (L1 + L2) * 0.99
    if d > max_reach:
        x_m, y_m = x_m * max_reach / d, y_m * max_reach / d
        d = max_reach
    if d < abs(L1 - L2) * 1.01:
        return None

    cos_k = max(-1, min(1, (L1**2 + L2**2 - d**2) / (2 * L1 * L2)))
    knee_int = acos(cos_k)
    alpha = atan2(x_m, y_m)
    cos_b = max(-1, min(1, (L1**2 + d**2 - L2**2) / (2 * L1 * d)))
    theta_top = alpha - acos(cos_b)

    knee = np.array([L1 * sin(theta_top), L1 * cos(theta_top)])
    theta_lower = theta_top + (pi - knee_int)
    rod2_pt = knee - params["dko"] * np.array([sin(theta_lower), cos(theta_lower)])

    A, B = _ci(origin, params["rbl"], rod2_pt, params["Lr2"])
    if A is None:
        return None
    bc_left = A if A[0] <= B[0] else B
    phi_left = atan2(bc_left[0], bc_left[1])
    phi_right = phi_left - params["abc"]
    bc_right = np.array([params["rbr"] * sin(phi_right), params["rbr"] * cos(phi_right)])

    A, B = _ci(origin, params["rh"], bc_right, params["Lr1"])
    if A is None:
        return None
    horn = A if A[0] >= B[0] else B
    return degrees(theta_top), degrees(atan2(horn[0], horn[1]))


def sag_fk(theta_top_deg, theta_bot_deg, params):
    """Sagittal FK. Returns (x_mm, y_mm) or None."""
    theta_top = radians(theta_top_deg)
    theta_bot = radians(theta_bot_deg)
    L1, L2 = params["L1"], params["L2"]
    origin = np.zeros(2)

    knee = np.array([L1 * sin(theta_top), L1 * cos(theta_top)])
    horn = np.array([params["rh"] * sin(theta_bot), params["rh"] * cos(theta_bot)])
    A, B = _ci(origin, params["rbr"], horn, params["Lr1"])
    if A is None:
        return None

    def bcl_x(candidate):
        return sin(atan2(candidate[0], candidate[1]) + params["abc"])

    bc_right = A if bcl_x(A) < bcl_x(B) else B
    phi_right = atan2(bc_right[0], bc_right[1])
    phi_left = phi_right + params["abc"]
    bc_left = np.array([params["rbl"] * sin(phi_left), params["rbl"] * cos(phi_left)])
    A, B = _ci(knee, params["dko"], bc_left, params["Lr2"])
    if A is None:
        return None

    def diff(candidate):
        leg_dir = knee - candidate
        delta = (atan2(leg_dir[0], leg_dir[1]) - theta_top) % (2 * pi)
        return delta - 2 * pi if delta > pi else delta

    dA, dB = diff(A), diff(B)
    rod2 = A if 0 < dA < pi else (B if 0 < dB < pi else (A if abs(dA - pi / 2) < abs(dB - pi / 2) else B))
    leg_dir = (knee - rod2) / np.linalg.norm(knee - rod2)
    foot = knee + L2 * leg_dir
    return foot[0] * 1000, foot[1] * 1000


# ── Composite helpers ─────────────────────────────────────────────────────────

def full_ik_from_sag(x_mm, y_mm, lateral_m=None):
    """Full 3-DOF IK from a sagittal plane target in mm."""
    if lateral_m is None:
        lateral_m = LEG_LATERAL_OFFSET
    r_foot = np.array([x_mm / 1000.0, lateral_m, -y_mm / 1000.0])
    t1_deg, x_sag, y_sag = hip_ik(r_foot, LEG_INDEX, PARAMS)
    sol = sag_ik(x_sag, y_sag, PARAMS)
    if sol is None:
        return None
    return t1_deg, sol[0], sol[1]


def solve_body_target(bx, by, bz):
    """Full 3-DOF IK from a body-frame target in meters."""
    r_foot = np.array([bx, by, bz])
    t1_deg, x_sag, y_sag = hip_ik(r_foot, LEG_INDEX, PARAMS)
    sol = sag_ik(x_sag, y_sag, PARAMS)
    if sol is None:
        return None
    return t1_deg, sol[0], sol[1]


def crouch_reference_angles():
    """Return the actual IK angles for the startup crouch pose."""
    sol = full_ik_from_sag(*STARTUP_SAG)
    if sol is None:
        raise RuntimeError(
            f"Startup sag {STARTUP_SAG} mm is unreachable for leg {LEG_NAMES[LEG_INDEX]}"
        )
    return sol


def _servo_deg_to_us(servo_deg):
    """Convert a direct servo angle in degrees into pulse width."""
    servo_deg = max(0.0, min(SERVO_RANGE_DEG, float(servo_deg)))
    return PWM_MIN_US + (servo_deg / SERVO_RANGE_DEG) * (PWM_MAX_US - PWM_MIN_US)


def _us_to_servo_deg(pulse_us):
    """Reverse of _servo_deg_to_us."""
    return (pulse_us - PWM_MIN_US) / (PWM_MAX_US - PWM_MIN_US) * SERVO_RANGE_DEG


def _servo_deg_to_ik(servo_deg, offset_deg, multiplier):
    """Convert a direct servo angle into the script's control-space angle."""
    servo_deg = max(0.0, min(SERVO_RANGE_DEG, float(servo_deg)))
    return (servo_deg - SERVO_CENTER_DEG - offset_deg) / multiplier


def servo_degrees_to_angles(hip_deg, top_deg, bot_deg):
    """Map direct servo angles into the script's control-space angle triplet."""
    return (
        _servo_deg_to_ik(hip_deg, HIP_OFFSET, HIP_MULTIPLIER),
        _servo_deg_to_ik(top_deg, TOP_OFFSET, TOP_MULTIPLIER),
        _servo_deg_to_ik(bot_deg, BOT_OFFSET, BOT_MULTIPLIER),
    )


def stand_reference_angles():
    """Return the standing startup pose expressed in control-space angles."""
    return servo_degrees_to_angles(*STARTUP_SERVO_DEG)


def _ik_to_servo_us(joint_angle_deg, offset_deg, multiplier):
    """Map an IK joint angle to a servo pulse width in microseconds.

    Servos expect 0–270 degrees centered at SERVO_CENTER_DEG. Offset and multiplier
    correct for mounting direction and neutral position.
    """
    cmd = SERVO_CENTER_DEG + multiplier * joint_angle_deg + offset_deg
    return _servo_deg_to_us(cmd)


def _servo_us_to_ik(pulse_us, offset_deg, multiplier):
    """Reverse of _ik_to_servo_us: pulse width -> IK angle in degrees."""
    cmd = _us_to_servo_deg(pulse_us)
    return (cmd - SERVO_CENTER_DEG - offset_deg) / multiplier


def _us_to_duty(pulse_us):
    """Convert a pulse width in microseconds to a PCA9685 duty-cycle value (0–65535)."""
    period_us = 1_000_000.0 / PWM_FREQ
    return int(round(pulse_us / period_us * 65535))


def _duty_to_us(duty_16):
    """Reverse of _us_to_duty: 16-bit duty cycle -> pulse width in microseconds."""
    period_us = 1_000_000.0 / PWM_FREQ
    return duty_16 * period_us / 65535.0


_pca = None


def init_hardware():
    """Initialize the PCA9685. Returns True on success, False in sim mode."""
    global _pca
    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        i2c = busio.I2C(board.SCL, board.SDA)
        _pca = PCA9685(i2c, address=PCA9685_I2C_ADDRESS)
        _pca.frequency = PWM_FREQ
        print(
            f"  PCA9685 OK  (addr=0x{PCA9685_I2C_ADDRESS:02X}  "
            f"channels: hip={HIP_CH} top={TOP_CH} bot={BOT_CH})"
        )
        return True
    except Exception as exc:
        print(f"  [WARN] PCA9685 unavailable ({exc})")
        print("         Running in simulation mode.\n")
        return False


def _set_pulse(channel, pulse_us):
    if _pca is None:
        return
    _pca.channels[channel].duty_cycle = _us_to_duty(pulse_us)


def send_angles(t1_deg, tt_deg, tb_deg, verbose=True):
    """Drive all three servos to the given IK angles."""
    hip_us = _ik_to_servo_us(t1_deg, HIP_OFFSET, HIP_MULTIPLIER)
    top_us = _ik_to_servo_us(tt_deg, TOP_OFFSET, TOP_MULTIPLIER)
    bot_us = _ik_to_servo_us(tb_deg, BOT_OFFSET, BOT_MULTIPLIER)
    _set_pulse(HIP_CH, hip_us)
    _set_pulse(TOP_CH, top_us)
    _set_pulse(BOT_CH, bot_us)
    if verbose:
        print(f"    theta_1  ={t1_deg:+7.2f} deg  ch{HIP_CH}  {hip_us:.0f} us")
        print(f"    theta_top={tt_deg:+7.2f} deg  ch{TOP_CH}  {top_us:.0f} us")
        print(f"    theta_bot={tb_deg:+7.2f} deg  ch{BOT_CH}  {bot_us:.0f} us")


def move_to_angles(target_angles, from_angles=None, verbose=True):
    """Move to an angle triplet, interpolating when hardware is active."""
    t1_target, tt_target, tb_target = target_angles

    if from_angles is None or _pca is None:
        send_angles(t1_target, tt_target, tb_target, verbose=verbose)
        return t1_target, tt_target, tb_target

    t1_start, tt_start, tb_start = from_angles
    for step in range(1, STEPS_PER_MOVE + 1):
        blend = step / STEPS_PER_MOVE
        send_angles(
            t1_start + blend * (t1_target - t1_start),
            tt_start + blend * (tt_target - tt_start),
            tb_start + blend * (tb_target - tb_start),
            verbose=False,
        )
        time.sleep(STEP_DELAY_S)

    if verbose:
        send_angles(t1_target, tt_target, tb_target, verbose=True)
    return t1_target, tt_target, tb_target


def move_to_servo_degrees(target_servo_deg, from_angles=None, verbose=True):
    """Move using direct servo angles in degrees."""
    hip_deg, top_deg, bot_deg = [float(v) for v in target_servo_deg]
    if verbose:
        print(
            f"\n  Servo target: hip={hip_deg:.1f} deg"
            f"  top={top_deg:.1f} deg  bot={bot_deg:.1f} deg"
        )
    return move_to_angles(
        servo_degrees_to_angles(hip_deg, top_deg, bot_deg),
        from_angles=from_angles,
        verbose=verbose,
    )


def move_to_sag(x_mm, y_mm, from_angles=None, verbose=True):
    """Move the foot to a sagittal target in mm."""
    sol = full_ik_from_sag(x_mm, y_mm)
    if sol is None:
        print(f"  [WARN] ({x_mm:.0f}, {y_mm:.0f}) mm unreachable - skipped")
        return None

    t1_deg, tt_deg, tb_deg = sol
    if verbose:
        fk = sag_fk(tt_deg, tb_deg, PARAMS)
        fk_str = f"({fk[0]:.1f}, {fk[1]:.1f})mm" if fk else "?"
        print(f"\n  Target: ({x_mm:.0f}, {y_mm:.0f})mm  FK check: {fk_str}")

    return move_to_angles((t1_deg, tt_deg, tb_deg), from_angles=from_angles, verbose=verbose)


def release():
    """Zero all servo duty cycles (lets the servos go limp)."""
    if _pca is None:
        return
    for channel in [HIP_CH, TOP_CH, BOT_CH]:
        _pca.channels[channel].duty_cycle = 0
    print("  Servos released.")


# ── Self-tests ────────────────────────────────────────────────────────────────

def test_math():
    """Run IK/FK round-trip checks and print results. Returns True if all pass."""
    from math import sqrt

    print("\n-- IK/FK self-test ---------------------------------------------")
    print("  Sagittal IK/FK:")
    nominal_y_m = abs(DEFAULT_BODY_Z_M)
    sag_tests = [
        (0.000, nominal_y_m, "Nominal"),
        (0.025, nominal_y_m - 0.005, "Fwd +25mm"),
        (0.050, nominal_y_m - 0.015, "Wide fwd"),
        (-0.025, nominal_y_m - 0.005, "Back -25mm"),
        (-0.040, nominal_y_m - 0.010, "Wide back"),
        (0.000, nominal_y_m - 0.020, "Raised"),
        (0.000, nominal_y_m + 0.010, "Extended"),
    ]
    print(f"  {'Pose':>16}   {'theta_top':>10}  {'theta_bot':>10}  FK err")
    print("  " + "-" * 60)
    all_ok = True
    for x_m, y_m, name in sag_tests:
        sol = sag_ik(x_m, y_m, PARAMS)
        if sol is None:
            print(f"  {name:<16}  UNREACHABLE")
            all_ok = False
            continue
        tt, tb = sol
        fk = sag_fk(tt, tb, PARAMS)
        if fk is None:
            print(f"  {name:<16}   {tt:+9.3f} deg  {tb:+9.3f} deg  FK failed")
            all_ok = False
            continue
        x_mm, y_mm = x_m * 1000, y_m * 1000
        err = sqrt((fk[0] - x_mm) ** 2 + (fk[1] - y_mm) ** 2)
        ok = "OK" if err < 0.001 else f"{err:.4f} mm"
        if err >= 0.001:
            all_ok = False
        print(f"  {name:<16}   {tt:+9.3f} deg  {tb:+9.3f} deg  {ok}")

    print(f"\n  Hip abductor IK for leg {LEG_INDEX}:")
    hip_tests = [
        (0.000, LEG_LATERAL_OFFSET, DEFAULT_BODY_Z_M, f"Nominal, {LEG_NAMES[LEG_INDEX]}"),
        (0.030, LEG_LATERAL_OFFSET, DEFAULT_BODY_Z_M + 0.005, f"Fwd 30mm, {LEG_NAMES[LEG_INDEX]}"),
        (-0.030, LEG_LATERAL_OFFSET, DEFAULT_BODY_Z_M + 0.005, f"Back 30mm, {LEG_NAMES[LEG_INDEX]}"),
    ]
    print(f"  {'Pose':>18}   {'theta_1':>9}  {'x_sag':>10}  {'y_sag':>10}")
    print("  " + "-" * 60)
    for bx, by, bz, name in hip_tests:
        r_foot = np.array([bx, by, bz])
        t1, xs, ys = hip_ik(r_foot, LEG_INDEX, PARAMS)
        print(f"  {name:<18}   {t1:+8.2f} deg  {xs * 1000:+9.2f}mm  {ys * 1000:+9.2f}mm")

    print(f"\n  {'Sagittal checks passed' if all_ok else 'Sagittal checks failed'}")
    return all_ok


def test_sweep(sim_only=False):
    """Sweep the foot through an arc of waypoints and check each IK solution."""
    from math import sqrt

    print("\n-- Sweep test --------------------------------------------------")
    nominal_y_mm = STARTUP_SAG[1]
    waypoints = [
        (0, nominal_y_mm),
        (20, nominal_y_mm - 5),
        (40, nominal_y_mm - 10),
        (40, nominal_y_mm),
        (20, nominal_y_mm + 10),
        (0, nominal_y_mm + 10),
        (-20, nominal_y_mm + 10),
        (-40, nominal_y_mm),
        (-40, nominal_y_mm - 10),
        (-20, nominal_y_mm - 5),
        (0, nominal_y_mm),
    ]
    current = None
    if not sim_only and _pca is not None:
        print("  Moving to start ...")
        current = move_to_sag(*waypoints[0], verbose=True)
        time.sleep(0.5)

    print(f"\n  {'Step':>4}  {'x':>6}  {'y':>6}  {'theta_1':>9}  {'theta_top':>10}  {'theta_bot':>10}  FK err")
    print("  " + "-" * 78)
    for i, (x, y) in enumerate(waypoints):
        sol = full_ik_from_sag(x, y)
        if sol is None:
            print(f"  {i + 1:>4}  ({x:+4.0f},{y:4.0f})  UNREACHABLE")
            continue
        t1, tt, tb = sol
        fk = sag_fk(tt, tb, PARAMS)
        r_foot = np.array([x / 1000.0, LEG_LATERAL_OFFSET, -y / 1000.0])
        _, xs, ys = hip_ik(r_foot, LEG_INDEX, PARAMS)
        err = sqrt((fk[0] - xs * 1000) ** 2 + (fk[1] - ys * 1000) ** 2) if fk else float("nan")
        ok = "OK" if err < 0.001 else f"{err:.3f} mm"
        print(
            f"  {i + 1:>4}  ({x:+4.0f},{y:4.0f})  {t1:+8.2f} deg"
            f"  {tt:+9.2f} deg  {tb:+9.2f} deg  {ok}"
        )
        if not sim_only and _pca is not None:
            current = move_to_sag(x, y, from_angles=current, verbose=False)
            time.sleep(0.3)

    if not sim_only and _pca is not None:
        print("\n  Sweep done. Releasing in 2 s ...")
        time.sleep(2)
        release()


def test_mock_walk(sim_only=False, cycles=2, from_angles=None):
    """Run a small direct-servo stepping pattern around the standing pose."""
    cycles = max(1, int(cycles))
    print("\n-- Mock walk ---------------------------------------------------")
    current = from_angles
    step = 1

    if current is None:
        current = stand_reference_angles()

    for cycle in range(cycles):
        print(f"  Cycle {cycle + 1}/{cycles}")
        for servo_target in MOCK_WALK_SERVO_WAYPOINTS:
            hip_deg, top_deg, bot_deg = servo_target
            target_angles = servo_degrees_to_angles(*servo_target)
            print(
                f"    step {step:>2}: servo=({hip_deg:5.1f}, {top_deg:5.1f}, {bot_deg:5.1f}) deg"
                f"  cmd=({target_angles[0]:+6.1f}, {target_angles[1]:+6.1f}, {target_angles[2]:+6.1f})"
            )
            if not sim_only and _pca is not None:
                current = move_to_servo_degrees(
                    servo_target,
                    from_angles=current,
                    verbose=False,
                )
                time.sleep(MOCK_WALK_DWELL_S)
            else:
                current = target_angles
            step += 1

    if not sim_only and _pca is not None:
        current = move_to_servo_degrees(
            STARTUP_SERVO_DEG,
            from_angles=current,
            verbose=False,
        )
        print("\n  Mock walk done. Standing at startup pose.")

    return current


def interactive(sim_only=False):
    global HIP_OFFSET, TOP_OFFSET, BOT_OFFSET
    print("\n-- Interactive mode -------------------------------------------")
    stand = stand_reference_angles()
    hip_us = _servo_deg_to_us(STARTUP_SERVO_DEG[0])
    top_us = _servo_deg_to_us(STARTUP_SERVO_DEG[1])
    bot_us = _servo_deg_to_us(STARTUP_SERVO_DEG[2])

    print("  Servos are PASSIVE (no PWM). Position the leg at the standing")
    print("  startup pose, then type 'hold' to lock.")
    print()
    print("  Startup stand servo angles and pulse widths:")
    print(
        f"    hip  (ch{HIP_CH}): {STARTUP_SERVO_DEG[0]:6.1f} deg  ->  {hip_us:.0f} us"
    )
    print(
        f"    top  (ch{TOP_CH}): {STARTUP_SERVO_DEG[1]:6.1f} deg  ->  {top_us:.0f} us"
    )
    print(
        f"    bot  (ch{BOT_CH}): {STARTUP_SERVO_DEG[2]:6.1f} deg  ->  {bot_us:.0f} us"
    )
    print("  Equivalent control-space angles:")
    print(f"    theta_1={stand[0]:+7.2f}  theta_top={stand[1]:+7.2f}  theta_bot={stand[2]:+7.2f}")
    print()
    print("  If 'hold' causes a servo to jump the wrong way:")
    print("    - flip its MULTIPLIER to -1 in the code")
    print("  If a servo is off by a few degrees:")
    print("    - use 'offset hip|top|bot <deg>' to adjust live")
    print("    - use 'show_offsets' to print values to hard-code")

    current = None  # No position locked yet.

    print("\n  Commands:")
    print("    hold                 lock servos at startup stand (first command!)")
    print("    servo <h> <t> <b>    move by direct servo angles in degrees")
    print("    raw <t1> <tt> <tb>   move by control-space angles")
    print("    sag <x> <y>         sagittal target in mm")
    print("    body <x> <y> <z>    body-frame target in meters")
    print("    offset <hip|top|bot> <deg>   adjust offset & re-send")
    print("    show_offsets         print current offsets for code")
    print("    ik <x> <y>          print IK only")
    print("    fk <tt> <tb>        print FK only")
    print("    sweep                run the arc sweep")
    print("    mockwalk [n]         run the direct-servo mock walk for n cycles")
    print("    stand / neutral      return to startup stand")
    print("    release              let the servos go limp")
    print("    q / quit             exit")
    print()

    while True:
        try:
            raw = input("  > ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            break

        if cmd == "sag" and len(parts) == 3:
            try:
                x_mm, y_mm = float(parts[1]), float(parts[2])
                current = move_to_sag(x_mm, y_mm, from_angles=current, verbose=True)
            except ValueError:
                print("  Usage: sag <x_mm> <y_mm>")
            continue

        if cmd == "body" and len(parts) == 4:
            try:
                bx, by, bz = float(parts[1]), float(parts[2]), float(parts[3])
                sol = solve_body_target(bx, by, bz)
                if sol is None:
                    print("  Unreachable")
                    continue
                t1_deg, tt_deg, tb_deg = sol
                print(
                    f"  IK: theta_1={t1_deg:+.2f} deg"
                    f"  theta_top={tt_deg:+.2f} deg  theta_bot={tb_deg:+.2f} deg"
                )
                current = move_to_angles(
                    (t1_deg, tt_deg, tb_deg), from_angles=current, verbose=True
                )
            except ValueError:
                print("  Usage: body <x_m> <y_m> <z_m>")
            continue

        if cmd == "raw" and len(parts) == 4:
            try:
                t1_deg, tt_deg, tb_deg = float(parts[1]), float(parts[2]), float(parts[3])
                current = move_to_angles(
                    (t1_deg, tt_deg, tb_deg), from_angles=current, verbose=True
                )
                fk = sag_fk(tt_deg, tb_deg, PARAMS)
                if fk:
                    print(f"  FK foot (sagittal): ({fk[0]:.2f}, {fk[1]:.2f}) mm")
            except ValueError:
                print("  Usage: raw <t1_deg> <tt_deg> <tb_deg>")
            continue

        if cmd == "servo" and len(parts) == 4:
            try:
                hip_deg, top_deg, bot_deg = (
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                )
                current = move_to_servo_degrees(
                    (hip_deg, top_deg, bot_deg),
                    from_angles=current,
                    verbose=True,
                )
            except ValueError:
                print("  Usage: servo <hip_deg> <top_deg> <bot_deg>")
            continue

        if cmd == "offset" and len(parts) == 3:
            try:
                which = parts[1].lower()
                val = float(parts[2])
                if which == "hip":
                    HIP_OFFSET = val
                elif which == "top":
                    TOP_OFFSET = val
                elif which == "bot":
                    BOT_OFFSET = val
                else:
                    print("  Usage: offset <hip|top|bot> <deg>")
                    continue
                print(f"  {which.upper()}_OFFSET = {val:+.1f} deg")
                # Re-send current position with new offset.
                if current is not None:
                    send_angles(*current, verbose=True)
                    print("  Re-sent current angles with updated offset.")
            except ValueError:
                print("  Usage: offset <hip|top|bot> <deg>")
            continue

        if cmd == "show_offsets":
            print(f"\n  Copy these into the script header:")
            print(f"    HIP_OFFSET = {HIP_OFFSET}")
            print(f"    TOP_OFFSET = {TOP_OFFSET}")
            print(f"    BOT_OFFSET = {BOT_OFFSET}")
            print()
            continue

        if cmd == "ik" and len(parts) == 3:
            try:
                x_mm, y_mm = float(parts[1]), float(parts[2])
                sol = full_ik_from_sag(x_mm, y_mm)
                if sol is None:
                    print(f"  ({x_mm}, {y_mm}) mm unreachable")
                    continue
                t1_deg, tt_deg, tb_deg = sol
                fk = sag_fk(tt_deg, tb_deg, PARAMS)
                print(
                    f"  IK: theta_1={t1_deg:+.2f} deg"
                    f"  theta_top={tt_deg:+.2f} deg  theta_bot={tb_deg:+.2f} deg"
                )
                if fk:
                    from math import sqrt
                    err = sqrt((fk[0] - x_mm) ** 2 + (fk[1] - y_mm) ** 2)
                    print(f"  FK check: ({fk[0]:.4f}, {fk[1]:.4f})mm  err={err:.6f}mm")
            except ValueError:
                print("  Usage: ik <x_mm> <y_mm>")
            continue

        if cmd == "fk" and len(parts) == 3:
            try:
                tt_deg, tb_deg = float(parts[1]), float(parts[2])
                fk = sag_fk(tt_deg, tb_deg, PARAMS)
                if fk:
                    print(
                        f"  FK: theta_top={tt_deg:.2f} deg"
                        f"  theta_bot={tb_deg:.2f} deg  ->  ({fk[0]:.2f}, {fk[1]:.2f})mm"
                    )
                else:
                    print("  FK: infeasible")
            except ValueError:
                print("  Usage: fk <theta_top_deg> <theta_bot_deg>")
            continue

        if cmd == "sweep":
            test_sweep(sim_only=sim_only)
            current = None
            continue

        if cmd == "mockwalk":
            try:
                cycles = 2 if len(parts) == 1 else int(parts[1])
                current = test_mock_walk(
                    sim_only=sim_only,
                    cycles=cycles,
                    from_angles=current,
                )
            except ValueError:
                print("  Usage: mockwalk [cycles]")
            continue

        if cmd in ("neutral", "stand"):
            current = move_to_servo_degrees(
                STARTUP_SERVO_DEG,
                from_angles=current,
                verbose=True,
            )
            continue

        if cmd == "hold":
            if current is None:
                # First hold — send the standing startup pose to lock servos.
                current = stand_reference_angles()
            send_angles(*current, verbose=True)
            print("  Servos locked.")
            continue

        if cmd == "release":
            release()
            current = None
            continue

        print(f"  Unknown: {raw}")

    if not sim_only and _pca is not None:
        print("\n  Releasing ...")
        release()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Argos 3-DOF single-leg test")
    parser.add_argument("--sim", action="store_true", help="Simulation only (no I2C)")
    parser.add_argument("--math", action="store_true", help="Run the IK/FK self-test and exit")
    parser.add_argument("--sweep", action="store_true", help="Run the arc sweep and exit")
    parser.add_argument(
        "--sag", nargs=2, type=float, metavar=("X", "Y"),
        help="Move foot to sagittal (X Y) mm and exit",
    )
    parser.add_argument(
        "--goto", nargs=3, type=float, metavar=("X", "Y", "Z"),
        help="Move foot to body-frame (X Y Z) m and exit",
    )
    parser.add_argument(
        "--raw", nargs=3, type=float, metavar=("T1", "TT", "TB"),
        help="Send raw angles (theta_1 theta_top theta_bot in degrees) and exit",
    )
    parser.add_argument(
        "--servo",
        nargs=3,
        type=float,
        metavar=("HIP", "TOP", "BOT"),
        help="Send direct servo angles in degrees and exit",
    )
    parser.add_argument(
        "--mock-walk",
        nargs="?",
        const=2,
        type=int,
        metavar="CYCLES",
        help="Run the direct-servo mock walk routine and exit",
    )
    args = parser.parse_args()

    leg_name = LEG_NAMES[LEG_INDEX]
    crouch_t1_deg, crouch_tt_deg, crouch_tb_deg = crouch_reference_angles()
    stand_t1_deg, stand_tt_deg, stand_tb_deg = stand_reference_angles()

    print("=" * 62)
    print("  Argos 3-DOF Single-Leg Test")
    print(f"  Leg index   : {LEG_INDEX} ({leg_name})")
    print(f"  Channels    : hip={HIP_CH}  top={TOP_CH}  bot={BOT_CH}")
    print(f"  Servo travel: {SERVO_RANGE_DEG:.0f} deg  center={SERVO_CENTER_DEG:.1f} deg")
    print(
        "  Startup stand servo: "
        f"hip={STARTUP_SERVO_DEG[0]:.1f}  top={STARTUP_SERVO_DEG[1]:.1f}  bot={STARTUP_SERVO_DEG[2]:.1f}"
    )
    print(
        "  Startup stand cmd : "
        f"theta_1={stand_t1_deg:+.2f}  "
        f"theta_top={stand_tt_deg:+.2f}  "
        f"theta_bot={stand_tb_deg:+.2f}"
    )
    print(f"  Startup sag : ({STARTUP_SAG[0]}, {STARTUP_SAG[1]}) mm")
    print(
        "  IK ref pose : "
        f"theta_1={crouch_t1_deg:+.2f}  "
        f"theta_top={crouch_tt_deg:+.2f}  "
        f"theta_bot={crouch_tb_deg:+.2f}"
    )
    print(
        f"  L1_hip={PARAMS['L1_hip'] * 1000:.1f}mm"
        f"  L_upper={PARAMS['L1'] * 1000:.1f}mm"
        f"  L_lower={PARAMS['L2'] * 1000:.1f}mm"
    )
    print(
        f"  BC left={PARAMS['rbl'] * 1000:.1f}mm"
        f"  right={PARAMS['rbr'] * 1000:.1f}mm"
        f"  alpha={degrees(PARAMS['abc']):.0f} deg"
    )
    print(
        f"  Rod1={PARAMS['Lr1'] * 1000:.1f}mm"
        f"  Rod2={PARAMS['Lr2'] * 1000:.1f}mm"
        f"  horn={PARAMS['rh'] * 1000:.1f}mm"
        f"  dko={PARAMS['dko'] * 1000:.1f}mm"
    )
    print("=" * 62)

    if not test_math():
        print("\n  IK self-test failed. Check PARAMS before moving hardware.")
        sys.exit(1)

    if args.math:
        return

    sim_only = args.sim
    if not sim_only:
        sim_only = not init_hardware()

    if args.sag:
        x_mm, y_mm = args.sag
        print(f"\n  Moving to sagittal ({x_mm:.0f}, {y_mm:.0f}) mm ...")
        move_to_sag(x_mm, y_mm, verbose=True)
        if not sim_only:
            time.sleep(2)
            release()
        return

    if args.goto:
        bx, by, bz = args.goto
        sol = solve_body_target(bx, by, bz)
        if sol is None:
            print("  Unreachable")
            return
        t1_deg, tt_deg, tb_deg = sol
        print(
            f"  Body ({bx:.3f}, {by:.3f}, {bz:.3f})m ->"
            f"  theta_1={t1_deg:+.2f} deg  theta_top={tt_deg:+.2f} deg  theta_bot={tb_deg:+.2f} deg"
        )
        move_to_angles((t1_deg, tt_deg, tb_deg), verbose=True)
        if not sim_only:
            time.sleep(2)
            release()
        return

    if args.raw:
        t1_deg, tt_deg, tb_deg = args.raw
        move_to_angles((t1_deg, tt_deg, tb_deg), verbose=True)
        if not sim_only:
            time.sleep(2)
            release()
        return

    if args.servo:
        hip_deg, top_deg, bot_deg = args.servo
        move_to_servo_degrees((hip_deg, top_deg, bot_deg), verbose=True)
        if not sim_only:
            time.sleep(2)
            release()
        return

    if args.sweep:
        test_sweep(sim_only=sim_only)
        return

    if args.mock_walk is not None:
        test_mock_walk(sim_only=sim_only, cycles=args.mock_walk)
        if not sim_only:
            time.sleep(2)
            release()
        return

    interactive(sim_only=sim_only)


if __name__ == "__main__":
    main()
