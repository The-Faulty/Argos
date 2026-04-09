"""Single-leg bench test for Argos.

This script talks straight to the PCA9685 from a Raspberry Pi so you can
check one leg without bringing up ROS.
"""

import argparse
import sys
import time
from math import pi, acos, atan2, sqrt, sin, cos, fmod, asin, radians, degrees

import numpy as np


PCA9685_I2C_ADDRESS = 0x40

# PCA9685 channel assignments.
HIP_CH = 2
TOP_CH = 1
BOT_CH = 0

# Pulse limits from the servo datasheet.
PWM_MIN_US = 370
PWM_MAX_US = 2400
PWM_FREQ = 50

# Flip a multiplier if a servo runs backward.
# Adjust an offset if neutral is not centered.
HIP_OFFSET = 0.0
HIP_MULTIPLIER = +1

TOP_OFFSET = 0.0
TOP_MULTIPLIER = +1

BOT_OFFSET = 0.0
BOT_MULTIPLIER = +1

# 0=FR, 1=FL, 2=RR, 3=RL
LEG_INDEX = 0

# All distances are in meters. See Config.py for measurement notes.
PARAMS = dict(
    L1_hip=0.05162,
    phi=radians(73.917),
    L1=0.130,
    L2=0.150,
    rbl=0.035,
    rbr=0.050,
    abc=radians(60.0),
    Lr1=0.065,
    Lr2=0.120,
    rh=0.035,
    dko=0.030,
)

LEG_LATERAL_OFFSET = -0.061

# Motion settings.
STEPS_PER_MOVE = 30
STEP_DELAY_S = 0.018
STARTUP_SAG = (0, 200)


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


def full_ik_from_sag(x_mm, y_mm, lateral_m=None):
    """Solve the full 3-DOF IK from a sagittal target."""
    if lateral_m is None:
        lateral_m = LEG_LATERAL_OFFSET
    r_foot = np.array([x_mm / 1000.0, lateral_m, -y_mm / 1000.0])
    t1_deg, x_sag, y_sag = hip_ik(r_foot, LEG_INDEX, PARAMS)
    sol = sag_ik(x_sag, y_sag, PARAMS)
    if sol is None:
        return None
    return t1_deg, sol[0], sol[1]


def solve_body_target(bx, by, bz):
    """Solve IK for a body-frame target."""
    r_foot = np.array([bx, by, bz])
    t1_deg, x_sag, y_sag = hip_ik(r_foot, LEG_INDEX, PARAMS)
    sol = sag_ik(x_sag, y_sag, PARAMS)
    if sol is None:
        return None
    return t1_deg, sol[0], sol[1]


def _ik_to_servo_us(joint_angle_deg, offset_deg, multiplier):
    """Convert an IK angle into a pulse width in microseconds."""
    cmd = 90.0 + multiplier * joint_angle_deg + offset_deg
    cmd = max(0.0, min(180.0, cmd))
    return PWM_MIN_US + (cmd / 180.0) * (PWM_MAX_US - PWM_MIN_US)


def _us_to_duty(pulse_us):
    """Convert pulse width in microseconds to PCA9685 duty cycle."""
    period_us = 1_000_000.0 / PWM_FREQ
    return int(round(pulse_us / period_us * 65535))


_pca = None


def init_hardware():
    """Initialize the PCA9685. Returns True on success."""
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
    """Move to a known angle triplet, interpolating when hardware is active."""
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


def move_to_sag(x_mm, y_mm, from_angles=None, verbose=True):
    """Move the foot to a sagittal target."""
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
    if _pca is None:
        return
    for channel in [HIP_CH, TOP_CH, BOT_CH]:
        _pca.channels[channel].duty_cycle = 0
    print("  Servos released.")


def test_math():
    print("\n-- IK/FK self-test ---------------------------------------------")
    print("  Sagittal IK/FK:")
    sag_tests = [
        (0.000, 0.200, "Nominal"),
        (0.040, 0.190, "Fwd +40mm"),
        (0.080, 0.170, "Wide fwd"),
        (-0.040, 0.190, "Back -40mm"),
        (-0.080, 0.160, "Wide back"),
        (0.000, 0.170, "Raised"),
        (0.000, 0.230, "Extended"),
    ]
    print(f"  {'Pose':>16}   {'theta_top':>10}  {'theta_bot':>10}  FK err")
    print("  " + "-" * 60)
    all_ok = True
    for x_m, y_m, name in sag_tests:
        sol = sag_ik(x_m, y_m, PARAMS)
        if sol is None:
            print(f"  {name:<16}  UNREACHABLE")
            continue
        tt, tb = sol
        fk = sag_fk(tt, tb, PARAMS)
        x_mm, y_mm = x_m * 1000, y_m * 1000
        err = sqrt((fk[0] - x_mm) ** 2 + (fk[1] - y_mm) ** 2) if fk else float("nan")
        ok = "OK" if err < 0.001 else f"{err:.4f} mm"
        if err >= 0.001:
            all_ok = False
        print(f"  {name:<16}   {tt:+9.3f} deg  {tb:+9.3f} deg  {ok}")

    print(f"\n  Hip abductor IK for leg {LEG_INDEX}:")
    hip_tests = [
        (0.000, -0.061, -0.200, "Nominal, FR"),
        (0.050, -0.061, -0.190, "Fwd 50mm, FR"),
        (-0.040, -0.061, -0.190, "Back 40mm, FR"),
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
    print("\n-- Sweep test --------------------------------------------------")
    waypoints = [
        (0, 200),
        (40, 190),
        (80, 170),
        (80, 200),
        (40, 210),
        (0, 210),
        (-40, 210),
        (-80, 200),
        (-80, 170),
        (-40, 190),
        (0, 200),
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


def interactive(sim_only=False):
    print("\n-- Interactive mode -------------------------------------------")
    print("  Commands:")
    print("    sag <x> <y>         sagittal target in mm")
    print("    body <x> <y> <z>    body-frame target in meters")
    print("    raw <t1> <tt> <tb>  raw angles in degrees")
    print("    ik <x> <y>          print IK only")
    print("    fk <tt> <tb>        print FK only")
    print("    sweep               run the arc sweep")
    print("    neutral             return to nominal (0, 200)")
    print("    release             let the servos go limp")
    print("    q / quit            exit")
    print()

    current = None
    if not sim_only and _pca is not None:
        print("  Moving to neutral ...")
        current = move_to_sag(*STARTUP_SAG, verbose=True)
        time.sleep(0.5)

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
                    (t1_deg, tt_deg, tb_deg),
                    from_angles=current,
                    verbose=True,
                )
            except ValueError:
                print("  Usage: body <x_m> <y_m> <z_m>")
            continue

        if cmd == "raw" and len(parts) == 4:
            try:
                t1_deg, tt_deg, tb_deg = (
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                )
                current = move_to_angles(
                    (t1_deg, tt_deg, tb_deg),
                    from_angles=current,
                    verbose=True,
                )
                fk = sag_fk(tt_deg, tb_deg, PARAMS)
                if fk:
                    print(f"  FK foot (sagittal): ({fk[0]:.2f}, {fk[1]:.2f}) mm")
            except ValueError:
                print("  Usage: raw <t1_deg> <tt_deg> <tb_deg>")
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

        if cmd == "neutral":
            current = move_to_sag(*STARTUP_SAG, from_angles=current, verbose=True)
            continue

        if cmd == "release":
            release()
            current = None
            continue

        print(f"  Unknown: {raw}")

    if not sim_only and _pca is not None:
        print("\n  Releasing ...")
        release()


def main():
    parser = argparse.ArgumentParser(description="Argos 3-DOF single-leg test")
    parser.add_argument("--sim", action="store_true", help="Simulation only (no I2C)")
    parser.add_argument("--math", action="store_true", help="Run the IK/FK self-test and exit")
    parser.add_argument("--sweep", action="store_true", help="Run the arc sweep and exit")
    parser.add_argument(
        "--sag",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Move the sagittal foot target to (X Y) mm and exit",
    )
    parser.add_argument(
        "--goto",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Move the body-frame foot target to (X Y Z) m and exit",
    )
    parser.add_argument(
        "--raw",
        nargs=3,
        type=float,
        metavar=("T1", "TT", "TB"),
        help="Send raw angles (theta_1 theta_top theta_bot in degrees) and exit",
    )
    args = parser.parse_args()

    leg_name = "FR" if LEG_INDEX == 0 else "FL" if LEG_INDEX == 1 else "RR" if LEG_INDEX == 2 else "RL"

    print("=" * 62)
    print("  Argos 3-DOF Single-Leg Test")
    print(f"  Leg index   : {LEG_INDEX} ({leg_name})")
    print(f"  Channels    : hip={HIP_CH}  top={TOP_CH}  bot={BOT_CH}")
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

    if args.sweep:
        test_sweep(sim_only=sim_only)
        return

    interactive(sim_only=sim_only)


if __name__ == "__main__":
    main()
