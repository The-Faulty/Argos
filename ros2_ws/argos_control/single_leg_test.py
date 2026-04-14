"""Single-leg bench test for Argos.

Talks straight to the PCA9685 over I2C so you can check one leg without
bringing up ROS.  Run with --sim if no hardware is attached.
"""

import argparse
import sys
import time
from math import pi, radians, degrees

import numpy as np

# Import geometry and IK/FK from the main library so this file stays in sync
# with Config.py.  If the hardware changes, update Config.py — not this file.
try:
    from .Config import Configuration
    from .Kinematics import _hip_abductor_ik, _sagittal_ik, leg_fk as _leg_fk
except ImportError:
    from Config import Configuration
    from Kinematics import _hip_abductor_ik, _sagittal_ik, leg_fk as _leg_fk


PCA9685_I2C_ADDRESS = 0x40

# PCA9685 channel assignments for this test leg.
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

# Pull geometry straight from Config so there's one source of truth.
_config = Configuration()
PARAMS = _config.leg_params
LEG_LATERAL_OFFSET = -_config.LEG_LR

# Motion settings.
STEPS_PER_MOVE = 30
STEP_DELAY_S = 0.018
STARTUP_SAG = (0, 200)   # (x_mm, y_mm) for the nominal neutral pose


# ── Unit-converting wrappers around Kinematics.py ─────────────────────────────
# The ROS nodes work in radians and meters.  This script uses degrees and
# millimeters so the interactive REPL stays easy to read.

def hip_ik(r_foot, leg_index, params):
    """Hip abductor IK. Returns (theta_1_deg, x_sag_m, y_sag_m)."""
    t1_rad, x_sag, y_sag = _hip_abductor_ik(
        r_foot, leg_index, params["L1_hip"], params["phi"]
    )
    return degrees(t1_rad), x_sag, y_sag


def sag_ik(x_m, y_m, params):
    """Sagittal linkage IK. Returns (theta_top_deg, theta_bot_deg) or None."""
    result = _sagittal_ik(x_m, y_m, params)
    if result is None:
        return None
    t_upper, t_bot = result
    return degrees(t_upper), degrees(t_bot)


def sag_fk(theta_top_deg, theta_bot_deg, params):
    """Sagittal FK. Returns (x_mm, y_mm) or None."""
    result = _leg_fk(radians(theta_top_deg), radians(theta_bot_deg), params)
    if result is None:
        return None
    x_m, y_m = result
    return x_m * 1000.0, y_m * 1000.0


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


# ── Servo driver ──────────────────────────────────────────────────────────────

def _ik_to_servo_us(joint_angle_deg, offset_deg, multiplier):
    """Convert an IK angle to a pulse width in microseconds."""
    cmd = 90.0 + multiplier * joint_angle_deg + offset_deg
    cmd = max(0.0, min(180.0, cmd))
    return PWM_MIN_US + (cmd / 180.0) * (PWM_MAX_US - PWM_MIN_US)


def _us_to_duty(pulse_us):
    """Convert pulse width in microseconds to PCA9685 duty cycle (0–65535)."""
    period_us = 1_000_000.0 / PWM_FREQ
    return int(round(pulse_us / period_us * 65535))


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
        (0.000, -_config.LEG_LR, -0.200, "Nominal, FR"),
        (0.050, -_config.LEG_LR, -0.190, "Fwd 50mm, FR"),
        (-0.040, -_config.LEG_LR, -0.190, "Back 40mm, FR"),
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
    waypoints = [
        (0, 200), (40, 190), (80, 170), (80, 200), (40, 210),
        (0, 210), (-40, 210), (-80, 200), (-80, 170), (-40, 190), (0, 200),
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


# ── Interactive REPL ──────────────────────────────────────────────────────────

def interactive(sim_only=False):
    """Run the interactive command loop."""
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
    args = parser.parse_args()

    leg_name = {0: "FR", 1: "FL", 2: "RR", 3: "RL"}.get(LEG_INDEX, "?")

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
