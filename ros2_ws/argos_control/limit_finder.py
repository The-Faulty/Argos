"""Argos joint limit finder.

Explore the coupled top/bottom servo workspace to find safe joint limits.
Hip is independent so you just sweep it. For top+bottom, this script lets
you nudge each independently and mark limit points when you feel binding
or collision.

Usage:
    python LimitFinder.py
"""

import sys
import time

PCA9685_I2C_ADDRESS = 0x40
PWM_MIN_US = 370
PWM_MAX_US = 2400
PWM_FREQ = 50
SERVO_RANGE_DEG = 180

# Channel assignments — match your wiring.
HIP_CH = 2
TOP_CH = 1
BOT_CH = 0

CENTER = SERVO_RANGE_DEG / 2.0  # 90

_pca = None


def init_pca():
    global _pca
    import board, busio
    from adafruit_pca9685 import PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    _pca = PCA9685(i2c, address=PCA9685_I2C_ADDRESS)
    _pca.frequency = PWM_FREQ
    print(f"  PCA9685 OK (0x{PCA9685_I2C_ADDRESS:02X})")


def set_angle(channel, angle_deg):
    angle_deg = max(0.0, min(SERVO_RANGE_DEG, angle_deg))
    pulse_us = PWM_MIN_US + (angle_deg / SERVO_RANGE_DEG) * (PWM_MAX_US - PWM_MIN_US)
    period_us = 1_000_000.0 / PWM_FREQ
    duty = int(round(pulse_us / period_us * 65535))
    _pca.channels[channel].duty_cycle = duty
    return angle_deg


def release_ch(channel):
    _pca.channels[channel].duty_cycle = 0


def release_all():
    for ch in range(16):
        _pca.channels[ch].duty_cycle = 0


def print_state(hip, top, bot):
    print(f"    HIP(ch{HIP_CH})={hip:6.1f} deg   "
          f"TOP(ch{TOP_CH})={top:6.1f} deg   "
          f"BOT(ch{BOT_CH})={bot:6.1f} deg")


def print_marks(marks):
    if not marks:
        print("    (none)")
        return
    for i, m in enumerate(marks):
        print(f"    [{i}] {m['label']:>12}  "
              f"hip={m['hip']:6.1f}  top={m['top']:6.1f}  bot={m['bot']:6.1f}")


def find_hip_limits():
    """Sweep hip independently to find min/max."""
    print("\n" + "=" * 60)
    print("  PHASE 1: Hip limits (ch{})".format(HIP_CH))
    print("=" * 60)
    print("  Top and bot are at center (90). Sweep hip only.")
    print()
    print("  Commands:")
    print("    <angle>        set hip to angle")
    print("    +<step>        nudge hip up     (e.g. +5)")
    print("    -<step>        nudge hip down   (e.g. -5)")
    print("    mark <label>   save current position (e.g. mark min, mark max)")
    print("    done           finish hip, move to top/bot")
    print()

    hip = CENTER
    top = CENTER
    bot = CENTER
    set_angle(HIP_CH, hip)
    set_angle(TOP_CH, top)
    set_angle(BOT_CH, bot)
    marks = []

    print("  Current:")
    print_state(hip, top, bot)
    print()

    while True:
        try:
            raw = input("  hip> ").strip()
        except (KeyboardInterrupt, EOFError):
            return None

        if not raw:
            continue

        if raw.lower() == "done":
            return marks

        if raw.lower().startswith("mark"):
            parts = raw.split(maxsplit=1)
            label = parts[1] if len(parts) > 1 else f"point_{len(marks)}"
            marks.append({"label": label, "hip": hip, "top": top, "bot": bot})
            print(f"  Marked '{label}' at hip={hip:.1f}")
            continue

        try:
            if raw.startswith("+") or raw.startswith("-"):
                hip += float(raw)
            else:
                hip = float(raw)
            hip = max(0.0, min(SERVO_RANGE_DEG, hip))
            set_angle(HIP_CH, hip)
            print_state(hip, top, bot)
        except ValueError:
            print("  Enter an angle, +/-step, 'mark <label>', or 'done'")


def find_sagittal_limits():
    """Explore the coupled top/bottom workspace."""
    print("\n" + "=" * 60)
    print("  PHASE 2: Top/Bot coupled limits (ch{}, ch{})".format(TOP_CH, BOT_CH))
    print("=" * 60)
    print("  Hip stays at center. Nudge top and bot to find where")
    print("  the linkage binds, collides, or overextends.")
    print()
    print("  Commands:")
    print("    t <angle>      set top to angle")
    print("    t +/-<step>    nudge top")
    print("    b <angle>      set bot to angle")
    print("    b +/-<step>    nudge bot")
    print("    mark <label>   save current (top, bot) as a limit point")
    print("    marks          show all saved points")
    print("    center         return both to 90")
    print("    release        go limp (feel the linkage by hand)")
    print("    lock           re-engage at current angles")
    print("    done           finish")
    print()
    print("  Suggested workflow:")
    print("    1. Hold top at center, sweep bot to find its min & max")
    print("    2. Move top +20, sweep bot again — mark limits")
    print("    3. Move top -20, sweep bot again — mark limits")
    print("    4. Repeat a few times to map the safe workspace")
    print()

    hip = CENTER
    top = CENTER
    bot = CENTER
    set_angle(HIP_CH, hip)
    set_angle(TOP_CH, top)
    set_angle(BOT_CH, bot)
    engaged = True
    marks = []

    print("  Current:")
    print_state(hip, top, bot)
    print()

    while True:
        try:
            raw = input("  tb> ").strip()
        except (KeyboardInterrupt, EOFError):
            return None

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd == "done":
            return marks

        if cmd == "center":
            top, bot = CENTER, CENTER
            set_angle(TOP_CH, top)
            set_angle(BOT_CH, bot)
            engaged = True
            print_state(hip, top, bot)
            continue

        if cmd == "release":
            release_ch(TOP_CH)
            release_ch(BOT_CH)
            engaged = False
            print("  Top & bot released (limp). Move linkage by hand.")
            continue

        if cmd == "lock":
            set_angle(TOP_CH, top)
            set_angle(BOT_CH, bot)
            engaged = True
            print("  Re-engaged at:")
            print_state(hip, top, bot)
            continue

        if cmd == "marks":
            print_marks(marks)
            continue

        if cmd == "mark":
            label = parts[1] if len(parts) > 1 else f"point_{len(marks)}"
            marks.append({"label": label, "hip": hip, "top": top, "bot": bot})
            print(f"  Marked '{label}' at top={top:.1f} bot={bot:.1f}")
            continue

        if cmd in ("t", "top") and len(parts) == 2:
            try:
                val = parts[1]
                if val.startswith("+") or val.startswith("-"):
                    top += float(val)
                else:
                    top = float(val)
                top = max(0.0, min(SERVO_RANGE_DEG, top))
                if engaged:
                    set_angle(TOP_CH, top)
                print_state(hip, top, bot)
            except ValueError:
                print("  Usage: t <angle> or t +/-<step>")
            continue

        if cmd in ("b", "bot") and len(parts) == 2:
            try:
                val = parts[1]
                if val.startswith("+") or val.startswith("-"):
                    bot += float(val)
                else:
                    bot = float(val)
                bot = max(0.0, min(SERVO_RANGE_DEG, bot))
                if engaged:
                    set_angle(BOT_CH, bot)
                print_state(hip, top, bot)
            except ValueError:
                print("  Usage: b <angle> or b +/-<step>")
            continue

        print("  Unknown. Use t/b <angle>, mark, marks, center, release, lock, done")


def main():
    print("=" * 60)
    print("  Argos Joint Limit Finder")
    print("=" * 60)
    print(f"  Channels: hip=ch{HIP_CH}  top=ch{TOP_CH}  bot=ch{BOT_CH}")
    print(f"  Servo range: 0-{SERVO_RANGE_DEG} deg  center={CENTER:.0f} deg")

    try:
        init_pca()
    except Exception as e:
        print(f"  PCA9685 init failed: {e}")
        sys.exit(1)

    release_all()

    # Phase 1: hip
    hip_marks = find_hip_limits()
    if hip_marks is None:
        print("\n  Aborted.")
        release_all()
        return

    # Phase 2: top/bot
    sag_marks = find_sagittal_limits()
    if sag_marks is None:
        print("\n  Aborted.")
        release_all()
        return

    release_all()

    # Summary
    print("\n" + "=" * 60)
    print("  RESULTS")
    print("=" * 60)

    if hip_marks:
        print("\n  Hip marks:")
        print_marks(hip_marks)

        hip_vals = [m["hip"] for m in hip_marks]
        print(f"\n  Hip range: {min(hip_vals):.1f} - {max(hip_vals):.1f} deg")
        print(f"  Hip range from center: {min(hip_vals) - CENTER:+.1f} to {max(hip_vals) - CENTER:+.1f} deg")
    else:
        print("\n  No hip marks recorded.")

    if sag_marks:
        print("\n  Top/Bot marks:")
        print_marks(sag_marks)

        top_vals = [m["top"] for m in sag_marks]
        bot_vals = [m["bot"] for m in sag_marks]
        print(f"\n  Top range: {min(top_vals):.1f} - {max(top_vals):.1f} deg")
        print(f"  Top range from center: {min(top_vals) - CENTER:+.1f} to {max(top_vals) - CENTER:+.1f} deg")
        print(f"\n  Bot range: {min(bot_vals):.1f} - {max(bot_vals):.1f} deg")
        print(f"  Bot range from center: {min(bot_vals) - CENTER:+.1f} to {max(bot_vals) - CENTER:+.1f} deg")

        print("\n  NOTE: top and bot limits are coupled! The ranges above are")
        print("  the outer envelope. Not all (top, bot) combinations are safe.")
        print("  Check your marked points to see which pairs are valid.")
    else:
        print("\n  No top/bot marks recorded.")

    print()


if __name__ == "__main__":
    main()