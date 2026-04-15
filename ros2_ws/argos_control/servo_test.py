"""Argos servo tester.

Set any PCA9685 channel to any angle and hold it so you can
attach servo horns and the chassis while the servo is locked.

Usage:
    python ServoTest.py
"""

import sys
import time

PCA9685_I2C_ADDRESS = 0x40
PWM_MIN_US = 370
PWM_MAX_US = 2400
PWM_FREQ = 50
SERVO_RANGE_DEG = 180  # 180-degree servos

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
    """Send an angle (0-180) to a channel."""
    angle_deg = max(0.0, min(SERVO_RANGE_DEG, angle_deg))
    pulse_us = PWM_MIN_US + (angle_deg / SERVO_RANGE_DEG) * (PWM_MAX_US - PWM_MIN_US)
    period_us = 1_000_000.0 / PWM_FREQ
    duty = int(round(pulse_us / period_us * 65535))
    _pca.channels[channel].duty_cycle = duty
    return angle_deg, pulse_us


def release(channel):
    _pca.channels[channel].duty_cycle = 0


def release_all():
    for ch in range(16):
        _pca.channels[ch].duty_cycle = 0


def main():
    print("=" * 50)
    print("  Argos Servo Tester")
    print("=" * 50)

    try:
        init_pca()
    except Exception as e:
        print(f"  PCA9685 init failed: {e}")
        sys.exit(1)

    print()
    print("  Commands:")
    print("    <ch> <angle>    set channel to angle (0-180)")
    print("    <ch> release    release channel (go limp)")
    print("    all release     release all channels")
    print("    q               quit (releases all)")
    print()
    center_deg = SERVO_RANGE_DEG / 2.0
    center_us = PWM_MIN_US + (center_deg / SERVO_RANGE_DEG) * (PWM_MAX_US - PWM_MIN_US)
    print(f"  180-deg servos: center = {center_deg:.0f} deg  ({center_us:.0f} us)")
    print()
    print("  Examples:")
    print("    0 90     -> ch0 to center")
    print("    0 0      -> ch0 to 0 deg")
    print("    1 90     -> ch1 to center")
    print("    0 release -> ch0 go limp")
    print()

    while True:
        try:
            raw = input("  > ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if not raw:
            continue

        parts = raw.split()

        if parts[0].lower() in ("q", "quit", "exit"):
            break

        if parts[0].lower() == "all" and len(parts) == 2 and parts[1].lower() == "release":
            release_all()
            print("  All channels released.")
            continue

        if len(parts) != 2:
            print("  Usage: <ch> <angle>  or  <ch> release")
            continue

        try:
            ch = int(parts[0])
        except ValueError:
            print("  Channel must be a number (0-15)")
            continue

        if ch < 0 or ch > 15:
            print("  Channel must be 0-15")
            continue

        if parts[1].lower() == "release":
            release(ch)
            print(f"  ch{ch} released (limp)")
            continue

        try:
            angle = float(parts[1])
        except ValueError:
            print("  Angle must be a number (0-180)")
            continue

        angle, pulse_us = set_angle(ch, angle)
        print(f"  ch{ch} -> {angle:.1f} deg  ({pulse_us:.0f} us)  LOCKED")

    print("\n  Releasing all ...")
    release_all()
    print("  Done.")


if __name__ == "__main__":
    main()
