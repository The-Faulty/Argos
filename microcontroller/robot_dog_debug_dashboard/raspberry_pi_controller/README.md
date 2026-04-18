# Raspberry Pi Controller

This package runs the robot dog controller directly on a Raspberry Pi and drives the PCA9685 over I2C instead of sending commands to a microcontroller over serial.

## What it provides

- The same HTTP and WebSocket API shape as the existing dashboard bridge
- Direct PCA9685 output on the Raspberry Pi
- The same leg kinematics, animation playback, built-in walk/crouch motions, joint limits, and panic release behavior
- A dry-run mode for desktop testing without I2C hardware

## Run on a Raspberry Pi

1. Enable I2C on the Pi.
2. Connect the PCA9685 to the Pi I2C bus.
3. Install Node.js 20+.
4. From this folder, run:

```bash
./launch_pi_controller.sh
```

## Useful environment variables

- `PORT=8787`
- `PI_I2C_BUS=1`
- `PI_PCA9685_ADDRESS=0x40`
- `PI_PCA9685_FREQUENCY=50`
- `PI_SERVO_PWM_MIN_TICKS=102`
- `PI_SERVO_PWM_MAX_TICKS=512`
- `PI_SERVO_SUPPLY_VOLTS=7.4`
- `PI_DOG_DRY_RUN=1` to skip real I2C access

## Dashboard integration

Point the dashboard at the Raspberry Pi server instead of the serial bridge. The Pi controller exposes:

- `GET /api/status`
- `POST /api/connect`
- `POST /api/disconnect`
- `POST /api/command`
- `POST /api/animations`
- `POST /api/animations/:id/play`
- `POST /api/animations/:id/stop`
- `WS /telemetry`
