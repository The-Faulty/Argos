# Argos Servo Firmware

Arduino sketch that runs on the on-board ESP32 (ESP32-S3 dev board, the one wired to the PCA9685 servo driver). Originally from the `andy-servo-control` branch — this is the hardware path proven to actually move the joints.

## What it does

- Listens on USB serial @ **921600 baud** for newline-delimited JSON commands.
- Drives 12 servos through a PCA9685 over I2C (SDA = GPIO 6, SCL = GPIO 7).
- Runs per-leg inverse kinematics for foot-XY targets and per-leg smooth servo interpolation respecting per-axis speed limits.
- Reads an analog gas sensor on `GAS_ADC_PIN` (default GPIO 1) and emits the raw 12-bit value with each telemetry frame.
- Periodically broadcasts a `{"type":"state",...}` JSON line with mode, per-leg desired/current servo angles, joint limits, and gas reading.

## Wire format

Pi → ESP32 (one JSON object per line):

| Type | Fields |
| --- | --- |
| `hello` | seq |
| `get_state` | seq |
| `set_mode` | mode, seq |
| `set_servo_update_rate_hz` | hz (40–200), seq |
| `release_servos` | seq |
| `set_leg_servo_angles` | legId, hipServoDeg, thighServoDeg, calfServoDeg (each 0–180), seq |
| `set_leg_foot_xy` | legId, x, y, seq |
| `set_leg_joint_angles` | legId, thighDeg, calfDeg, seq |
| `set_leg_servo_channel_map` | legId, hipChannel, thighChannel, calfChannel, seq |
| `set_leg_joint_limits` | legId, thighMinDeg, thighMaxDeg, calfMinDeg, calfMaxDeg, seq |
| `set_leg_servo_speed_limit` | legId, hipDegPerSec, thighDegPerSec, calfDegPerSec, seq |
| `run_builtin` | builtin (`walk`/`crouch`), seq |
| `play_animation` / `pause_animation` / `stop_animation` / `upload_animation` | … |

Leg IDs: `front_left`, `front_right`, `rear_left`, `rear_right`.

ESP32 → Pi: `{"type":"hello_ack"|"ack"|"state"|"error",...}` lines. The Pi server just parses the JSON; see [`dashboard/pi_server/serial_bridge.js`](../../dashboard/pi_server/serial_bridge.js) for the consumer.

## Build & flash

1. Install Arduino IDE 2.x.
2. Boards Manager → install **esp32 by Espressif** (≥ 2.0.14).
3. Library Manager → install **Adafruit PWM Servo Driver Library**.
4. Open `argos_servo.ino`. Select the right ESP32-S3 board variant + port.
5. Upload. Open Serial Monitor at 921600 baud — you should see periodic `{"type":"state",...}` lines.

## Pin reference

| Signal | GPIO |
| --- | --- |
| I2C SDA (PCA9685) | 6 |
| I2C SCL (PCA9685) | 7 |
| Gas analog | 1 (configurable: `GAS_ADC_PIN`) |

If your physical board wires the gas sensor to a different ADC pin, change `GAS_ADC_PIN` near the top of `argos_servo.ino` and reflash.
