# Argos Leg Controller

Argos is a quadruped robot dog control stack for a 4-leg, 12-servo platform. This repository contains the browser dashboards, shared kinematics and gait logic, desktop serial bridge code, and the active microcontroller firmware in [`../firmware/argos_servo`](../firmware/argos_servo) that drives the PCA9685 servo controller.

The project is built around a custom leg linkage: each leg has hip yaw, thigh, and calf motion, with the calf driven through a servo horn, short link, bell crank, and long link. The dashboard and desktop bridge calculate foot targets, joint targets, gait poses, and animation keyframes into servo angles before anything is sent to the ESP32.

## Capabilities

- Browser dashboard for teleoperation, calibration, servo tuning, and animation preview/playback.
- Serial control of the active `firmware/argos_servo` target at `921600` baud.
- PCA9685 servo output at I2C address `0x40` and `150 Hz` servo refresh.
- Four legs, three servo channels per leg: hip yaw, thigh, calf.
- Per-leg servo channel mapping, joint limits, speed limits, and neutral trim.
- Host-side foot-space inverse kinematics for the linked thigh/calf mechanism.
- Direct dashboard controls for foot XY, joint angles, and servo angles, all serialized to servo-angle commands.
- Host-side gait helpers that stream servo-angle full-body poses.
- Full-body animation clips uploaded with resolved per-servo keyframes.
- JSON-over-serial protocol and HTTP/WebSocket bridge API.
- Panic servo release command that disables PCA9685 outputs.

## Repository Layout

```text
leg_controller/
  README.md
  launch_robot_dog_debug_dashboard.ps1/.cmd
  ../firmware/
    argos_servo/
      argos_servo.ino
  robot_dog_leg_kinematics_visualization.jsx
  robot_dog_debug_dashboard/
    src/                         React dashboard
    shared/                      Kinematics, gait, protocol, animation helpers
    bridge/server.js             Desktop serial bridge
    robot_dog_leg_smooth_interpolated/  Legacy sketch retained for reference
    tests/                       Node test suite
```

## System Architecture

There are three main layers:

1. Dashboard
   - React/Vite frontend.
   - Workspaces: Teleop, Leg Tuning, Animations.
   - Talks to a backend over HTTP and WebSocket.

2. Desktop serial bridge
   - Express/WebSocket backend on port `8787`.
   - Lists serial ports, connects to the Feather, forwards commands, mirrors local dashboard state, and streams telemetry.
   - Provides animation upload/play/stop endpoints for the dashboard.

3. ESP32 firmware
   - Receives newline-delimited JSON commands over USB serial.
   - Supports direct servo-angle, direct foot XY, direct joint-angle, builtin, and animation playback modes.
   - Smoothly interpolates servo moves.
   - The active board/pin mapping lives with the firmware in [`../firmware/argos_servo/README.md`](../firmware/argos_servo/README.md).

The Feather is the real-time servo controller. Kinematics, gait generation, and animation-to-servo conversion run on the host; the firmware remains responsible for servo smoothing, safety release, telemetry, and low-level PWM writes.

## Hardware Summary

Core electronics:

- ESP32 board and wiring matched to [`../firmware/argos_servo`](../firmware/argos_servo).
- Adafruit-compatible PCA9685 16-channel PWM servo driver.
- 12 hobby servos: 3 per leg.
- Separate servo supply, assumed `7.4 V` in the firmware model.
- Common ground between Feather, PCA9685 logic, and servo power.
- USB serial link from the host computer to the Feather.

Servo/PCA9685 assumptions:

- Servo angle range: `0` to `180 deg`.
- Neutral servo angle: `90 deg`.
- Pulse width range: `1000 us` to `2000 us`.
- PCA9685 I2C address: `0x40`.
- PCA9685 refresh: `150 Hz`.
- Servo supply model: `7.4 V`.
- Default speed limit: `180 deg/sec` per axis.
- Firmware hardware speed estimate at 7.4 V: about `545 deg/sec`, reduced by a `0.80` safety factor to about `436 deg/sec`.
- Servo outputs are released by writing full-off to each PCA9685 channel.

Active firmware wiring and board notes are documented in [`../firmware/argos_servo/README.md`](../firmware/argos_servo/README.md). The legacy `robot_dog_leg_smooth_interpolated` sketch remains in-tree for comparison, but it is no longer the primary dashboard target.

## Servo Channel Map

The default PCA9685 channel allocation is:

| Leg | Hip yaw | Thigh | Calf |
| --- | ---: | ---: | ---: |
| Front left | 8 | 0 | 1 |
| Front right | 9 | 2 | 3 |
| Rear left | 10 | 4 | 5 |
| Rear right | 11 | 6 | 7 |

Right-side thigh and calf servo angles are mirrored around `90 deg` before full-body poses are sent. Hip yaw direction is handled by per-leg signs in firmware: left legs use `+1`, right legs use `-1`.

## Robot Model

Argos is modeled as a quadruped with:

- 4 legs: `front_left`, `front_right`, `rear_left`, `rear_right`.
- 3 joints per leg: `hipYaw`, `thigh`, `calf`.
- 12 total servo outputs.
- A body overview layout of `280 x 110` software units, with leg anchors:
  - Front left: `(-100, -40)`
  - Front right: `(100, -40)`
  - Rear left: `(-100, 40)`
  - Rear right: `(100, 40)`

The body layout is used by the dashboard visualization. The leg linkage dimensions below are the important values for recreating the mechanism.

## Leg Kinematics

Units are millimeters for linkage geometry and degrees/radians for angles.

Per-leg planar linkage constants:

| Parameter | Value |
| --- | ---: |
| Thigh length | `127 mm` |
| Calf length | `127 mm` |
| Servo horn length | `20 mm` |
| Short link length | `30 mm` |
| Bell crank radius | `40 mm` |
| Long link length | `150 mm` |
| Calf attach offset from knee | `30 mm` |
| Hip pivot | `(0, 0)` |
| Calf servo pivot | `(-20, -22)` |
| Foot origin offset | `(40, -140)` |

Coordinate convention:

- The leg solver operates in a local 2D plane.
- `hipPivot` is the local origin.
- Positive `x` is forward in the leg plane.
- Positive `y` is upward in the math model.
- Dashboard foot commands are relative to `footOriginOffset`.
- A foot command of `{ x: 0, y: 0 }` means absolute foot point `(40, -140)`.

Linkage sequence:

1. The thigh rotates about the hip pivot.
2. The knee is `127 mm` from the hip along `thetaThigh`.
3. The calf servo horn rotates about `servoPivot`.
4. The horn end connects through a `30 mm` short link to a bell crank point on a `40 mm` radius around the hip.
5. A second bell crank point, offset by 90 degrees, drives a `150 mm` long link.
6. The long link attaches to the calf `30 mm` from the knee.
7. The foot is `127 mm` from the knee along the solved calf angle.

The shared JS kinematics module is still the source of truth for the current dashboards. The browser and desktop bridge convert foot and joint targets into servo angles before serial transport so the debug and full-body dashboards stay aligned with their previews, even though the active firmware also supports direct foot and joint commands.

## Joint Limits

Default joint limits:

| Joint | Min | Max |
| --- | ---: | ---: |
| Hip yaw | `-35 deg` | `35 deg` |
| Thigh | `-145 deg` | `15 deg` |
| Calf | `-165 deg` | `-25 deg` |

The dashboard can update joint limits per leg. Limits are normalized if min/max are reversed, then used by the host-side IK and direct joint controls before servo angles are sent.

## Calibration Model

Servo model:

- A model servo angle of `90 deg` is neutral.
- Thigh neutral is fixed at `thetaThigh = 0`.
- Calf servo neutral is solved by the host-side kinematics model from the neutral foot position.
- Hip yaw maps directly as `servoDeg = 90 + hipYawDeg`.
- Thigh and calf model servo angles map to linkage angles through the neutral calibration.

Servo trims:

- Trims are stored per leg and per joint.
- Runtime trim changes are tracked by the dashboard and serial bridge.
- Record final trim offsets before ending a calibration session, and retune after rebuilding hardware.

Default startup joint target:

- Hip yaw: `0 deg`.
- Thigh: `-40 deg`.
- Calf: `-135 deg`.

Suggested calibration flow:

1. Power the Feather and dashboard without loading the legs.
2. Confirm the PCA9685 is detected at `0x40`.
3. Use Panic Release before attaching horns.
4. Attach each servo horn near mechanical neutral.
5. Confirm the channel map with one joint at a time.
6. Apply neutral trims in the dashboard until all legs match the neutral stance.
7. Tighten joint limits before testing gait.
8. Start with low speed limits and short direct movements.

## Locomotion

Procedural drive is generated in `shared/locomotion.js`.

Drive command axes:

- `vx`: forward/back, clamped `-1..1`.
- `vy`: strafe, clamped `-1..1`.
- `yawRate`: rotate, clamped `-1..1`.

Default gait behavior:

- Trot phase offsets:
  - `front_left`: `0`
  - `front_right`: `0.5`
  - `rear_left`: `0.5`
  - `rear_right`: `0`
- Cycle time: `1.8 - strideMagnitude * 0.35` seconds.
- Forward stride scale: `68 mm`.
- Strafe stride scale: `32 mm`.
- Foot lift: `56 mm`.
- Stance support dip: `10 mm`.
- Yaw hip bias: `24 deg`, signed by side.
- Strafe hip yaw contribution: `44 deg`, clamped by joint limits.

Modes:

- `idle`: no active motion.
- `stand`: neutral standing pose.
- `drive`: gait generator streams full-body poses.
- `calibration`: direct tuning mode.
- Firmware supports direct servo-angle mode and animation playback from uploaded servo-angle keyframes.

## Animation Clips

Animation clips are full-body foot keyframe tracks:

```json
{
  "version": 2,
  "name": "debug-step",
  "duration": 2,
  "tracks": {
    "front_left": [
      { "time": 0, "foot": { "x": 0, "y": 0 } },
      { "time": 0.5, "foot": { "x": 20, "y": 8 } }
    ],
    "front_right": [],
    "rear_left": [],
    "rear_right": []
  }
}
```

The animation helper validates clips, clamps keyframe times to the clip duration, sorts frames, and resolves foot keyframes into servo angles during upload. The firmware stores up to `32` keyframes per uploaded leg track and interpolates servo angles during playback.

Legacy single-leg clips can be imported and mapped to one selected leg or mirrored pairs from the dashboard.

## Serial Protocol

The firmware wire protocol is newline-delimited JSON over USB serial at `921600` baud. Commands include a `type` field and may include a `seq` number. Responses include `ack`, `error`, `state`, `hello_ack`, and animation progress messages.

Common commands:

```json
{ "type": "hello", "seq": 1 }
{ "type": "get_state", "seq": 2 }
{ "type": "release_servos", "seq": 3 }
{ "type": "set_mode", "mode": "direct_servo_angles", "seq": 4 }
```

Per-leg firmware servo command:

```json
{
  "type": "set_leg_servo_angles",
  "legId": "front_left",
  "hipServoDeg": 90,
  "thighServoDeg": 90,
  "calfServoDeg": 90
}
```

Dashboard-side full-body pose command:

```json
{
  "type": "apply_full_body_pose",
  "FLHipYawDeg": 90,
  "FLThighDeg": 90,
  "FLCalfDeg": 90,
  "FRHipYawDeg": 90,
  "FRThighDeg": 90,
  "FRCalfDeg": 90,
  "RLHipYawDeg": 90,
  "RLThighDeg": 90,
  "RLCalfDeg": 90,
  "RRHipYawDeg": 90,
  "RRThighDeg": 90,
  "RRCalfDeg": 90
}
```

Configuration commands:

- `set_leg_servo_channel_map`
- `set_leg_joint_limits`
- `set_leg_servo_speed_limit`
- `set_leg_servo_trim` through the serial bridge
- `upload_animation` with `begin`, servo-angle `frame`, `commit`
- `play_animation`, `pause_animation`, `stop_animation`

The serial bridge preserves the current dashboard command schema, including `apply_full_body_pose`, `set_stance`, `set_motion_mode`, and `set_drive_command`, then translates those requests into the active firmware wire format. The bridge still prefers servo-angle streaming for these dashboards so host-side previews, trims, gait helpers, and saved poses stay consistent.

## Backend API

The desktop serial bridge exposes the dashboard-facing API:

- `GET /api/status`
- `POST /api/connect`
- `POST /api/disconnect`
- `POST /api/command`
- `POST /api/animations`
- `POST /api/animations/:id/play`
- `POST /api/animations/:id/stop`
- `WS /telemetry`

In development the Vite app uses `http://localhost:8787` as the backend unless `VITE_BACKEND_URL` is set.

## Dashboard Workspaces

Teleop:

- Connect/disconnect the ESP32 Feather.
- Select a serial port.
- Forward/back/strafe/rotate controls.
- Keyboard controls: WASD, arrow keys, Q/E.
- Stance-height slider for lowering or raising the neutral body pose.
- Stand, Calibration, Idle, Stop, and Panic Release.
- Robot overview and live status cards.

Leg Tuning:

- Select a leg.
- View desired/current foot, joint, and servo telemetry.
- Command foot XY, joint angles, or raw servo angles.
- Update PCA9685 channel map.
- Update joint limits.
- Update servo speed limits.
- Apply/reset neutral trim.

Animations:

- Edit or import full-body animation clips.
- Preview keyframes in the dashboard.
- Upload clips to the backend/firmware.
- Play and stop clips on the robot.

## Running on a Desktop

From `robot_dog_debug_dashboard`:

```bash
npm install
npm run bridge
```

In a second terminal:

```bash
npm run dev
```

Open the Vite URL, usually:

```text
http://localhost:5173
```

On Windows, the root helper script starts the serial bridge and dashboard:

```powershell
.\launch_robot_dog_debug_dashboard.ps1
```

## Serial Bridge Configuration

| Variable | Default | Purpose |
| --- | --- | --- |
| `PORT` | `8787` | HTTP/WebSocket port |
| `VITE_BACKEND_URL` | `http://<dashboard-host>:8787` | Optional dashboard backend override |

## Firmware Build and Upload

Required Arduino libraries:

- ESP32 Arduino core.
- Adafruit PWM Servo Driver Library.
- Adafruit BusIO.

The canonical sketch now lives in [`../firmware/argos_servo`](../firmware/argos_servo). Follow that folder's README for board-specific wiring notes. Example compile/upload commands are shown below for the common ESP32-S3 target used with this refactor firmware.

Compile examples:

```bash
arduino-cli compile --fqbn <your-esp32-s3-fqbn> ../firmware/argos_servo
```

Upload example:

```bash
arduino-cli upload -p COM5 --fqbn <your-esp32-s3-fqbn> ../firmware/argos_servo
```

Use the correct serial port for your machine.

## Firmware Timing and Smoothing

Important firmware timing constants:

- Serial baud: `921600`.
- Telemetry interval: `200 ms`.
- Animation status interval: `250 ms`.
- Builtin status interval: `400 ms`.
- Duplicate servo targets within `0.05 deg` are ignored while moving in the serial bridge queue before they hit the wire.

The firmware estimates current servo position from commanded motion. There is no physical servo feedback. Telemetry `current` values are therefore estimates, not encoder measurements.

## Testing

From `robot_dog_debug_dashboard`:

```bash
npm test
npm run build
```

The test suite covers:

- Clip conversion and interpolation.
- Kinematics and joint limit behavior.
- Gait generation.
- Servo pose flattening/mirroring.
- Protocol validation.
- Serial JSON extraction.

## Recreating the Robot

Minimum build checklist:

1. Build a quadruped body with four identical leg modules.
2. Install 3 servos per leg for hip yaw, thigh, and calf linkage actuation.
3. Match the linkage dimensions in the kinematics table as closely as possible.
4. Mount each calf servo pivot at `(-20, -22)` relative to the hip pivot in the leg plane.
5. Use a `20 mm` servo horn, `30 mm` short link, `40 mm` bell crank arm, `150 mm` long link, and `30 mm` calf attach offset.
6. Wire all 12 servos to the PCA9685 channel map above.
7. Power servos from a supply sized for stall current, with common ground to the Feather.
8. Flash the Feather firmware.
9. Start the dashboard and serial bridge.
10. Calibrate trims, channel map, joint limits, and speed limits before walking.

Design assumptions to preserve:

- Servos accept `1000-2000 us` pulses.
- Servos are safe across the configured `0-180 deg` command range.
- Linkage geometry is symmetric between legs, with right-side servo outputs mirrored in software.
- The default physical stance starts from joint angles `{ hipYaw: 0, thigh: -40, calf: -135 }`.
- Servo power is stable under dynamic gait loads.

## Safety Notes

- Keep the robot lifted or unloaded during first firmware uploads and channel-map tests.
- Use Panic Release before installing or repositioning horns.
- Never power servos only from the Feather USB/3V rail.
- Tie grounds together before sending servo commands.
- Start with conservative joint limits.
- Retune trims after any horn, linkage, or servo replacement.
- The software has no force sensing, IMU stabilization, current sensing, or encoder feedback.

## Known Limitations

- Servo position telemetry is estimated from commands, not measured.
- The gait is open-loop and does not adapt to terrain.
- No IMU balancing or body attitude correction is implemented.
- No foot contact sensing is implemented.
- Linkage dimensions are hardcoded in the shared JS kinematics model; changing hardware geometry requires updating the host-side model.
- The dashboard visualization body dimensions are approximate software layout values, not a full CAD model.
