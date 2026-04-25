# Argos Dashboard Guide

The Argos dashboard is a browser-based control + telemetry panel for the
quadruped. It is the replacement for the retired single-leg `web/leg_viz/`
Python tool and covers the full 12-joint robot: foot targets, per-joint
commands, gait tuning, IMU/gas telemetry, servo calibration overrides,
stance presets, full-body animation playback, and recording.

## Overview

The dashboard is a React single-page app served by a small Node/Express
server on the Raspberry Pi. The Node server proxies browser traffic to
`rosbridge_websocket`, which speaks to the ROS 2 control stack. ROS 2 then
talks to the ESP32-C6 over the micro-ROS/UART link. The ESP32 owns the
servos, the IMU, and the gas sensor. Argos is tethered, so there is no
battery monitor.

```
 ┌────────────┐  HTTPS / WS       ┌────────────────────────────┐
 │   Browser  │ ◀─────────────────▶│  Pi Node server (pi_server)│
 │ (React SPA)│    /telemetry      │  Express + WS + roslibjs   │
 └────────────┘                    └────────────┬───────────────┘
                                                │  ws://localhost:9090
                                                ▼
                                   ┌────────────────────────────┐
                                   │  rosbridge_websocket (ROS2)│
                                   └────────────┬───────────────┘
                                                │  ROS 2 DDS
                                                ▼
                         ┌──────────────────────────────────────┐
                         │  argos_control + quadruped_bringup   │
                         │  (gait, safety, dashboard bridge)    │
                         └─────────────────┬────────────────────┘
                                           │ /joint_command (UART)
                                           ▼
                                   ┌────────────────────────────┐
                                   │  ESP32-C6 micro-ROS client │
                                   │  servos, IMU, gas          │
                                   └────────────────────────────┘
```

Leg ordering is `FR / FL / RR / RL` everywhere (never `front_left`, etc.),
and there are three joints per leg (`coxa`, `femur`, `tibia`) for 12 total.
The canonical joint name list lives in
`ros2_ws/argos_control/ros_support.py::JOINT_NAMES`; the dashboard mirrors
it in `dashboard/shared/robot-config.js` and a build-time test
(`dashboard/tests/robot_config.test.js`) diffs the two.

## Prerequisites

- Raspberry Pi 4 (or equivalent) with Raspberry Pi OS / Ubuntu 22.04
- ROS 2 Humble installed and sourced (`/opt/ros/humble/setup.bash`)
- `ros-humble-rosbridge-suite` apt package installed
- Node 20 or newer (the SPA uses Vite 6 and Node's built-in test runner)
- ESP32-C6 flashed with the latest firmware from `firmware/esp32c6/`
  (the firmware exposes `/release_servos` and the per-joint servo
  calibration table)
- The device udev rules installed so `/dev/ttyESP32` is stable — see
  `docs/pi_hardware_contract.md`

## First-boot setup

```bash
# 1. Build the dashboard front-end + install Node deps
cd dashboard
npm install
npm run build

# 2. Build the ROS 2 workspace (one-time, or after a code change)
cd ../ros2_ws
colcon build --packages-select argos_control quadruped_bringup
source install/setup.bash
```

Persistent state (servo override table, stance library) is written to
`~/.argos/` on first run — the `dashboard_bridge_node` creates the
directory if missing.

## Launching the full stack

```bash
source /opt/ros/humble/setup.bash
source ~/argos/ros2_ws/install/setup.bash

ros2 launch quadruped_bringup full_system.launch.py \
    enable_esp32:=true \
    enable_dashboard:=true
```

What that does, in order:

1. Starts the URDF / state publisher.
2. Starts the motion pipeline (gait planner, safety node, command mux).
3. Starts the micro-ROS agent on `/dev/ttyESP32`.
4. Starts `rosbridge_websocket` on `ws://localhost:9090` (not exposed to
   the LAN on purpose — the Node server proxies it).
5. Starts `dashboard_bridge_node` which translates dashboard intents into
   `/joint_command/raw` (so foothold_checker + safety_node keep enforcing
   limits).
6. Starts `node pi_server/server.js` on `0.0.0.0:8787`, which serves the
   built SPA and exposes the `/telemetry` websocket.

Open the dashboard at `http://<pi-ip>:8787` in any modern browser on the
same LAN.

## Feature walkthrough

- **Mode selector** (`idle / crouch / stand / extend / crawl / trot /
  direct_foot_xyz / direct_joint_angles / direct_servo_angles /
  animation_playback`). Switching to a `direct_*` mode publishes
  `/control_mode=manual` so the gait planner stops fighting for the bus;
  switching back to a gait mode publishes `/control_mode=auto`.
- **Twist joystick** — maps to `/teleop/cmd_vel` when a gait mode is
  active. The pi_server rate-limits to 20 Hz.
- **Settings panel** — per-joint servo inversion and offset overrides,
  persisted to `~/.argos/servo_overrides.json` via the
  `/dashboard/servo_overrides` topic. The bridge applies the formula
  `invert ? -a + offset : a + offset` (radians) before publishing
  `/joint_command/raw`. Per-leg joint min/max angle inputs (coxa /
  femur / tibia, in degrees) go out on `/dashboard/joint_limits` and
  are persisted to `~/.argos/joint_limits.json`; the bridge intersects
  these with the firmware defaults and applies them inside
  `_clamp_envelope` before `Kinematics.clamp_joint_matrix`. The same
  panel also holds the per-joint speed ceilings
  (`~/.argos/servo_speed_limits.json`, deg/s) and the PCA9685 PWM
  update rate (`~/.argos/servo_update_rate.json`, Hz). Speed limits go
  out on `/dashboard/servo_speed_limits`; the bridge enforces them by
  rate-limiting per-joint deltas on `/joint_command/raw`. The PWM rate
  goes out on `/dashboard/servo_update_rate_hz`; the ESP32
  reconfigures the PCA9685 on the next tick.
- **Stance presets** — named poses saved to `~/.argos/stances.json`.
  Clicking a preset publishes its name on `/dashboard/stance_play`; the
  bridge looks the pose up and pushes its 12-angle vector onto the
  safety pipeline.
- **Stride-length tuning** — sliders for `delta_x`, `delta_y`,
  `swing_time`, and `rotate_rate_max` matching the `GAIT_TUNABLE_PARAMS`
  block in `dashboard/shared/robot-config.js`.
- **Rotate buttons** — shortcut for `angular.z` twists; sends a zero
  Twist on release.
- **Disconnect = e-stop** — closing the browser tab or clicking
  Disconnect publishes `/release_servos=True` which makes the ESP32
  release its PCA9685 outputs and latches `/estop`. The safety node will
  refuse to re-engage until a gait mode is selected again.
- **IMU / gas panels** — live telemetry subscribed off `/imu/data_raw`
  and `/gas` through rosbridge. Panels grey out after 1 s without a
  message.
- **Animation playback** — upload a JSON clip conforming to
  `DEFAULT_FULL_BODY_CLIP` (see `dashboard/shared/animation.js`) and
  scrub / play it. The bridge runs IK on each frame before publishing.
- **Recording** — start/stop a telemetry recording from the UI. Files
  land under `~/.argos/recordings/`; the Node server owns the rosbag
  writes.
- **Low-latency WS command channel** — the telemetry WebSocket also
  accepts `{type:"command", command:{...}}` frames. The dashboard
  streams servo-slider drags and foot-target drags over WS (8 ms
  throttle) and falls back to `POST /api/command` when the socket is
  closed. Set `VITE_BACKEND_URL` in `.env.pi` for `npm run dev` when
  the pi_server is on another host.

## Troubleshooting

- **`ws://<pi>:9090` won't connect** — rosbridge is bound to localhost
  only on purpose. The browser should talk to the Node server on port
  8787, which proxies. Check that `ros2 launch quadruped_bringup
  rosbridge.launch.py` is running.
- **ESP32 telemetry missing (IMU/gas greyed out)** — verify
  `/dev/ttyESP32` exists (`ls -l /dev/ttyESP32`) and that the micro-ROS
  agent is running (`enable_esp32:=true` in the launch). Check
  `ros2 topic echo /imu/data_raw` on the Pi.
- **`servo_overrides.json` corrupted** — stop the dashboard, delete
  `~/.argos/servo_overrides.json`, and restart. The bridge rewrites an
  empty table on boot.
- **Dashboard shows stale telemetry** — the Node server only forwards
  ROS messages it has an active subscription for. Reloading the tab
  re-subscribes. If messages are still missing, check that
  `rosbridge_websocket` is reporting the subscription (`ros2 topic info
  /imu/data_raw`).
- **Disconnect does not release the servos** — confirm the ESP32
  firmware is current (it must subscribe to `/release_servos`). A quick
  check: `ros2 topic echo /release_servos` while toggling the panel.
- **IK miss on `/dashboard/foot_targets`** — the bridge logs
  `IK miss on foot target: ...` when a leg is out of workspace. Verify
  foot targets stay inside the default stance envelope by comparing
  against `DEFAULT_STANCE` in `dashboard/shared/robot-config.js`.
