# Argos Expo Execution Plan

This plan is narrowed to the current expo goal:

- the robot stands and walks reliably
- LiDAR and RealSense both stream live data
- the robot builds a map of a small area in RViz

That is enough for a strong senior design expo. Thermal, gas mapping, mission
logic, and Gazebo are not required for the current demo.

## Demo definition

The expo story should be:

1. Power on the robot and bring up ROS 2 cleanly.
2. Show live LiDAR and RealSense feeds.
3. Show the quadruped walking under controlled commands.
4. Build a visible SLAM map while the robot moves.
5. Explain the Pi/ESP32-C6 split and the safety path.

## Current must-have scope

Required for the expo:

- stable stand
- stable crawl gait
- reliable `e-stop`
- live `/scan`
- live RealSense RGB/depth topics
- correct TF tree and robot model
- SLAM map generation while walking or being teleoperated

Nice to have, but not required:

- trot gait
- IMU fusion into EKF
- cleaner teleop UI
- rosbag recording
- polished RViz layout

Explicitly out of scope unless extra time appears:

- thermal camera integration
- victim detection
- gas hazard mapping
- mission orchestrator
- Gazebo simulation
- full Nav2 autonomy

## Task order

### 1. Freeze the real robot geometry and safety limits

Goal: stop hardware uncertainty from destabilizing the software stack.

- Confirm final `L1/L2/L3`, bell-crank dimensions, and servo directions
- Set per-joint safe limits and neutral positions
- Finalize tethered power protection and brownout behavior
- Validate one leg on the bench before running the full robot

Acceptance criteria:

- `Config.py` and the URDF match measured hardware
- each servo has a known neutral and safe min/max
- one physical leg moves without binding or overheating

### 2. Finish the ESP32-C6 walking/control path

Goal: make the real robot stand and crawl on command.

- Implement the 200 Hz Teensy loop
- subscribe to `/joint_command`
- publish `/joint_states` and `/imu/data_raw`
- add watchdog behavior and PWM disable on stale commands
- verify PCA9685 output and servo calibration on the full body

Acceptance criteria:

- robot stands consistently
- robot can crawl a short distance without losing posture
- `/joint_states` updates at the expected rate
- cutting commands triggers safe behavior

### 3. Lock down sensor bring-up

Goal: make LiDAR and RealSense boringly reliable.

- verify `/dev/ttyLIDAR` and `/dev/ttyTEENSY`
- confirm RealSense launch is stable on the Pi
- confirm LiDAR frame and camera frame match the URDF
- verify RViz can display scan, RGB, depth, and TF together

Acceptance criteria:

- sensors launch without manual replug/retry loops
- RViz shows consistent LiDAR and camera data
- TF frames line up with the physical robot

### 4. Add mapping before autonomy

Goal: map an area with the robot under manual control.

- integrate `slam_toolbox`
- feed LiDAR into SLAM first
- use teleop or simple velocity commands for motion during mapping
- add IMU/EKF only if mapping quality clearly needs it

Acceptance criteria:

- robot can build a recognizable map of a hallway or room
- map remains stable enough to explain live at the expo
- mapping works without depending on Gazebo or thermal features

### 5. Rehearse the exact demo

Goal: remove surprises before expo day.

- create a fixed startup order
- create a fixed RViz layout
- define a 2-3 minute operator script
- prepare a fallback demo path if walking fails

Acceptance criteria:

- the team can run the demo repeatedly with the same steps
- each person knows what to say during the live explanation
- there is a backup plan such as sensor + mapping on a stand or cart

## Suggested team split

- Chase: Pi bring-up, ROS 2 launch flow, SLAM integration, system glue
- Andrew: linkage validation, servo mounting, mechanical reliability, tether safety
- Alana + Alesander: ESP32-C6 loop, PCA9685 driving, IMU, watchdog behavior
- Jared: RViz/demo flow, RealSense + LiDAR validation, map demo setup

## Simple weekly priority

If you need one short checklist, use this:

1. Make one leg safe and calibrated.
2. Make the whole robot stand.
3. Make the whole robot crawl.
4. Make LiDAR and RealSense launch every time.
5. Make SLAM build a map while moving.
6. Polish the demo script and fallback plan.

## De-scope guidance

If time gets tight, cut in this order:

1. trot gait
2. EKF polish
3. Gazebo
4. thermal and mission features
5. Nav2 autonomy

Do not cut these:

1. safe stand
2. controlled walking
3. live sensor visualization
4. mapping demo
5. clear explanation of architecture and safety
