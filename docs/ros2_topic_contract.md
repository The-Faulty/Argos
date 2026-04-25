# Argos ROS 2 Topic Contract

This is the current software contract between the Raspberry Pi ROS 2 stack,
simulation, and the ESP32-C6 micro-ROS client.

Current expo focus: walking, live LiDAR + RealSense data, and SLAM mapping.
Thermal and mission topics listed below are optional future work and are not
required for the current expo demo.

## Core command and state topics

- `/teleop/cmd_vel` (`geometry_msgs/Twist`): operator command input
- `/nav/cmd_vel` (`geometry_msgs/Twist`): Nav2 command input
- `/cmd_vel` (`geometry_msgs/Twist`): muxed motion command
- `/command_source` (`std_msgs/String`): `teleop`, `nav`, `idle`, or `estop`
- `/gait_mode` (`std_msgs/String`): `crouch`, `stand`, `crawl`, or `trot`
- Startup behavior: the control stack boots in `crouch`, which publishes zero on all 12 joint targets until another gait mode is commanded
- `/estop` (`std_msgs/Bool`): global e-stop latch
- `/joint_command/raw` (`sensor_msgs/JointState`): gait planner output before safety
- `/joint_command/safe` (`sensor_msgs/JointState`): safety-filtered joint targets
- `/joint_command` (`sensor_msgs/JointState`): fixed-rate command stream to the ESP32-C6
- `/joint_states` (`sensor_msgs/JointState`): current joint state feedback or preview

## Sensor topics

- `/imu/data_raw` (`sensor_msgs/Imu`): raw IMU from the ESP32-C6
- `/gas` (`std_msgs/Float32`): filtered gas sensor output
- `/scan` (`sensor_msgs/LaserScan`): RPLiDAR A1M8 output
- `/camera/*`: RealSense streams
- `/thermal/image_raw` (`sensor_msgs/Image`, `32FC1`): MLX90640-style thermal image
- `/thermal/camera_info` (`sensor_msgs/CameraInfo`): thermal camera intrinsics

## Dashboard bridge topics

Published and consumed by `argos_control.dashboard_bridge_node` and the Pi
Node server (`dashboard/pi_server/server.js`). All names come from
`ros2_ws/argos_control/ros_support.py::TOPICS` — change the string in one
place only.

| Topic | Type | Publisher | Subscriber | Notes |
| --- | --- | --- | --- | --- |
| `/control_mode` | `std_msgs/String` | dashboard (Node server) | `dashboard_bridge_node`, `gait_planner_node` | `auto` or `manual`. Manual mode suppresses gait_planner raw writes so both sources never fight the bus. |
| `/dashboard/foot_targets` | `geometry_msgs/PoseArray` | dashboard | `dashboard_bridge_node` | 4 poses, body-frame, order FR/FL/RR/RL. Bridge runs IK and forwards to `/joint_command/raw`. |
| `/dashboard/joint_angles` | `sensor_msgs/JointState` | dashboard | `dashboard_bridge_node` | 12 positions in `JOINT_NAMES` order (or name-indexed). Used by direct-joint mode. |
| `/dashboard/servo_overrides` | `std_msgs/String` (JSON) | dashboard | `dashboard_bridge_node` | Per-joint `{invert, offset_rad}`. Persisted atomically to `~/.argos/servo_overrides.json`. |
| `/dashboard/joint_limits` | `std_msgs/String` (JSON) | dashboard | `dashboard_bridge_node` | Per-joint `{min_rad, max_rad}` (or `min_deg/max_deg`). Bridge intersects with firmware defaults, applies before `Kinematics.clamp_joint_matrix`. Persisted to `~/.argos/joint_limits.json`. Only safety gate on `/dashboard/joint_angles` — `safety_node` sits on the pre-mux channel and never sees dashboard output. |
| `/dashboard/stance_play` | `std_msgs/String` | dashboard | `dashboard_bridge_node` | Named stance lookup from `~/.argos/stances.json`. |
| `/dashboard/servo_speed_limits` | `std_msgs/String` (JSON) | dashboard | `dashboard_bridge_node` | Per-joint `{joint_name: deg_per_sec}`. Bridge rate-limits `/joint_command/raw` deltas so slider jumps ramp instead of snapping. Persisted to `~/.argos/servo_speed_limits.json`. |
| `/dashboard/servo_update_rate_hz` | `std_msgs/Float32` | dashboard | ESP32-C6 firmware | PCA9685 PWM refresh frequency in Hz. Firmware clamps to `[40, 200]` before calling `pca9685_set_pwm_freq`. Persisted on the Pi at `~/.argos/servo_update_rate.json`. |
| `/release_servos` | `std_msgs/Bool` | dashboard | ESP32-C6 firmware | `True` releases the PCA9685 outputs — used on Disconnect for e-stop. |

## Mission topics

- `/victim_detections` (`visualization_msgs/MarkerArray`): thermal hotspot markers
- `/hazard_map` (`nav_msgs/OccupancyGrid`): gas-based 2D hazard grid
- `/mission/state` (`std_msgs/String`): `IDLE`, `EXPLORE`, `DETECT`, `REPORT`, `CONTINUE`, or `ESTOP`
- `/mission/detection_count` (`std_msgs/Int32`): current victim candidate count

## Joint naming contract

The robot description, simulation, and hardware all use the same joint names:

- `FR_coxa_joint`
- `FR_femur_joint`
- `FR_tibia_joint`
- `FL_coxa_joint`
- `FL_femur_joint`
- `FL_tibia_joint`
- `RR_coxa_joint`
- `RR_femur_joint`
- `RR_tibia_joint`
- `RL_coxa_joint`
- `RL_femur_joint`
- `RL_tibia_joint`

The ROS 2 control nodes publish these in that exact order.

## Twist command mapping

The current gait planner interprets `/cmd_vel` like this:

- `linear.x`: forward velocity command
- `linear.y`: lateral velocity command
- `linear.z`: body height offset from the nominal stand height
- `angular.x`: body roll command
- `angular.y`: body pitch command
- `angular.z`: yaw rate command

## Expected rates

- `/cmd_vel`: 20-50 Hz
- `/joint_command/raw`: 50 Hz
- `/joint_command/safe`: 100 Hz
- `/joint_command`: 100 Hz
- `/joint_states`: 100 Hz
- `/imu/data_raw`: 100 Hz
- `/gas`: 10 Hz
- `/thermal/image_raw`: 4 Hz
- `/victim_detections`: 4 Hz
- `/hazard_map`: 1 Hz
