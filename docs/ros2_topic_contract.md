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
