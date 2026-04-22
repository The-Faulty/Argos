# Argos Component Test Guide

This is the quick-reference companion to [build_guide.md](build_guide.md).
Use it when the robot is wired and you want a fast go / no-go check on each
connected component.

## Before you start

- source the workspace
- keep the robot supported off the ground
- keep the servo rail off until the ESP32 bridge is already up
- use the ESP32 bridge for robot tests; do not mix in direct Pi-side PCA9685 tools

## Command matrix

| Component | What should already be running | Checker command | Pass signal |
|---|---|---|---|
| Host + device aliases | nothing | `ros2 run quadruped_bringup argos_component_check preflight --expect-lidar --expect-realsense --expect-thermal` | serial aliases, USB visibility, and optional I2C all pass |
| ESP32 bridge | `ros2 launch quadruped_bringup esp32_bridge.launch.py` | `ros2 run quadruped_bringup argos_component_check esp32` | `/joint_states`, `/imu/data_raw`, `/gas` all pass |
| IMU only | `esp32_bridge.launch.py` | `ros2 run quadruped_bringup argos_component_check imu` | IMU topic rate and finite values pass |
| Gas only | `esp32_bridge.launch.py` | `ros2 run quadruped_bringup argos_component_check gas` | gas topic appears and stays finite |
| Joint actuation | `esp32_bridge.launch.py`, servo rail on | `ros2 run quadruped_bringup argos_joint_jog --joint FR_coxa_joint` | expected joint moves and returns to zero |
| Full joint actuation | `esp32_bridge.launch.py`, servo rail on | `ros2 run quadruped_bringup argos_joint_jog --bidirectional` | each joint moves alone, no wrong joint moves |
| RPLidar | `ros2 launch quadruped_bringup rplidar.launch.py` | `ros2 run quadruped_bringup argos_component_check lidar` | `/scan` rate and finite ranges pass |
| RealSense | `ros2 launch quadruped_bringup realsense.launch.py` | `ros2 run quadruped_bringup argos_component_check realsense` | color + aligned depth both pass |
| Thermal camera | `ros2 launch quadruped_bringup mission_stack.launch.py` | `ros2 run quadruped_bringup argos_component_check thermal` | thermal image + camera info both pass |
| Whole robot | `esp32_bridge.launch.py` + `sensors.launch.py` | `ros2 run quadruped_bringup argos_component_check full` | MCU + lidar + RealSense all pass |
| Whole robot + thermal | previous row plus `mission_stack.launch.py` | `ros2 run quadruped_bringup argos_component_check full --include-thermal` | everything above plus thermal passes |

## Recommended order for a new robot

1. `argos_component_check preflight`
2. `argos_component_check esp32`
3. `argos_joint_jog --joint FR_coxa_joint`
4. `argos_joint_jog --bidirectional`
5. `argos_component_check lidar`
6. `argos_component_check realsense`
7. `argos_component_check thermal`
8. `argos_component_check full --include-thermal`

## When to use the older bench tools

These are still useful, but only with the ESP32 disconnected from the live
robot I2C / PWM path:

- `ros2_ws/argos_control/servo_test.py`
- `ros2_ws/argos_control/single_leg_test.py`
- `web/leg_viz/server.py`

Use them for horn alignment, single-leg linkage tuning, or off-robot bench
work. For the assembled robot, prefer `argos_joint_jog`.
