# Argos

Argos is a low-cost search and rescue quadruped being built for Georgia Tech
Senior Design in Spring 2026. The system is split between a Raspberry Pi 4
running ROS 2 Humble and a Teensy 4.0 handling the fast control loop.

## Current workspace

- `ros2_ws/src/argos_control`: ROS 2 package wrapper and node entry points
- `ros2_ws/src/quadruped_bringup`: launch files and sensor bring-up assets
- `ros2_ws/src/quadruped_description`: robot description, frames, and RViz assets
- `ros2_ws/argos_control`: leg math, control helpers, and single-leg bench tools
- `firmware`: embedded code and MCU-facing work
- `simulation`: Gazebo and robot model work

## Development environment

The repo includes a Dockerfile, Docker Compose service, and VS Code devcontainer
setup for a ROS 2 Humble workflow.

Typical flow:

```bash
docker compose build
docker compose up -d
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Sensor bring-up

LiDAR only:

```bash
ros2 launch quadruped_bringup rplidar.launch.py
ros2 launch quadruped_bringup view_lidar.launch.py
```

RealSense only:

```bash
ros2 launch quadruped_bringup realsense.launch.py
ros2 launch quadruped_bringup view_camera.launch.py
```

Both sensors:

```bash
ros2 launch quadruped_bringup sensors.launch.py
```

Robot description only:

```bash
ros2 launch quadruped_bringup state_publisher.launch.py start_rviz:=true
```

Control stack only:

```bash
ros2 launch quadruped_bringup control_stack.launch.py
```

Full stack:

```bash
ros2 launch quadruped_bringup full_system.launch.py enable_teensy:=true start_rviz:=true
```

## Notes

- `ROS_DOMAIN_ID` is set to `42` to match the Pi/Teensy design.
- On the Pi, the intended serial aliases are `/dev/ttyLIDAR` and `/dev/ttyTEENSY`.
- The single-leg tester can be run without ROS from `argos_control`.
- The control library is now wrapped as a ROS 2 package so `colcon` can see it.
- The ROS 2 topic contract is documented in `docs/ros2_topic_contract.md`.
- The Pi device and bring-up contract is documented in `docs/pi_hardware_contract.md`.
