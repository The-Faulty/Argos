# Argos

Argos is a low-cost search and rescue quadruped being built for Georgia Tech
Senior Design in Spring 2026. The system is split between a Raspberry Pi 4
running ROS 2 Humble and an ESP32-C6 handling the fast control loop.

## Current expo focus

The current design expo target is intentionally narrow:

- stable stand and walk
- live RPLiDAR + RealSense bring-up
- real-time mapping in RViz

Gazebo, thermal, gas mapping, and higher-level mission logic are optional
future work, not the current critical path.

## Current workspace

- `ros2_ws/src/argos_control`: ROS 2 package wrapper and node entry points
- `ros2_ws/src/argos_mission`: ROS 2 package wrapper for mission/perception bench nodes
- `ros2_ws/src/quadruped_bringup`: launch files and sensor bring-up assets
- `ros2_ws/src/Argos_description`: robot description, frames, meshes, and RViz assets
- `ros2_ws/argos_control`: leg math, control helpers, and single-leg bench tools
- `ros2_ws/argos_mission`: mission state, gas hazard map, and thermal demo nodes
- `firmware/esp32c6`: ESP-IDF + micro-ROS firmware for the C6 (servos, IMU, gas)
- `web/leg_viz`: bench-only single-leg web visualizer + Pi PCA9685 bridge

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
ros2 launch quadruped_bringup full_system.launch.py enable_esp32:=true start_rviz:=true
```

Expo demo stack:

```bash
ros2 launch quadruped_bringup expo_demo.launch.py enable_esp32:=true start_demo_commander:=true
```

Mission stack only:

```bash
ros2 launch quadruped_bringup mission_stack.launch.py
```

Full stack with mission nodes:

```bash
ros2 launch quadruped_bringup full_system.launch.py enable_esp32:=true enable_mission:=true start_rviz:=true
```

## Notes

- `ROS_DOMAIN_ID` is set to `42` to match the Pi/ESP32-C6 design.
- On the Pi, the intended serial aliases are `/dev/ttyLIDAR` and `/dev/ttyESP32`.
- The single-leg tester can be run without ROS from `argos_control`.
- The control library is now wrapped as a ROS 2 package so `colcon` can see it.
- The mission bench stack is optional and not part of the current expo-critical path.
- The ROS 2 topic contract is documented in `docs/ros2_topic_contract.md`.
- The Pi device and bring-up contract is documented in `docs/pi_hardware_contract.md`.
- The recommended expo-critical execution order is documented in `docs/expo_execution_plan.md`.
- The single source-of-truth expo checklist is documented in `docs/expo_master_todo.md`.
- The repeatable demo launch flow is documented in `docs/expo_demo_runbook.md`.
- The architecture, safety, terrain, and scope talking points are documented in `docs/expo_story.md`.
- The end-to-end build instructions (Pi + ESP32 + sensors) are documented in `docs/build_guide.md`.
