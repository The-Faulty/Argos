# Argos Pi Minimal Setup

This is the current bring-up path for the robot state you described:

- Raspberry Pi 4
- ESP32 Feather HUZZAH32
- no IMU yet
- no lidar yet
- no RealSense yet

Use this document instead of the full build guide until those sensors are
actually installed.

## 1. Install the Pi-side ROS tools

Assumes Raspberry Pi OS and ROS 2 Humble are already installed.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-micro-ros-agent \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro
```

If `rosdep` has not been initialized on this Pi yet:

```bash
sudo rosdep init
rosdep update
```

## 2. Build the workspace

```bash
cd ~/argos/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Set the ROS domain on the Pi shell you actually use:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/argos/ros2_ws/install/setup.bash
```

## 3. Find the HUZZAH32 serial device

Until you create a custom udev alias, use the stable `/dev/serial/by-id/...`
path directly.

```bash
ls -l /dev/serial/by-id
ls -l /dev/ttyUSB* /dev/ttyACM*
```

If you want `/dev/ttyESP32`, customize
`ros2_ws/src/argos_bringup/config/99-argos-devices.rules` with the board's
real USB serial number first.

## 4. Flash note for the MCU

The firmware directory is still named `firmware/esp32c6`, but the checked-in
`sdkconfig` currently targets plain `esp32`, which is what the Feather HUZZAH32
needs.

The Pi-side bridge now defaults to `115200` baud because the current custom
UART transport in `firmware/esp32c6/main/esp32_serial_transport.c` is hardcoded
to that rate.

If you need to rebuild or reflash:

```bash
cd ~/argos/firmware/esp32c6
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

## 5. Preflight on the Pi

Replace the serial path with the one you found in `/dev/serial/by-id`.

```bash
ros2 run quadruped_bringup argos_component_check \
  preflight \
  --esp32-device /dev/serial/by-id/<your-board>
```

You do not need `--expect-lidar` or `--expect-realsense` right now.

## 6. Start the minimal robot stack

```bash
ros2 launch quadruped_bringup pi_minimal.launch.py \
  serial_device:=/dev/serial/by-id/<your-board>
```

What this launch does:

- starts the robot description / TF tree
- starts the control stack
- disables the foothold checker
- disables IMU stabilization and push recovery
- does not launch lidar, RealSense, mission, or dashboard nodes
- optionally starts the ESP32 micro-ROS bridge

## 7. Validate the MCU loop

In a second terminal with the workspace sourced:

```bash
ros2 run quadruped_bringup argos_component_check core
```

If the gas sensor is installed but the IMU is not:

```bash
ros2 run quadruped_bringup argos_component_check esp32 --skip-imu
```

If neither gas nor IMU is installed:

```bash
ros2 run quadruped_bringup argos_component_check esp32 --skip-imu --skip-gas
```

## 8. Jog one joint at a time

Keep the servo power rail off until the bridge is already up. Support the
robot so nothing can slam into the floor.

```bash
ros2 run quadruped_bringup argos_joint_jog --joint FR_coxa_joint
ros2 run quadruped_bringup argos_joint_jog --bidirectional
```

## 9. What to avoid right now

- Do not launch `sensors.launch.py` yet.
- Do not expect `/imu/data_raw` yet.
- Do not use `full_system.launch.py` with `enable_sensors:=true` yet.
- Do not rely on `/dev/ttyESP32` until you have a board-specific udev rule.
