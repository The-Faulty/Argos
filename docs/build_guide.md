# Argos Physical Robot Build Guide

This guide covers the repo-supported path from bare electronics and a finished
mechanical frame to a powered robot that can publish sensor data, jog joints,
and run the full ROS 2 bring-up stack.

What this guide does cover:

- power, wiring, and bring-up order
- Raspberry Pi setup
- ROS 2 workspace setup
- ESP32-C6 firmware build and flash
- per-component test scripts
- first full-system validation

What this guide does not cover in exact step-by-step detail:

- CAD assembly drawings
- fastener-by-fastener mechanical assembly
- custom PCB rework

Those artifacts are not currently stored in this repo, so the mechanical
sections below stay at the checklist / wiring-contract level instead of
inventing geometry-specific instructions.

Cross references:

- Component tests: [component_test_guide.md](component_test_guide.md)
- Pi hardware contract: [pi_hardware_contract.md](pi_hardware_contract.md)
- Topic contract: [ros2_topic_contract.md](ros2_topic_contract.md)
- Demo runbook: [expo_demo_runbook.md](expo_demo_runbook.md)

---

## 1. Success criteria

You are done when all of the following are true:

- `/dev/ttyESP32` and `/dev/ttyLIDAR` exist on the Pi
- the ESP32 bridge publishes `/joint_states`, `/imu/data_raw`, and `/gas`
- the lidar publishes `/scan`
- the RealSense publishes color and aligned depth
- the thermal camera publishes `/thermal/image_raw` if that sensor is installed
- `argos_joint_jog` can move each joint one at a time
- `argos_component_check full` passes
- `expo_demo.launch.py` runs without manual patching

---

## 2. Hardware checklist

### Compute

- Raspberry Pi 4 with reliable USB-C power
- microSD card, 32 GB or larger
- ESP32-C6 dev board with native USB
- powered USB hub if the RealSense needs one

### Actuation

- 12 hobby servos
- PCA9685 servo driver
- separate 5-6 V servo power supply, at least 10 A continuous

### Sensors

- RPLiDAR A1M8
- Intel RealSense D435 or D455
- LSM9DS0 IMU
- MQ-series gas sensor breakout
- MLX90640 thermal camera if mission sensing is installed

### Cables and lab gear

- one real USB data cable for the ESP32
- lidar USB-UART cable
- RealSense USB 3 cable
- I2C wiring and pull-ups as required by your breakouts
- bench supply, multimeter, and a way to support the robot off the ground

---

## 3. Label everything before assembly

Do this before you zip-tie wires or mount the top shell.

- Label legs `FR`, `FL`, `RR`, `RL`
- Label each servo by joint name before plugging it into the PCA9685 harness
- Keep the PCA9685 channel map aligned with the firmware table in `firmware/esp32c6/main/main.c`

Servo channel map:

| Channel | Joint |
|---|---|
| 0 | `FR_coxa_joint` |
| 1 | `FR_femur_joint` |
| 2 | `FR_tibia_joint` |
| 3 | `FL_coxa_joint` |
| 4 | `FL_femur_joint` |
| 5 | `FL_tibia_joint` |
| 6 | `RR_coxa_joint` |
| 7 | `RR_femur_joint` |
| 8 | `RR_tibia_joint` |
| 9 | `RL_coxa_joint` |
| 10 | `RL_femur_joint` |
| 11 | `RL_tibia_joint` |

If your physical harness does not match this table, fix the harness or the
firmware before trying to walk the robot.

---

## 4. Wiring and power

### Power rails

Keep the logic rail and servo rail separate.

| Rail | Source | Loads |
|---|---|---|
| 5 V logic | Pi USB-C PSU | Pi, ESP32-C6 over USB, sensor breakouts |
| 5-6 V servo rail | bench supply | PCA9685 V+ rail and servos only |
| GND | shared | every subsystem must share ground |

Rules:

- never power the servos from the Pi 5 V rail
- tie grounds together before sending PWM or ADC signals
- power the servo rail last and turn it off first

### I2C bus

The ESP32-C6 owns the actuator/sensor I2C bus:

- `GPIO23` SDA
- `GPIO22` SCL
- PCA9685 at `0x00` on the current Argos board
- LSM9DS0 accel/mag at `0x1D`
- LSM9DS0 gyro at `0x6B`
- gas sensor is analog on `GPIO4`

The thermal camera is currently expected on the Pi's I2C bus at `0x33`.

### USB devices on the Pi

- ESP32-C6 -> `/dev/ttyESP32`
- lidar USB-UART -> `/dev/ttyLIDAR`
- RealSense -> USB 3 camera device

Install the udev rules from `ros2_ws/src/quadruped_bringup/config/99-argos-devices.rules`
before relying on the serial aliases.

---

## 5. Raspberry Pi OS and system setup

### 5.1 Flash and update

1. Flash Raspberry Pi OS Bookworm 64-bit.
2. Enable SSH during imaging.
3. Boot the Pi and run:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

### 5.2 Enable interfaces

```bash
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo apt install -y i2c-tools usbutils
```

### 5.3 Install Docker or native ROS 2

Pick one supported workflow.

#### Option A: Docker / devcontainer-style workflow

```bash
sudo apt install -y docker.io docker-compose-plugin git
sudo usermod -aG docker $USER
# log out and back in
git clone <argos-repo> ~/argos
cd ~/argos
docker compose build
docker compose up -d
```

#### Option B: Native ROS 2 Humble

Install ROS 2 Humble and the packages already assumed by the launch files:

- `ros-humble-rviz2`
- `ros-humble-xacro`
- `ros-humble-robot-state-publisher`
- `ros-humble-rplidar-ros`
- `ros-humble-realsense2-camera`
- `ros-humble-micro-ros-agent`
- `ros-humble-slam-toolbox`

Also install:

```bash
sudo apt install -y python3-pip python3-smbus
python3 -m pip install adafruit-blinka adafruit-circuitpython-mlx90640
```

### 5.4 Install udev rules

```bash
cd ~/argos
sudo cp ros2_ws/src/quadruped_bringup/config/99-argos-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyESP32 /dev/ttyLIDAR
```

### 5.5 Build the workspace

#### Native

```bash
cd ~/argos/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### Docker

```bash
docker compose exec argos bash
cd /workspace/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 5.6 ROS environment

Add this to the shell you actually use on the Pi:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/argos/ros2_ws/install/setup.bash
```

---

## 6. ESP32-C6 firmware

### 6.1 Install ESP-IDF

Use ESP-IDF v5.x.

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
. ./export.sh
```

### 6.2 Clone the micro-ROS ESP-IDF component as a sibling repo

```text
~/dev/
├── argos/
└── micro_ros_espidf_component/
```

```bash
cd ~/dev
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git
```

### 6.3 Configure the firmware

```bash
cd ~/dev/argos/firmware/esp32c6
idf.py set-target esp32c6
idf.py menuconfig
```

Verify these settings:

- PCA9685 I2C address matches your hardware
- gas ADC GPIO matches your wiring
- joint-command watchdog remains at `250 ms`
- IMU/control loop rate remains at `100 Hz`
- gas publish rate remains at `10 Hz`
- micro-ROS transport matches the Pi-side bridge expectations

### 6.4 Build and flash

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

You want to see the IMU and PCA9685 initialize without repeated I2C errors.

---

## 7. First preflight on the Pi

Run the passive checks before powering the servos.

From source:

```bash
python3 ros2_ws/src/quadruped_bringup/scripts/argos_component_check \
  preflight \
  --expect-lidar \
  --expect-realsense \
  --expect-thermal
```

After the workspace is built:

```bash
ros2 run quadruped_bringup argos_component_check \
  preflight \
  --expect-lidar \
  --expect-realsense \
  --expect-thermal
```

Expected pass conditions:

- `ros2`, `colcon`, and `micro_ros_agent` are available
- `/dev/ttyESP32` exists
- `/dev/ttyLIDAR` exists
- the RealSense is visible on USB
- `/dev/i2c-1` exists and `0x33` responds if thermal is installed

Do not move on until preflight is green.

---

## 8. Bring-up order on real hardware

Use this order every time:

1. servo PSU off
2. Pi power on
3. SSH in and source the workspace
4. confirm preflight
5. start the ESP32 bridge
6. only then turn on the servo PSU
7. run the joint jog / sensor tests

Shutdown order:

1. stop launches
2. turn off servo PSU
3. disconnect or power down the Pi

---

## 9. Per-component tests

This is the recommended sequence for a newly assembled robot.

### 9.1 URDF and TF only

```bash
ros2 launch quadruped_bringup state_publisher.launch.py start_rviz:=true
```

Use this to confirm the model loads and the frame tree is sane before touching
real hardware.

### 9.2 ESP32 bridge, IMU, gas, and feedback

Start the bridge:

```bash
ros2 launch quadruped_bringup esp32_bridge.launch.py
```

In another terminal:

```bash
ros2 run quadruped_bringup argos_component_check esp32
ros2 run quadruped_bringup argos_component_check imu
ros2 run quadruped_bringup argos_component_check gas
```

Pass criteria:

- `/joint_states` arrives with 12 positions
- `/imu/data_raw` is live
- `/gas` is live and finite

### 9.3 Joint-by-joint actuation test

Support the robot so the legs cannot slam into the floor.

Single joint:

```bash
ros2 run quadruped_bringup argos_joint_jog --joint FR_coxa_joint
```

All joints, one at a time:

```bash
ros2 run quadruped_bringup argos_joint_jog --bidirectional
```

Use this step to confirm:

- the commanded joint name moves the expected physical joint
- the direction is correct
- no linkage binds near the neutral pose

If a joint moves backward or centers incorrectly, fix the calibration table in
`firmware/esp32c6/main/main.c` before doing any gait testing.

### 9.4 RPLidar

```bash
ros2 launch quadruped_bringup rplidar.launch.py
ros2 run quadruped_bringup argos_component_check lidar
```

Optional viewer:

```bash
ros2 launch quadruped_bringup view_lidar.launch.py
```

### 9.5 RealSense

```bash
ros2 launch quadruped_bringup realsense.launch.py
ros2 run quadruped_bringup argos_component_check realsense
```

Optional viewer:

```bash
ros2 launch quadruped_bringup view_camera.launch.py
```

### 9.6 Thermal camera

If the MLX90640 is installed:

```bash
ros2 launch quadruped_bringup mission_stack.launch.py
ros2 run quadruped_bringup argos_component_check thermal
```

If you want to force the real camera and fail hard instead of falling back to
mock data, set these parameters in `mission_stack.yaml` first:

- `thermal_node.backend: mlx90640`
- `thermal_node.fallback_to_mock: false`

### 9.7 Full system data path

Bridge + sensors:

```bash
ros2 launch quadruped_bringup esp32_bridge.launch.py
ros2 launch quadruped_bringup sensors.launch.py
ros2 run quadruped_bringup argos_component_check full
```

Add thermal:

```bash
ros2 launch quadruped_bringup mission_stack.launch.py
ros2 run quadruped_bringup argos_component_check full --include-thermal
```

---

## 10. Bench-only legacy tools

The repo already contains two direct bench utilities in `ros2_ws/argos_control/`:

- `servo_test.py`
- `single_leg_test.py`

Use them only when the ESP32 is not trying to drive the same hardware.

Important:

- do not run the Pi-side direct PCA9685 tools while the ESP32 owns the I2C bus
- do not run `web/leg_viz/server.py` with the live robot powered

These tools are good for one-leg assembly and horn alignment, but the normal
physical robot acceptance path should use `argos_joint_jog` through the ESP32.

---

## 11. Servo calibration workflow

When all joints move and nothing binds near neutral:

1. use `argos_joint_jog --joint <joint_name>` to identify each servo
2. correct `channel`, `direction`, and `offset_deg` in `firmware/esp32c6/main/main.c`
3. reflash the ESP32
4. rerun the jog test
5. only after all 12 joints are correct, test crawl / stand behavior

Keep the robot supported during calibration. Zero radians should remain a safe,
repeatable crouch / neutral test pose.

---

## 12. First integrated robot launch

Once per-component checks are passing:

```bash
ros2 launch quadruped_bringup full_system.launch.py enable_esp32:=true start_rviz:=true
```

For the expo demo profile:

```bash
ros2 launch quadruped_bringup expo_demo.launch.py enable_esp32:=true start_demo_commander:=true
```

Before using the demo commander, verify manually that:

- estop works
- the robot stands without saturating servos
- the gait planner is using the correct servo calibration

---

## 13. Quick acceptance checklist

- `argos_component_check preflight` passes
- `argos_component_check esp32` passes
- `argos_joint_jog --bidirectional` completes without a wrong joint moving
- `argos_component_check lidar` passes
- `argos_component_check realsense` passes
- `argos_component_check thermal` passes if thermal hardware is installed
- `argos_component_check full --include-thermal` passes for the fully populated robot
- `full_system.launch.py` runs cleanly

---

## 14. Common failure signatures

### `/joint_states` never appears

- bridge not running
- wrong serial device
- wrong micro-ROS transport or baud rate
- ESP32 firmware not flashed or bootlooping

### Joints twitch or the wrong leg moves

- channel map does not match the harness
- servo direction sign is wrong
- offset is wrong
- more than one controller is driving the PCA9685

### Lidar launch runs but `/scan` is empty

- wrong serial alias
- lidar motor not spinning
- CP2102 cable is power-only or marginal

### RealSense starts but topics are missing

- USB 3 bandwidth / power issue
- powered hub needed
- camera name mismatch in the checker command

### Thermal node runs but only mock data appears

- MLX90640 libraries missing on the Pi
- I2C not enabled
- `0x33` not visible on bus 1
- `fallback_to_mock` left enabled while hardware is disconnected
