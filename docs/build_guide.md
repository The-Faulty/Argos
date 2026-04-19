# Argos Build Guide

End-to-end instructions for building Argos from bare hardware to a running
demo. Follow the sections in order — each one assumes the previous one is
done.

Cross-references:

- Topic contract: [ros2_topic_contract.md](ros2_topic_contract.md)
- Pi device map: [pi_hardware_contract.md](pi_hardware_contract.md)
- Demo flow: [expo_demo_runbook.md](expo_demo_runbook.md)
- Critical-path checklist: [expo_master_todo.md](expo_master_todo.md)

---

## 1. Bill of materials

### Compute

- Raspberry Pi 4 (4 GB or 8 GB), microSD card (32 GB+, A2 class), USB-C PSU
- ESP32-C6 dev board with native USB (e.g. Seeed XIAO ESP32-C6 or ESP32-C6-DevKitC-1)
- USB-C cable (Pi ↔ ESP32-C6 must be a real data cable)
- Powered USB 3.0 hub (RealSense D435/D455 needs USB 3, and the Pi has only two USB 3 ports)

### Sensors

- RPLiDAR A1M8 + supplied CP2102 USB-UART adapter
- Intel RealSense D435 or D455
- MLX90640 thermal camera breakout (I2C, addr `0x33`)
- LSM9DS0 9-axis IMU breakout (I2C: accel/mag at one addr, gyro at another)
- MQ-series gas sensor breakout (analog out — MQ-2, MQ-4, MQ-135 all work)

### Actuation

- 12 × hobby servos rated for 180° travel (one per joint)
- PCA9685 16-channel I2C PWM driver (addr `0x40`)
- Servo power supply: 5–6 V, ≥10 A bench supply (do **not** power servos from
  the Pi 5V rail)

### Misc

- Female-female jumper wires for I2C bus
- 4.7 kΩ I2C pull-up resistors (most breakouts include them — verify before
  paralleling many devices)
- Tethered AC power, extension cord, power strip for demo day

---

## 2. Wiring

### Power domains (keep separate)

| Rail | Source | Loads |
|---|---|---|
| 5 V logic | Pi USB-C PSU | Pi, ESP32-C6 (over Pi USB), all I2C breakouts |
| 5–6 V servo | bench supply | PCA9685 V+ rail only — **not** the Pi |
| GND | common | tied between all three rails |

The single most common mistake is powering 12 servos from the Pi's 5V pin.
The Pi browns out, the ESP32 resets, the demo dies. Use a separate supply
and tie grounds.

### I2C bus (single bus, four devices)

The ESP32-C6 owns one I2C bus that talks to:

| Device | Address | Notes |
|---|---|---|
| PCA9685 | `0x40` | servo PWM driver |
| LSM9DS0 (accel/mag) | `0x1D` (LSM9DS0_I2C_AM_ADDRESS_2) | |
| LSM9DS0 (gyro) | `0x6B` (LSM9DS0_I2C_G_ADDRESS_2) | same chip, two slave addresses |
| MLX90640 | `0x33` | thermal camera (currently wired to the **Pi** I2C, not the ESP32 — see note below) |

ESP32-C6 I2C pins (per `firmware/esp32c6/main/main.c`): `SDA=GPIO23`, `SCL=GPIO22`.
Add 4.7 kΩ pull-ups on each line if no breakout in the chain provides them.

> **Thermal camera placement decision pending.** Today the
> `argos_mission.thermal_node` reads the MLX90640 from the Pi's I2C
> ([pi_hardware_contract.md](pi_hardware_contract.md)). Leave it on the Pi
> unless a future change moves it. Adding it to the ESP32 bus is fine
> electrically (different address) but doubles the firmware work.

### USB

| Cable | Pi side | Other end |
|---|---|---|
| USB-A to micro-USB | Pi USB 2 | RPLiDAR A1M8 USB-UART adapter (CP2102) |
| USB-A to USB-C | Pi USB 3 (via powered hub recommended) | RealSense D435/D455 |
| USB-A to USB-C | Pi USB 2 | ESP32-C6 native USB CDC |

### Servo channel map

Channels 0–11 on the PCA9685, in this order:

| Ch | Joint | Ch | Joint |
|---|---|---|---|
| 0 | FR coxa | 6 | RR coxa |
| 1 | FR femur | 7 | RR femur |
| 2 | FR tibia | 8 | RR tibia |
| 3 | FL coxa | 9 | RL coxa |
| 4 | FL femur | 10 | RL femur |
| 5 | FL tibia | 11 | RL tibia |

This ordering is hard-coded in `firmware/esp32c6/main/main.c` (`s_servo_cal[]`)
and must match the labels you stick on the harness.

### Gas sensor

MQ analog output → ESP32-C6 `GPIO4` (default `CONFIG_ARGOS_GAS_ADC_GPIO`).
Heater on the breakout → 5 V from the same logic supply. Allow ≥60 s warm-up
before trusting readings.

---

## 3. Raspberry Pi setup

### 3.1 Flash the OS

1. Flash **Raspberry Pi OS Bookworm 64-bit** (Lite is fine) with Raspberry Pi
   Imager. In the imager's advanced options, enable SSH and set hostname/user.
2. Boot the Pi, log in over SSH.
3. `sudo apt update && sudo apt full-upgrade -y && sudo reboot`.

### 3.2 Enable I2C and SPI

```bash
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo apt install -y i2c-tools
i2cdetect -y 1   # should show 0x33 once the MLX90640 is wired
```

### 3.3 Install dependencies

You have two paths. Pick one.

**Option A — Docker (recommended, matches the dev container).**

```bash
sudo apt install -y docker.io docker-compose-plugin git
sudo usermod -aG docker $USER   # log out and back in
git clone <argos-repo> ~/argos
cd ~/argos
docker compose build
docker compose up -d
docker compose exec argos bash
# inside the container:
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

**Option B — Native ROS 2 Humble install.** Follow
[docs.ros.org humble install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html),
then install the same apt packages listed in the project `Dockerfile`
(`ros-humble-slam-toolbox`, `ros-humble-realsense2-camera`,
`ros-humble-rplidar-ros`, `ros-humble-micro-ros-agent`,
`ros-humble-robot-state-publisher`, `ros-humble-xacro`,
`ros-humble-robot-localization`, `ros-humble-rviz2`), plus pip:
`transforms3d numpy scipy matplotlib opencv-python-headless pyrealsense2`.

For the thermal sensor reader on the Pi:

```bash
sudo apt install -y python3-pip python3-smbus
python3 -m pip install adafruit-blinka adafruit-circuitpython-mlx90640
```

### 3.4 Install udev rules

```bash
sudo cp ros2_ws/src/quadruped_bringup/config/99-argos-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyLIDAR /dev/ttyESP32   # both symlinks should exist
```

If `/dev/ttyESP32` does not appear, plug the ESP32 in and run `lsusb`. If the
vendor:product is **not** `303a:1001`, edit
`ros2_ws/src/quadruped_bringup/config/99-argos-devices.rules` to match what
you see, then re-run the commands above.

### 3.5 Set ROS environment

In `~/.bashrc` (or the container's startup):

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/argos/ros2_ws/install/setup.bash
```

### 3.6 Build the workspace

```bash
cd ~/argos/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 4. ESP32-C6 firmware setup

### 4.1 Install ESP-IDF v5.x

On any host (Linux, macOS, Windows). The Pi can do it too but a laptop is
faster for the first flash.

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
. ./export.sh   # run this in every new shell
```

### 4.2 Clone the micro-ROS component as a sibling

The firmware's `CMakeLists.txt` expects `micro_ros_espidf_component` to live
**next to** the `argos` repo, not inside it:

```
~/dev/
├── argos/
│   └── firmware/esp32c6/
└── micro_ros_espidf_component/
```

```bash
cd ~/dev
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git
```

### 4.3 Configure

```bash
cd ~/dev/argos/firmware/esp32c6
idf.py set-target esp32c6
idf.py menuconfig
```

In the menu, open **"Argos firmware settings"** and confirm:

- `PCA9685 I2C address` = `0x40`
- `GPIO used for the MQ-series gas sensor` = whatever you wired (default `4`)
- `Joint-command watchdog timeout (ms)` = `250`
- `IMU + /joint_states control loop rate (Hz)` = `100`
- `Gas sensor publish rate (Hz)` = `10`

Then under **"micro-ROS Settings → micro-ROS Transport Settings"**, set the
transport to **"Custom"** and confirm UART pins / baud match what the Pi side
expects (the agent launch uses `921600` on `/dev/ttyESP32`).

Save and exit.

### 4.4 Build, flash, monitor

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash       # use the ACM/COM port you see on plug-in
idf.py -p /dev/ttyACM0 monitor     # Ctrl-] to exit
```

You should see the firmware enumerate the LSM9DS0, init the PCA9685, and
start the micro-ROS task. Once the Pi-side agent connects, you'll see
`/imu/data_raw`, `/joint_states`, and `/gas` traffic in the monitor log.

### 4.5 Per-joint calibration (one-time per build)

Once the robot is mechanically assembled and you can identify each joint:

1. Bring the agent up on the Pi: `ros2 launch quadruped_bringup esp32_bridge.launch.py`.
2. Send a single joint to zero radians:
   `ros2 topic pub --once /joint_command sensor_msgs/JointState "{name: ['FR_coxa_joint'], position: [0.0]}"`.
3. If the joint is not centered, edit `s_servo_cal[]` in
   `firmware/esp32c6/main/main.c`: tweak `offset_deg` to recenter, flip
   `direction` if the joint moves the wrong way, tighten `min_deg`/`max_deg`
   to the safe travel measured by hand.
4. Rebuild + flash. Repeat for all 12 joints.

This is the work captured by the "Freeze servo numbering / directions /
neutral offsets / safe min and max" entries in
[expo_master_todo.md](expo_master_todo.md).

---

## 5. Bring-up order

Power-on order matters because the watchdog will release servos if the Pi
isn't talking yet:

1. **Servo PSU off**, Pi PSU off.
2. Plug all USB cables in.
3. Power the Pi. Wait for SSH to come up.
4. SSH in, `cd ~/argos`, attach to the container or source the workspace.
5. Power the servo PSU. Servos will start at neutral once the firmware
   initializes them — keep the robot supported in a crouch posture so it
   doesn't fall.
6. Launch the demo stack:

```bash
ros2 launch quadruped_bringup expo_demo.launch.py enable_esp32:=true start_demo_commander:=true
```

To shut down: stop the launch (Ctrl-C), then power off the servo PSU
**before** unplugging the ESP32.

---

## 6. Verification

Run these in separate terminals after `expo_demo.launch.py` is up.

```bash
ros2 topic hz /imu/data_raw     # ~100 Hz
ros2 topic hz /joint_states     # ~100 Hz (echo from MCU, not preview)
ros2 topic hz /joint_command    # ~100 Hz
ros2 topic hz /scan             # 5–10 Hz (RPLiDAR A1M8)
ros2 topic hz /gas              # ~10 Hz
ros2 topic hz /thermal/image_raw   # ~4 Hz (mission stack only)
```

Visual checks in RViz: robot model present, `/scan` lines on the floor plane,
RealSense color and depth windows live, SLAM `/map` filling in as you walk
the robot.

Failure-mode checks:

- Yank the ESP32 USB cable. Servos should sag (watchdog limp), not freeze in
  the last commanded pose.
- Publish `/estop true`. Robot should drop into safe crouch and stop
  responding to `/cmd_vel` until you publish `/estop false`.

---

## 7. Repo layout reference

| Path | What it is |
|---|---|
| `firmware/esp32c6/` | ESP-IDF + micro-ROS firmware (servos, IMU, gas) |
| `firmware/esp32c6/main/main.c` | Joint calibration table lives here |
| `firmware/esp32c6/components/pca9685/` | I2C PWM driver |
| `firmware/esp32c6/components/lsm9ds0/` | IMU driver |
| `ros2_ws/src/argos_control/` | Node entry points (gait, safety, joint cmd) |
| `ros2_ws/src/argos_mission/` | Mission state, gas hazard map, thermal/victim |
| `ros2_ws/src/quadruped_bringup/` | Launch files, sensor configs, udev rules |
| `ros2_ws/src/quadruped_description/` | URDF, RViz layouts |
| `ros2_ws/argos_control/` | Leg math, single-leg bench tools |
| `web/leg_viz/` | **Bench-only** PCA9685 web tester — do not run with ESP32 powered |

---

## 8. Common pitfalls

- **Two masters on the I2C bus.** Don't run `web/leg_viz/server.py` on the Pi
  while the ESP32 firmware is powered. Both will try to drive the PCA9685
  and both will lose.
- **Pi 5V brownout from servos.** Servo current spikes will reset the Pi if
  you share rails. Use a separate supply.
- **micro-ROS transport mismatch.** If the agent says "client connection
  timeout" forever, the firmware menuconfig transport doesn't match the
  agent launch (UART vs USB-CDC, wrong baud).
- **udev symlink missing after replug.** If `/dev/ttyESP32` disappears,
  `lsusb` to confirm the device is enumerated, then check the udev rule
  vendor/product IDs.
- **MLX90640 not detected.** `i2cdetect -y 1` on the Pi must show `33`. If it
  doesn't, check pull-ups and that I2C is enabled in `raspi-config`.
- **Thermal node falls back to mock.** If you want the real camera and never
  the simulated one, set `thermal_node.backend: mlx90640` and
  `thermal_node.fallback_to_mock: false` in `mission_stack.yaml`.
