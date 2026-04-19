# Argos Pi Hardware Contract

## Device mapping

- RPLiDAR A1M8: `/dev/ttyLIDAR`
- ESP32-C6 micro-ROS client: `/dev/ttyESP32`
- RealSense D435/D455: USB 3.0 camera device
- MLX90640: I2C address `0x33`

## MLX90640 setup on the Pi

1. Enable I2C in Raspberry Pi configuration.
2. Confirm the camera is visible on bus 1:

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

You should see `33` in the scan table.

3. Install the Python libraries used by `argos_mission.thermal_node`:

```bash
sudo apt install -y python3-pip python3-smbus
python3 -m pip install adafruit-blinka adafruit-circuitpython-mlx90640
```

Example udev rules live in:

- `ros2_ws/src/quadruped_bringup/config/99-argos-devices.rules`

## Install the udev rules on the Pi

```bash
sudo cp ros2_ws/src/quadruped_bringup/config/99-argos-devices.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Bring-up order

1. Confirm the serial aliases exist:

```bash
ls -l /dev/ttyLIDAR /dev/ttyESP32
```

2. Start the micro-ROS agent:

```bash
ros2 launch quadruped_bringup esp32_bridge.launch.py
```

3. Start the robot description and control stack:

```bash
ros2 launch quadruped_bringup state_publisher.launch.py
ros2 launch quadruped_bringup control_stack.launch.py
```

4. Start sensors:

```bash
ros2 launch quadruped_bringup sensors.launch.py
```

5. Or launch the full bench stack:

```bash
ros2 launch quadruped_bringup full_system.launch.py enable_esp32:=true start_rviz:=true
```

6. Launch the mission stack with thermal victim detection:

```bash
ros2 launch quadruped_bringup mission_stack.launch.py
```

To require a real MLX90640 and disable the simulated fallback, set these
parameters in `ros2_ws/src/quadruped_bringup/config/mission_stack.yaml`:

- `thermal_node.backend: mlx90640`
- `thermal_node.fallback_to_mock: false`

## Notes

- The control stack currently mirrors commanded joints to `/joint_states` for bench visualization.
- Once the ESP32-C6 publishes real `/joint_states`, disable the preview publisher in `control_stack.launch.py`.
- The RPLiDAR launch defaults to `frame_id:=lidar_link` and publishes a static TF from `base_link` because the imported `Argos_description` model does not define a lidar frame.
