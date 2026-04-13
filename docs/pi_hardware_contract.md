# Argos Pi Hardware Contract

## Device mapping

- RPLiDAR A1M8: `/dev/ttyLIDAR`
- ESP32-C6 micro-ROS client: `/dev/ttyESP32`
- RealSense D435/D455: USB 3.0 camera device
- MLX90640: I2C address `0x33`

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

## Notes

- The control stack currently mirrors commanded joints to `/joint_states` for bench visualization.
- Once the ESP32-C6 publishes real `/joint_states`, disable the preview publisher in `control_stack.launch.py`.
- The RPLiDAR launch defaults to `frame_id:=lidar_link` to match the robot description.
