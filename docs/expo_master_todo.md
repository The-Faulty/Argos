# Argos Expo Master Todo

This is the only checklist the team should use for the final expo push.

Current demo target:

- stable stand
- controlled crawl
- live LiDAR
- live RealSense
- SLAM map in RViz
- thermal camera
- victim detection
- gas mapping
- clear safety story

Not part of the required demo:

- Gazebo
- full Nav2 autonomy

## Must do first

- [ ] Freeze servo numbering so it matches the software joint order.
- [ ] Freeze servo directions for all 12 joints.
- [ ] Freeze neutral offsets for all 12 joints.
- [ ] Freeze safe min and max angles for all 12 joints.
- [ ] Confirm the final Pi + ESP32-C6 communication path.
- [ ] Confirm the final `/dev/ttyESP32` and `/dev/ttyLIDAR` device mapping.
- [ ] Confirm real leg geometry matches `Config.py` and the URDF.
- [ ] Write down one known-good startup order for power, Pi, ESP32-C6, and ROS.
- [ ] Confirm the ESP32-C6 USB vendor:product ID with `lsusb`; update the udev rule if it is not `303a:1001`.
- [ ] Confirm the MQ-series gas sensor ADC GPIO matches `CONFIG_ARGOS_GAS_ADC_GPIO` (default `GPIO4`).
- [ ] Confirm MLX90640 is wired to the Pi I2C bus and shows `0x33` in `i2cdetect`.

## Firmware bring-up (ESP32-C6)

- [ ] Flash `firmware/esp32c6/` once the per-joint calibration table is set.
- [ ] Verify PCA9685 and LSM9DS0 both initialize on the same I2C bus without collision.
- [ ] Verify `/imu/data_raw` publishes at ~100 Hz under load.
- [ ] Verify `/joint_states` echoes the last commanded pose at ~100 Hz.
- [ ] Verify `/gas` publishes at ~10 Hz with sensible mV after a 60 s warm-up.
- [ ] Verify the watchdog releases all 12 channels when `/joint_command` stops for >250 ms (yank-cable test).
- [ ] Walk every joint through its safe min/max once at low rate to confirm direction and offset.
- [ ] Confirm micro-ROS transport menuconfig matches the agent's UART/baud (921600).
- [ ] Tag a known-good firmware build so the team knows what is on the robot.

## Power and wiring

- [ ] Servo PSU is fully separate from the Pi 5V rail (shared GND only).
- [ ] Bench-test current draw with all 12 servos active to confirm the supply does not sag.
- [ ] Confirm `web/leg_viz/server.py` is NOT running while the ESP32 is powered (two masters on the PCA9685).
- [ ] Strain-relieve the servo bus, USB cables, and I2C jumpers before any walking test.

## Pi-side verification (post-firmware)

- [ ] Run `ros2 topic hz` on `/imu/data_raw`, `/joint_states`, `/joint_command`, `/scan`, `/gas`, `/thermal/image_raw` and record the steady rates.
- [ ] Confirm RViz shows the MCU echo on `/joint_states`, not the preview publisher.
- [ ] Confirm `/estop true` drops the robot into safe crouch and ignores `/cmd_vel` until cleared.
- [ ] Confirm `/dev/ttyESP32` and `/dev/ttyLIDAR` both appear after a cold boot (not just after manual replug).

## Hardware-required work

- [ ] Verify the robot powers on without slamming joints into hard stops.
- [ ] Verify the full robot can stand on all four feet.
- [ ] Tune the stand pose so the robot is not leaning or twisting.
- [ ] Verify `e-stop` works every time.
- [ ] Verify watchdog behavior when motion commands stop.
- [ ] Verify `/joint_command` reaches the ESP32-C6 correctly.
- [ ] Verify the ESP32-C6 publishes back at least basic `/joint_states`.
- [ ] Verify IMU data is live and has the correct sign/frame convention.
- [ ] Test light push recovery in stand.
- [ ] Tune the stabilization so the robot corrects in the right direction.
- [ ] Verify the recovery stand does not overextend any joint.
- [ ] Tune crawl only after stand is solid.
- [ ] Find the slowest reliable crawl settings for demo use.
- [ ] Verify the robot can crawl a short demo path without collapsing.
- [ ] Confirm LiDAR mounting is solid.
- [ ] Confirm RealSense mounting is solid.
- [ ] Verify LiDAR launches reliably on the Pi.
- [ ] Verify RealSense launches reliably on the Pi.
- [ ] Verify LiDAR and RealSense run together without power or USB issues.
- [ ] Verify TF frames line up with the real robot in RViz.
- [ ] Run at least one full real-world mapping pass with the robot moving.
- [ ] Save one video of a successful stand, walk, and mapping run.
- [ ] Identify the safest floor surface for the live demo.
- [ ] Pack spare screws, horns, tools, cables, and fasteners after the last hardware session.

## Software-only work

- [ ] Keep the launch flow simple and consistent.
- [ ] Make sure the final full-stack launch command is documented in one place.
- [ ] Make sure the final startup order is documented in one place.
- [ ] Finish the `slam_toolbox` bring-up path.
- [ ] Verify topic names are consistent across code, launch files, and docs.
- [ ] Verify TF frame names are consistent across URDF, sensors, and RViz.
- [ ] Build one clean RViz layout for the demo.
- [ ] Make sure the RViz layout shows robot model, scan, camera, and map clearly.
- [ ] Keep configs updated with real servo limits and offsets from hardware testing.
- [ ] Keep comments concise and easy to read.
- [ ] Save screenshots for the poster and demo backup.
- [ ] Save a rosbag or screen recording of one good mapping run if possible.
- [ ] Keep firmware and ROS revisions labeled clearly so the team knows what is on the robot.
- [ ] Write a short architecture explanation for the poster and live demo.
- [ ] Write a short explanation of why the scope was narrowed.
- [ ] Write a one-page troubleshooting note for sensors, MCU, and launch issues.
- [ ] Write the fallback plan if walking is unreliable on expo day.
- [ ] Sync the per-joint calibration in firmware with `Config.py` `SERVO_LIMITS_DEG` once measured on hardware.
- [ ] Confirm `publish_joint_states_preview: false` is the right setting once the MCU echo is verified live.

## Mission stack (thermal + victim + gas)

- [ ] Verify MLX90640 thermal stream comes up at ~4 Hz with `thermal_node.backend: mlx90640`.
- [ ] Verify victim detection markers appear on `/victim_detections` for a warm test target.
- [ ] Tune the thermal hotspot threshold so judges see clean detections, not noise.
- [ ] Verify `/hazard_map` darkens when isopropyl is waved past the gas sensor.
- [ ] Confirm the mission stack can run in parallel with locomotion + SLAM without dropping rates.

## Expo and logistics

- [ ] Decide the exact 2-3 minute live demo flow.
- [ ] Decide who drives the robot.
- [ ] Decide who talks during the demo.
- [ ] Decide what RViz panels stay visible during the demo.
- [ ] Decide whether the robot walks on the floor or from a safer controlled setup.
- [ ] Make sure the poster matches the actual final system.
- [ ] Make sure everyone can explain the Pi + ESP32-C6 split.
- [ ] Make sure everyone can explain the safety story.
- [ ] Make sure everyone can explain how LiDAR, RealSense, and SLAM fit together.
- [ ] Rehearse the full demo at least three times.
- [ ] Bring the robot.
- [ ] Bring the tethered power supply.
- [ ] Bring extension cords and a power strip.
- [ ] Bring the Raspberry Pi, SD card, and backup image if possible.
- [ ] Bring the ESP32-C6 board, cable, and known-good firmware build.
- [ ] Bring the demo laptop.
- [ ] Bring the exact launch commands in a note that opens quickly.
- [ ] Bring the RViz config.
- [ ] Bring saved screenshots and a saved video of a good run.
- [ ] Bring spare servo horns, screws, zip ties, tape, and tools.
- [ ] Bring backup USB cables, adapters, and hubs if needed.

## If there is extra time

- [ ] Improve map stability with IMU fusion or EKF if it clearly helps.
- [ ] Improve the push recovery tuning during stand.
- [ ] Improve crawl smoothness and reduce foot scuffing.
- [ ] Add a cleaner teleop workflow.
- [ ] Save and reload maps cleanly for demo backup.
- [ ] Polish RViz visuals further.
- [ ] Try trot only if crawl is already reliable.
- [ ] Clean up remaining legacy files and comments.
- [ ] Explore Gazebo only if the real robot demo is already locked.

## Absolute must-haves by expo day

- [ ] Safe stand
- [ ] Controlled crawl
- [ ] Working `e-stop`
- [ ] Live LiDAR
- [ ] Live RealSense
- [ ] Working RViz view
- [ ] Working SLAM map demo
- [ ] Clear speaking plan
- [ ] Backup demo plan
