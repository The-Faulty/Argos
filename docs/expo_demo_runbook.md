# Argos Expo Demo Runbook

This is the repeatable launch path for the judge-facing mobility and mapping
demo. It is intentionally focused on the parts that are most believable live:
safe locomotion, live sensing, and visible mapping.

## One launch path

Robot-connected demo:

```bash
ros2 launch quadruped_bringup expo_demo.launch.py enable_esp32:=true start_demo_commander:=true
```

Bench or dry-run demo:

```bash
ros2 launch quadruped_bringup expo_demo.launch.py enable_esp32:=false start_demo_commander:=false
```

## What the launch brings up

- `robot_state_publisher` with the quadruped model and sensor frames
- the ROS 2 control stack
- RPLiDAR and RealSense
- a lightweight command-integrated odometry estimate for the mapping demo
- `slam_toolbox` for live mapping
- one RViz layout that shows the map, robot model, LiDAR, camera feeds, TF,
  and foothold markers
- an optional demo commander that publishes a slow stand-to-crawl profile

## Default demo commander behavior

When `start_demo_commander:=true`, the robot follows a simple and repeatable
sequence:

1. Hold `crouch` for startup safety.
2. Transition to `stand`.
3. Publish a slow `crawl` command with a gentle yaw arc for mapping.
4. Return to `stand`.

The default crawl profile can be tuned at launch time:

```bash
ros2 launch quadruped_bringup expo_demo.launch.py \
  enable_esp32:=true \
  start_demo_commander:=true \
  demo_linear_x:=0.06 \
  demo_angular_z:=0.12 \
  demo_crawl_duration_s:=24.0
```

## RViz checklist

The `expo_demo.rviz` layout is meant to show the whole story in one window:

- `/map` so judges can see live SLAM output
- `/scan` so the lidar data source is visible
- `/camera/camera/color/image_raw` and aligned depth so the RealSense is obvious
- `/robot_description` and TF so the body and sensor placement are easy to explain
- `/foothold/markers` so the uneven-terrain story is visible without changing the
  motion demo

The mapping path is intentionally honest: until fused odometry is available from
hardware, the expo launch uses a lightweight command-based `odom` estimate to
seed `slam_toolbox`. That is good enough for a live demo, and it can be turned
off later when real odometry replaces it.

## Operator checklist

1. Power the robot in a tethered and recoverable setup.
2. Confirm `/dev/ttyESP32` and `/dev/ttyLIDAR` exist on the Pi.
3. Launch the expo demo stack.
4. Verify RViz shows robot model, scan, color, depth, and map panels updating.
5. Verify the robot stands before enabling the crawl profile.
6. Keep the hardware `e-stop` within reach during motion.

## Fast fallback

If locomotion is not trustworthy enough for the floor:

- launch the same stack with `start_demo_commander:=false`
- keep RViz, LiDAR, RealSense, and SLAM visible
- move the robot carefully by hand, on a stand, or on a cart

That still demonstrates sensing, TF alignment, and live mapping without
over-claiming autonomy.
