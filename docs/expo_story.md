# Argos Expo Story

These are the talking points that match the current software and launch flow.
They are written to be technically honest and easy to repeat live.

## Pi and ESP32-C6 split

Argos is split on purpose:

- The Raspberry Pi runs ROS 2, RViz, LiDAR, RealSense, mapping, and the higher
  level control nodes.
- The ESP32-C6 is the real-time side. It is the right place for the fast servo
  loop, IMU polling, watchdog behavior, and the final actuation path.
- For the expo mapping path, the Pi also publishes a lightweight demo odometry
  estimate so `slam_toolbox` has an `odom` frame before fused state estimation
  is ready.

Why this matters:

- Linux on the Pi is great for perception and orchestration, but it is not the
  safest place to generate tight servo timing.
- The microcontroller keeps the actuation path simple and deterministic.
- If the Pi stalls or command traffic stops, the MCU-side watchdog can still
  force a safe behavior.

## Safety story

The safety path is layered, not a single feature:

- `command_mux_node.py` gives `e-stop` top priority, then teleop, then nav,
  then idle zero.
- `safety_node.py` clamps every joint target to configured limits.
- `safety_node.py` also rate-limits joint motion so targets do not jump.
- If raw commands go stale, the safety node falls back to the crouch posture
  instead of holding the last motion indefinitely.
- The control stack starts in `crouch` by design, so the robot boots in a
  conservative state.

The live takeaway for judges is simple: motion commands are filtered before they
ever reach the MCU, and loss of commands pushes the robot toward a safer
posture.

## Uneven-terrain story

Argos is not just following a canned gait on a flat floor.

- The gait planner already accepts IMU data and uses it for body stabilization.
- It also supports a widened recovery stance if tilt exceeds the configured
  threshold.
- The foothold checker scores candidate touchdown locations against aligned
  RealSense depth.
- Safe touchdown adjustments are published back into the gait planner, and the
  RViz layout shows those foothold markers live.

That lets the team say, truthfully, that the robot is being built with uneven
terrain in mind even though the expo demo is still constrained and supervised.

## Scope story

The strongest story is disciplined scope, not inflated scope.

Argos intentionally prioritizes:

- reliable stand and crawl
- believable safety behavior
- live LiDAR and RealSense bring-up
- a visible SLAM map in RViz

Argos intentionally does not claim, yet:

- full Nav2 autonomy
- production thermal victim detection
- finished gas hazard mapping
- Gazebo as part of the expo-critical path

That is a mature engineering decision. A stable mobility and mapping demo is
more credible than a broad autonomy claim that is not ready for a live floor.

One honest caveat to say out loud if asked: the expo mapping path currently uses
command-integrated odometry as a demo bridge. That is a practical demo choice,
not the final state-estimation architecture.

## Files behind the story

- Mobility and command arbitration:
  `ros2_ws/argos_control/command_mux_node.py`
  `ros2_ws/argos_control/gait_planner_node.py`
- Safety filtering:
  `ros2_ws/argos_control/safety_node.py`
  `ros2_ws/argos_control/joint_command_publisher_node.py`
- Uneven-terrain hooks:
  `ros2_ws/argos_control/foothold_checker_node.py`
- Demo launch path:
  `ros2_ws/src/quadruped_bringup/launch/expo_demo.launch.py`
  `ros2_ws/src/quadruped_bringup/launch/slam.launch.py`
