"""Top-level launch file — brings up the full Argos system.

What gets started:
  - state_publisher  — URDF + TF tree (always on)
  - control_stack    — motion pipeline (always on)
  - sensors          — LiDAR + RealSense (enable_sensors, default true)
  - esp32_bridge     — micro-ROS serial agent (enable_esp32, default false)
  - mission_stack    — thermal, gas, victim detection (enable_mission, default false)

Example to start everything:
  ros2 launch quadruped_bringup full_system.launch.py enable_esp32:=true enable_mission:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo or bag time.",
    )
    enable_sensors_arg = DeclareLaunchArgument(
        "enable_sensors",
        default_value="true",
        description="Launch the LiDAR and RealSense bring-up.",
    )
    enable_esp32_arg = DeclareLaunchArgument(
        "enable_esp32",
        default_value="false",
        description="Launch the micro-ROS serial agent.",
    )
    enable_mission_arg = DeclareLaunchArgument(
        "enable_mission",
        default_value="false",
        description="Launch the mission/perception bench nodes.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Start RViz in the state publisher and sensor launches.",
    )

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_rviz": "false",
        }.items(),
    )

    control_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "control_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "sensors.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_sensors")),
        launch_arguments={
            "start_rviz": LaunchConfiguration("start_rviz"),
        }.items(),
    )

    esp32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "esp32_bridge.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_esp32")),
    )
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "mission_stack.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_mission")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            enable_sensors_arg,
            enable_esp32_arg,
            enable_mission_arg,
            start_rviz_arg,
            state_publisher_launch,
            control_stack_launch,
            sensors_launch,
            esp32_launch,
            mission_launch,
        ]
    )
