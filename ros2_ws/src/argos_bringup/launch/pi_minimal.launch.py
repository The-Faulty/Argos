"""Minimal Pi bring-up for the current robot: Pi + ESP32, no IMU/cameras/lidar."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")
    minimal_params = os.path.join(bringup_dir, "config", "control_stack_minimal.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time instead of wall clock time.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Start RViz with the robot description.",
    )
    enable_esp32_arg = DeclareLaunchArgument(
        "enable_esp32",
        default_value="true",
        description="Launch the micro-ROS serial bridge to the ESP32.",
    )
    serial_device_arg = DeclareLaunchArgument(
        "serial_device",
        default_value="/dev/ttyESP32",
        description="ESP32 serial device or /dev/serial/by-id path.",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="micro-ROS serial baud rate.",
    )
    control_params_file_arg = DeclareLaunchArgument(
        "control_params_file",
        default_value=minimal_params,
        description="Control-stack parameter file for minimal hardware bring-up.",
    )

    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "full_system.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "enable_sensors": "false",
            "enable_esp32": LaunchConfiguration("enable_esp32"),
            "enable_mission": "false",
            "enable_dashboard": "false",
            "control_params_file": LaunchConfiguration("control_params_file"),
            "publish_joint_states_preview": "false",
            "esp32_serial_device": LaunchConfiguration("serial_device"),
            "esp32_baudrate": LaunchConfiguration("baudrate"),
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            start_rviz_arg,
            enable_esp32_arg,
            serial_device_arg,
            baudrate_arg,
            control_params_file_arg,
            full_system_launch,
        ]
    )
