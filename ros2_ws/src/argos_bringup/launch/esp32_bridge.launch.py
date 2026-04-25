"""Start the micro-ROS serial agent that bridges the ESP32-C6 to the ROS 2 network.

The ESP32 runs micro-ROS firmware and communicates over UART at 921600 baud.
This agent forwards those messages onto the ROS 2 DDS bus so the control nodes
can talk to the servo driver and IMU as if they were normal ROS topics.

Usage:
  ros2 launch quadruped_bringup esp32_bridge.launch.py
  ros2 launch quadruped_bringup esp32_bridge.launch.py serial_device:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_device_arg = DeclareLaunchArgument(
        "serial_device",
        default_value="/dev/ttyESP32",
        description="Serial device used by the ESP32-C6.",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="921600",
        description="micro-ROS serial baud rate.",
    )

    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=[
            "serial",
            "--dev",
            LaunchConfiguration("serial_device"),
            "--baudrate",
            LaunchConfiguration("baudrate"),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([serial_device_arg, baudrate_arg, micro_ros_agent])
