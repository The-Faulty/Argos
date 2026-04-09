"""Launch the micro-ROS agent for the Teensy bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_device_arg = DeclareLaunchArgument(
        "serial_device",
        default_value="/dev/ttyTEENSY",
        description="Serial device used by the Teensy.",
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
