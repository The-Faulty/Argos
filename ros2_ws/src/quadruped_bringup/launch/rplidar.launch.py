"""
rplidar.launch.py — Launch the RPLidar A1M8 2D lidar node.

Usage:
  ros2 launch quadruped_bringup rplidar.launch.py
  ros2 launch quadruped_bringup rplidar.launch.py serial_port:=/dev/ttyUSB1

This launches the RPLidar A1M8 with settings suitable for the Argos
quadruped. The A1M8 specs:
  - 360° scan
  - 8000 samples/s (typical)
  - 5.5 Hz scan rate (default)
  - 0.15m – 12m range
  - UART/USB interface (CP2102 USB-UART bridge)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch arguments ---
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for the RPLidar (usually /dev/ttyUSB0)'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate', default_value='115200',
        description='Baud rate for the RPLidar A1M8'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='laser_frame',
        description='TF frame ID for the lidar'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode', default_value='Standard',
        description='Scan mode: Standard, Express, Boost, Sensitivity, Stability'
    )

    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value='/scan',
        description='Topic name for the LaserScan output'
    )

    # --- RPLidar Node ---
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_mode': LaunchConfiguration('scan_mode'),
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
        remappings=[
            ('scan', LaunchConfiguration('topic_name')),
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        scan_mode_arg,
        topic_name_arg,
        rplidar_node,
    ])
