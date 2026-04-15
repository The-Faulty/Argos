"""Launch the RPLiDAR A1M8 2D lidar node.

The lidar publishes LaserScan messages on /scan (remappable via topic_name).
An optional static TF broadcaster is included for bench testing when no URDF is loaded.

Examples:
  ros2 launch quadruped_bringup rplidar.launch.py
  ros2 launch quadruped_bringup rplidar.launch.py serial_port:=/dev/ttyLIDAR
  ros2 launch quadruped_bringup rplidar.launch.py publish_static_tf:=true lidar_x:=0.12 lidar_z:=0.08
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyLIDAR",
        description="Serial device for the lidar.",
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="115200",
        description="Baud rate for the A1M8.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="lidar_link",
        description="Frame published in the LaserScan messages.",
    )
    topic_name_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="/scan",
        description="LaserScan topic name.",
    )
    scan_mode_arg = DeclareLaunchArgument(
        "scan_mode",
        default_value="Standard",
        description="Lidar scan mode.",
    )
    publish_static_tf_arg = DeclareLaunchArgument(
        "publish_static_tf",
        default_value="false",
        description="Publish a temporary static TF for bench bring-up.",
    )
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="base_link",
        description="Parent frame for the optional static TF.",
    )
    lidar_x_arg = DeclareLaunchArgument("lidar_x", default_value="0.0")
    lidar_y_arg = DeclareLaunchArgument("lidar_y", default_value="0.0")
    lidar_z_arg = DeclareLaunchArgument("lidar_z", default_value="0.0")
    lidar_roll_arg = DeclareLaunchArgument("lidar_roll", default_value="0.0")
    lidar_pitch_arg = DeclareLaunchArgument("lidar_pitch", default_value="0.0")
    lidar_yaw_arg = DeclareLaunchArgument("lidar_yaw", default_value="0.0")

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": LaunchConfiguration("serial_baudrate"),
                "frame_id": LaunchConfiguration("frame_id"),
                "scan_mode": LaunchConfiguration("scan_mode"),
                "inverted": False,
                "angle_compensate": True,
            }
        ],
        remappings=[("scan", LaunchConfiguration("topic_name"))],
        output="screen",
        emulate_tty=True,
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_static_tf",
        condition=IfCondition(LaunchConfiguration("publish_static_tf")),
        arguments=[
            LaunchConfiguration("lidar_x"),
            LaunchConfiguration("lidar_y"),
            LaunchConfiguration("lidar_z"),
            LaunchConfiguration("lidar_yaw"),
            LaunchConfiguration("lidar_pitch"),
            LaunchConfiguration("lidar_roll"),
            LaunchConfiguration("parent_frame"),
            LaunchConfiguration("frame_id"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            serial_port_arg,
            serial_baudrate_arg,
            frame_id_arg,
            topic_name_arg,
            scan_mode_arg,
            publish_static_tf_arg,
            parent_frame_arg,
            lidar_x_arg,
            lidar_y_arg,
            lidar_z_arg,
            lidar_roll_arg,
            lidar_pitch_arg,
            lidar_yaw_arg,
            rplidar_node,
            static_tf_node,
        ]
    )
