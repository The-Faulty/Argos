"""Launch the RealSense and RPLiDAR bring-up together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")

    enable_realsense_arg = DeclareLaunchArgument(
        "enable_realsense",
        default_value="true",
        description="Launch the RealSense node.",
    )
    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar",
        default_value="true",
        description="Launch the RPLiDAR node.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Start RViz with the lidar config.",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="Camera namespace.",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyLIDAR",
        description="Serial device for the lidar.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="lidar_link",
        description="Laser frame published by the lidar node.",
    )
    publish_static_tf_arg = DeclareLaunchArgument(
        "publish_static_tf",
        default_value="false",
        description="Publish a temporary static TF for the lidar.",
    )
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="base_link",
        description="Parent frame for the optional lidar TF.",
    )
    lidar_x_arg = DeclareLaunchArgument("lidar_x", default_value="0.0")
    lidar_y_arg = DeclareLaunchArgument("lidar_y", default_value="0.0")
    lidar_z_arg = DeclareLaunchArgument("lidar_z", default_value="0.0")
    lidar_roll_arg = DeclareLaunchArgument("lidar_roll", default_value="0.0")
    lidar_pitch_arg = DeclareLaunchArgument("lidar_pitch", default_value="0.0")
    lidar_yaw_arg = DeclareLaunchArgument("lidar_yaw", default_value="0.0")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "realsense.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_realsense")),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rplidar.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_static_tf": LaunchConfiguration("publish_static_tf"),
            "parent_frame": LaunchConfiguration("parent_frame"),
            "lidar_x": LaunchConfiguration("lidar_x"),
            "lidar_y": LaunchConfiguration("lidar_y"),
            "lidar_z": LaunchConfiguration("lidar_z"),
            "lidar_roll": LaunchConfiguration("lidar_roll"),
            "lidar_pitch": LaunchConfiguration("lidar_pitch"),
            "lidar_yaw": LaunchConfiguration("lidar_yaw"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(bringup_dir, "config", "rplidar_view.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            enable_realsense_arg,
            enable_lidar_arg,
            start_rviz_arg,
            camera_name_arg,
            serial_port_arg,
            frame_id_arg,
            publish_static_tf_arg,
            parent_frame_arg,
            lidar_x_arg,
            lidar_y_arg,
            lidar_z_arg,
            lidar_roll_arg,
            lidar_pitch_arg,
            lidar_yaw_arg,
            realsense_launch,
            lidar_launch,
            rviz_node,
        ]
    )
