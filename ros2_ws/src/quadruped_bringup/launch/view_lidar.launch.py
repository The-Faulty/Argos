"""Launch the RPLiDAR node with RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")
    rviz_config = os.path.join(bringup_dir, "config", "rplidar_view.rviz")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyLIDAR",
        description="Serial device for the lidar.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="lidar_link",
        description="Laser frame used in RViz.",
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rplidar.launch.py")
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "frame_id": LaunchConfiguration("frame_id"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([serial_port_arg, frame_id_arg, lidar_launch, rviz_node])
