"""
view_camera.launch.py — Launch RealSense + RViz to visualize camera output.

Usage (in dev container or on Pi with display forwarding):
  ros2 launch quadruped_bringup view_camera.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_dir = get_package_share_directory('quadruped_bringup')

    # Include the realsense launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'realsense.launch.py')
        ),
    )

    # RViz with a pre-made config (if it exists)
    rviz_config = os.path.join(bringup_dir, 'config', 'realsense_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.isfile(rviz_config) else [],
        output='screen',
    )

    return LaunchDescription([
        realsense_launch,
        rviz_node,
    ])
