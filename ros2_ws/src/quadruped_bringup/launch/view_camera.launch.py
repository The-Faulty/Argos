"""Quick debug launch — start the RealSense and open RViz to check the camera feed.

Useful for verifying the camera is working before running the full system.
Requires a display (run locally or with X11 forwarding from the Pi).

Usage:
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
    description_dir = get_package_share_directory('Argos_description')

    # Include the realsense launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'realsense.launch.py')
        ),
    )

    # Prefer a camera-specific config when present, otherwise fall back to the
    # checked-in robot model RViz layout from the imported description package.
    rviz_config = os.path.join(bringup_dir, 'config', 'realsense_view.rviz')
    if not os.path.isfile(rviz_config):
        rviz_config = os.path.join(description_dir, 'config', 'robot_model.rviz')
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
