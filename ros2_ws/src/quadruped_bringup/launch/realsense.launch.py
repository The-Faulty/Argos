"""Launch the Intel RealSense D435i depth + color camera.

Streams color and depth at 640x480 @ 30fps with depth aligned to the color frame.
Point cloud and infrared streams are off by default to keep CPU load down on the Pi.

Usage:
  ros2 launch quadruped_bringup realsense.launch.py

This launches the RealSense camera with settings suitable for the Argos
quadruped. It enables:
  - Color stream at 640x480 @ 30fps
  - Depth stream at 640x480 @ 30fps
  - Aligned depth to color (for fusion later)
  - Point cloud (optional, off by default for performance)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch arguments ---
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', default_value='camera',
        description='Namespace / name for the camera node'
    )

    enable_color_arg = DeclareLaunchArgument(
        'enable_color', default_value='true',
        description='Enable the color (RGB) stream'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='true',
        description='Enable the depth stream'
    )

    enable_infra1_arg = DeclareLaunchArgument(
        'enable_infra1', default_value='false',
        description='Enable infrared stream 1'
    )

    enable_infra2_arg = DeclareLaunchArgument(
        'enable_infra2', default_value='false',
        description='Enable infrared stream 2'
    )

    pointcloud_arg = DeclareLaunchArgument(
        'pointcloud.enable', default_value='false',
        description='Enable point cloud generation (CPU intensive)'
    )

    align_depth_arg = DeclareLaunchArgument(
        'align_depth.enable', default_value='true',
        description='Align depth frames to the color camera frame'
    )

    publish_mount_tf_arg = DeclareLaunchArgument(
        'publish_mount_tf', default_value='true',
        description='Attach the RealSense driver frames to the URDF camera mount'
    )

    mount_parent_frame_arg = DeclareLaunchArgument(
        'mount_parent_frame', default_value='camera_link_1',
        description='URDF frame that represents the physical camera mount'
    )

    mount_child_frame_arg = DeclareLaunchArgument(
        'mount_child_frame', default_value='camera_link',
        description='Root frame published by the RealSense driver'
    )

    mount_x_arg = DeclareLaunchArgument('mount_x', default_value='0.0')
    mount_y_arg = DeclareLaunchArgument('mount_y', default_value='0.0')
    mount_z_arg = DeclareLaunchArgument('mount_z', default_value='0.0')
    mount_roll_arg = DeclareLaunchArgument('mount_roll', default_value='0.0')
    mount_pitch_arg = DeclareLaunchArgument('mount_pitch', default_value='0.0')
    mount_yaw_arg = DeclareLaunchArgument('mount_yaw', default_value='0.0')

    # --- RealSense Node ---
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=LaunchConfiguration('camera_name'),
        namespace=LaunchConfiguration('camera_name'),
        parameters=[{
            # --- Stream configuration ---
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_infra1': LaunchConfiguration('enable_infra1'),
            'enable_infra2': LaunchConfiguration('enable_infra2'),

            # --- Resolution & FPS ---
            'depth_module.depth_profile': '640,480,30',
            'rgb_camera.color_profile': '640,480,30',

            # --- Alignment and point cloud ---
            'align_depth.enable': LaunchConfiguration('align_depth.enable'),
            'pointcloud.enable': LaunchConfiguration('pointcloud.enable'),

            # --- Camera info ---
            'camera_name': LaunchConfiguration('camera_name'),

            # --- Quality of Service ---
            'enable_rgbd': False,
            'enable_gyro': False,
            'enable_accel': False,

            # --- Diagnostics ---
            'diagnostics_period': 1.0,
        }],
        output='screen',
        emulate_tty=True,
    )

    camera_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_static_tf',
        condition=IfCondition(LaunchConfiguration('publish_mount_tf')),
        arguments=[
            LaunchConfiguration('mount_x'),
            LaunchConfiguration('mount_y'),
            LaunchConfiguration('mount_z'),
            LaunchConfiguration('mount_yaw'),
            LaunchConfiguration('mount_pitch'),
            LaunchConfiguration('mount_roll'),
            LaunchConfiguration('mount_parent_frame'),
            LaunchConfiguration('mount_child_frame'),
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_name_arg,
        enable_color_arg,
        enable_depth_arg,
        enable_infra1_arg,
        enable_infra2_arg,
        pointcloud_arg,
        align_depth_arg,
        publish_mount_tf_arg,
        mount_parent_frame_arg,
        mount_child_frame_arg,
        mount_x_arg,
        mount_y_arg,
        mount_z_arg,
        mount_roll_arg,
        mount_pitch_arg,
        mount_yaw_arg,
        realsense_node,
        camera_mount_tf,
    ])
