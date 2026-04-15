"""Launch the repeatable Argos expo demo stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo or bag time.",
    )
    enable_esp32_arg = DeclareLaunchArgument(
        "enable_esp32",
        default_value="false",
        description="Launch the micro-ROS serial bridge to the ESP32-C6.",
    )
    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar",
        default_value="true",
        description="Launch the RPLiDAR bring-up.",
    )
    enable_realsense_arg = DeclareLaunchArgument(
        "enable_realsense",
        default_value="true",
        description="Launch the RealSense bring-up.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Launch RViz with the expo demo layout.",
    )
    start_slam_arg = DeclareLaunchArgument(
        "start_slam",
        default_value="true",
        description="Launch slam_toolbox for live mapping.",
    )
    start_demo_odometry_arg = DeclareLaunchArgument(
        "start_demo_odometry",
        default_value="true",
        description="Publish lightweight odometry from the muxed command for SLAM.",
    )
    start_demo_commander_arg = DeclareLaunchArgument(
        "start_demo_commander",
        default_value="false",
        description="Publish a repeatable stand-to-crawl demo profile.",
    )
    publish_joint_states_preview_arg = DeclareLaunchArgument(
        "publish_joint_states_preview",
        default_value="true",
        description="Mirror commanded joints to /joint_states for visualization.",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="Camera namespace used by the RealSense launch.",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyLIDAR",
        description="Serial device for the lidar.",
    )
    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(bringup_dir, "config", "slam_toolbox.yaml"),
        description="SLAM parameter file.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "config", "expo_demo.rviz"),
        description="RViz config for the expo demo.",
    )
    demo_startup_delay_arg = DeclareLaunchArgument(
        "demo_startup_delay_s",
        default_value="3.0",
        description="How long the demo commander holds crouch before standing.",
    )
    demo_stand_duration_arg = DeclareLaunchArgument(
        "demo_stand_duration_s",
        default_value="3.0",
        description="How long the demo commander holds stand before crawling.",
    )
    demo_crawl_duration_arg = DeclareLaunchArgument(
        "demo_crawl_duration_s",
        default_value="18.0",
        description="How long the demo commander publishes crawl commands.",
    )
    demo_settle_duration_arg = DeclareLaunchArgument(
        "demo_settle_duration_s",
        default_value="3.0",
        description="How long the demo commander settles back to stand.",
    )
    demo_loop_arg = DeclareLaunchArgument(
        "demo_loop",
        default_value="false",
        description="Loop the demo commander profile.",
    )
    demo_linear_x_arg = DeclareLaunchArgument(
        "demo_linear_x",
        default_value="0.08",
        description="Forward velocity used by the demo commander.",
    )
    demo_angular_z_arg = DeclareLaunchArgument(
        "demo_angular_z",
        default_value="0.18",
        description="Yaw rate used by the demo commander.",
    )

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_rviz": "false",
        }.items(),
    )

    control_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "control_stack.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_joint_states_preview": LaunchConfiguration(
                "publish_joint_states_preview"
            ),
        }.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "sensors.launch.py")
        ),
        launch_arguments={
            "enable_realsense": LaunchConfiguration("enable_realsense"),
            "enable_lidar": LaunchConfiguration("enable_lidar"),
            "start_rviz": "false",
            "camera_name": LaunchConfiguration("camera_name"),
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
    )

    esp32_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "esp32_bridge.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_esp32")),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "slam.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_slam")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("slam_params_file"),
        }.items(),
    )

    demo_odometry = Node(
        package="argos_control",
        executable="argos-command-odometry",
        name="command_odometry_node",
        condition=IfCondition(LaunchConfiguration("start_demo_odometry")),
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
        output="screen",
    )

    demo_commander = Node(
        package="argos_control",
        executable="argos-demo-commander",
        name="demo_commander_node",
        condition=IfCondition(LaunchConfiguration("start_demo_commander")),
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "startup_delay_s": LaunchConfiguration("demo_startup_delay_s"),
                "stand_duration_s": LaunchConfiguration("demo_stand_duration_s"),
                "crawl_duration_s": LaunchConfiguration("demo_crawl_duration_s"),
                "settle_duration_s": LaunchConfiguration("demo_settle_duration_s"),
                "loop_profile": ParameterValue(
                    LaunchConfiguration("demo_loop"), value_type=bool
                ),
                "linear_x": LaunchConfiguration("demo_linear_x"),
                "yaw_rate": LaunchConfiguration("demo_angular_z"),
            }
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            enable_esp32_arg,
            enable_lidar_arg,
            enable_realsense_arg,
            start_rviz_arg,
            start_slam_arg,
            start_demo_odometry_arg,
            start_demo_commander_arg,
            publish_joint_states_preview_arg,
            camera_name_arg,
            serial_port_arg,
            slam_params_file_arg,
            rviz_config_arg,
            demo_startup_delay_arg,
            demo_stand_duration_arg,
            demo_crawl_duration_arg,
            demo_settle_duration_arg,
            demo_loop_arg,
            demo_linear_x_arg,
            demo_angular_z_arg,
            state_publisher_launch,
            control_stack_launch,
            sensors_launch,
            esp32_launch,
            slam_launch,
            demo_odometry,
            demo_commander,
            rviz_node,
        ]
    )
