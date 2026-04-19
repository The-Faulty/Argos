"""Publish the Argos URDF, TF tree, and optionally open RViz.

Reads Argos.xacro from the Argos_description package, processes it with xacro,
and feeds it to robot_state_publisher so all TF frames are available to the
rest of the stack. The imported model roots at base_link, so this launch also
adds a zero-offset base_footprint -> base_link transform for the rest of the
stack.

Usage:
  ros2 launch quadruped_bringup state_publisher.launch.py
  ros2 launch quadruped_bringup state_publisher.launch.py start_rviz:=true
  ros2 launch quadruped_bringup state_publisher.launch.py use_joint_state_gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory("Argos_description")
    xacro_file = os.path.join(description_dir, "urdf", "Argos.xacro")
    rviz_config = os.path.join(description_dir, "config", "robot_model.rviz")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo or bag time.",
    )
    use_joint_state_gui_arg = DeclareLaunchArgument(
        "use_joint_state_gui",
        default_value="false",
        description="Launch joint_state_publisher_gui for manual pose checks.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Launch RViz with the robot model config.",
    )
    publish_base_footprint_tf_arg = DeclareLaunchArgument(
        "publish_base_footprint_tf",
        default_value="true",
        description="Publish a zero-offset base_footprint -> base_link transform.",
    )

    robot_description = Command(["xacro ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        ],
        output="screen",
    )

    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_static_tf",
        condition=IfCondition(LaunchConfiguration("publish_base_footprint_tf")),
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "base_footprint",
            "base_link",
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_joint_state_gui_arg,
            start_rviz_arg,
            publish_base_footprint_tf_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            base_footprint_tf,
            rviz_node,
        ]
    )
