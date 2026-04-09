"""Launch robot_state_publisher for the Argos description."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory("quadruped_description")
    xacro_file = os.path.join(description_dir, "urdf", "quadruped.urdf.xacro")
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
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz_node,
        ]
    )
