"""Launch the four Argos control nodes: command_mux, gait_planner, safety, joint_command_publisher.

Data flow once running:
  /teleop/cmd_vel or /nav/cmd_vel
      -> command_mux  -> /cmd_vel
      -> gait_planner -> /joint_command/raw
      -> safety_node  -> /joint_command/safe
      -> joint_command_publisher -> /joint_command  (to MCU via micro-ROS)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")
    # Default params live in config/control_stack.yaml — override at launch time if needed
    default_params = os.path.join(bringup_dir, "config", "control_stack.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML file with control node parameters.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo or bag time instead of wall clock.",
    )
    publish_joint_states_preview_arg = DeclareLaunchArgument(
        "publish_joint_states_preview",
        default_value="true",
        description="Mirror commanded joints to /joint_states so RViz can show the robot pose.",
    )

    # All four control nodes share the same YAML params file
    common_parameters = [
        LaunchConfiguration("params_file"),
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        },
    ]

    command_mux = Node(
        package="argos_control",
        executable="argos-command-mux",
        name="command_mux_node",
        parameters=common_parameters,
        output="screen",
    )
    gait_planner = Node(
        package="argos_control",
        executable="argos-gait-planner",
        name="gait_planner_node",
        parameters=common_parameters,
        output="screen",
    )
    safety_node = Node(
        package="argos_control",
        executable="argos-safety",
        name="safety_node",
        parameters=common_parameters,
        output="screen",
    )
    joint_command_publisher = Node(
        package="argos_control",
        executable="argos-joint-command-publisher",
        name="joint_command_publisher_node",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "publish_joint_states_preview": ParameterValue(
                    LaunchConfiguration("publish_joint_states_preview"),
                    value_type=bool,
                ),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            publish_joint_states_preview_arg,
            command_mux,
            gait_planner,
            safety_node,
            joint_command_publisher,
        ]
    )
