"""Launch the Argos mission and perception bench stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")
    default_params = os.path.join(bringup_dir, "config", "mission_stack.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML file with mission stack parameters.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Gazebo or bag time.",
    )

    common_parameters = [
        LaunchConfiguration("params_file"),
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        },
    ]

    thermal_node = Node(
        package="argos_mission",
        executable="argos-thermal",
        name="thermal_node",
        parameters=common_parameters,
        output="screen",
    )
    victim_detector = Node(
        package="argos_mission",
        executable="argos-victim-detector",
        name="victim_detector_node",
        parameters=common_parameters,
        output="screen",
    )
    gas_mapping = Node(
        package="argos_mission",
        executable="argos-gas-mapping",
        name="gas_mapping_node",
        parameters=common_parameters,
        output="screen",
    )
    mission_orchestrator = Node(
        package="argos_mission",
        executable="argos-mission-orchestrator",
        name="mission_orchestrator_node",
        parameters=common_parameters,
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            thermal_node,
            victim_detector,
            gas_mapping,
            mission_orchestrator,
        ]
    )
