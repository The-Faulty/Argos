"""Bring up everything the Argos dashboard needs.

Starts three things:
  - rosbridge_websocket  — browser <-> ROS bridge on port 9090
  - dashboard_bridge_node — translates dashboard intents into /joint_command/raw
  - node pi_server/server.js — the Node.js HTTP/WS dashboard server

The dashboard/ folder sits at the repo top level, not inside ros2_ws — we
resolve it relative to this launch file rather than via a ROS package share.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _dashboard_server_dir() -> str:
    """Resolve <repo>/dashboard using the launch file's on-disk location.

    When the bringup package is installed into install/share/.../launch the
    path above install/ is still the same repo, so walking up from __file__
    reliably finds the dashboard/ directory.
    """
    here = Path(__file__).resolve()
    # Walk up until we find a sibling dashboard/ directory. Handles both the
    # source tree and the colcon install tree.
    for parent in here.parents:
        candidate = parent / "dashboard"
        if (candidate / "pi_server" / "server.js").exists():
            return str(candidate)
    # Fall back to <repo>/dashboard relative to ros2_ws/src/quadruped_bringup/launch
    return str(here.parents[4] / "dashboard")


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_bringup")
    dashboard_dir = _dashboard_server_dir()

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="rosbridge_websocket port.",
    )
    dashboard_server_arg = DeclareLaunchArgument(
        "dashboard_server_dir",
        default_value=dashboard_dir,
        description="Directory containing pi_server/server.js.",
    )

    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rosbridge.launch.py")
        ),
        launch_arguments={"port": LaunchConfiguration("port")}.items(),
    )

    # Dashboard bridge — runs as a standalone node here so this launch file
    # is self-contained. full_system.launch.py already starts control_stack
    # (which leaves the bridge off by default); including it from there would
    # double-launch every control node, so we only add the bridge itself.
    params_file = os.path.join(bringup_dir, "config", "control_stack.yaml")
    dashboard_bridge = Node(
        package="argos_control",
        executable="argos-dashboard-bridge",
        name="dashboard_bridge_node",
        parameters=[params_file],
        output="screen",
    )

    node_server = ExecuteProcess(
        cmd=["node", "pi_server/server.js"],
        cwd=LaunchConfiguration("dashboard_server_dir"),
        output="screen",
        name="dashboard_node_server",
    )

    return LaunchDescription(
        [
            port_arg,
            dashboard_server_arg,
            rosbridge_launch,
            dashboard_bridge,
            node_server,
        ]
    )
