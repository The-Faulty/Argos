"""Launch rosbridge_websocket on port 9090 for the Argos dashboard.

The Node.js dashboard server (dashboard/pi_server/server.js) proxies browser
traffic into this websocket, which gives the browser normal ROS pub/sub.
Nothing here runs inside the browser — rosbridge_suite does the translation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="Port for the rosbridge_websocket server.",
    )
    address_arg = DeclareLaunchArgument(
        "address",
        default_value="0.0.0.0",
        description="Bind address for the rosbridge_websocket server.",
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        parameters=[
            {
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "address": LaunchConfiguration("address"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            port_arg,
            address_arg,
            rosbridge_node,
        ]
    )
