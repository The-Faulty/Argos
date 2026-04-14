"""Republish the latest safe joint target at a fixed rate."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .Config import Configuration
from .ros_support import (
    TOPICS,
    crouch_joint_matrix,
    joint_state_from_positions,
    matrix_to_ordered_positions,
    positions_from_joint_state,
)


class JointCommandPublisherNode(Node):
    """Buffers the latest safe joint command and re-publishes it at a fixed 100 Hz.

    Decouples the safety node's update rate from the MCU's expected packet rate.
    Also mirrors commands to /joint_states for RViz preview.
    """

    def __init__(self):
        super().__init__("joint_command_publisher_node")

        self.declare_parameter("safe_command_topic", TOPICS.joint_command_safe)
        self.declare_parameter("joint_command_topic", TOPICS.joint_command)
        self.declare_parameter("joint_states_topic", TOPICS.joint_states)
        self.declare_parameter("output_rate_hz", 100.0)
        self.declare_parameter("publish_joint_states_preview", True)

        safe_command_topic = self.get_parameter("safe_command_topic").value
        joint_command_topic = self.get_parameter("joint_command_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        output_rate_hz = float(self.get_parameter("output_rate_hz").value)
        self.publish_joint_states_preview = bool(
            self.get_parameter("publish_joint_states_preview").value
        )

        # Default to crouch so the robot holds still until the safety node sends something
        config = Configuration()
        self.latest_positions = matrix_to_ordered_positions(crouch_joint_matrix(config))

        self.command_pub = self.create_publisher(JointState, joint_command_topic, 10)
        self.joint_state_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.create_subscription(JointState, safe_command_topic, self._safe_command_callback, 10)
        self.create_timer(1.0 / output_rate_hz, self._publish)

    def _safe_command_callback(self, msg: JointState):
        try:
            self.latest_positions = positions_from_joint_state(msg)
        except (KeyError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring malformed safe joint command: {exc}")

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        msg = joint_state_from_positions(stamp, self.latest_positions)
        self.command_pub.publish(msg)
        if self.publish_joint_states_preview:
            self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
