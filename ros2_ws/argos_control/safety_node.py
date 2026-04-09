"""Safety filter for Argos joint commands."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .Config import Configuration
from .ros_contract import TOPICS
from .ros_helpers import (
    joint_limit_vectors,
    joint_state_from_positions,
    positions_from_joint_state,
    stand_joint_matrix,
    matrix_to_ordered_positions,
)


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        self.declare_parameter("raw_command_topic", TOPICS.joint_command_raw)
        self.declare_parameter("safe_command_topic", TOPICS.joint_command_safe)
        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter("max_joint_velocity_rad_s", 6.0)
        self.declare_parameter("raw_command_timeout_s", 0.25)

        raw_command_topic = self.get_parameter("raw_command_topic").value
        safe_command_topic = self.get_parameter("safe_command_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.max_joint_velocity_rad_s = float(
            self.get_parameter("max_joint_velocity_rad_s").value
        )
        self.raw_command_timeout_ns = int(
            float(self.get_parameter("raw_command_timeout_s").value) * 1e9
        )

        self.config = Configuration()
        self.min_limits, self.max_limits = joint_limit_vectors(self.config)
        self.stand_positions = matrix_to_ordered_positions(stand_joint_matrix(self.config))

        self.current_positions = self.stand_positions.copy()
        self.target_positions = self.stand_positions.copy()
        self.last_raw_time = None
        self.estop_active = False
        self.update_period_s = 1.0 / update_rate_hz

        self.safe_pub = self.create_publisher(JointState, safe_command_topic, 10)
        self.create_subscription(
            JointState, raw_command_topic, self._raw_command_callback, 10
        )
        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_timer(self.update_period_s, self._update)

    def _raw_command_callback(self, msg: JointState):
        try:
            positions = positions_from_joint_state(msg)
        except (KeyError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring malformed joint command: {exc}")
            return

        self.target_positions = np.clip(positions, self.min_limits, self.max_limits)
        self.last_raw_time = self.get_clock().now()

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _target_is_stale(self) -> bool:
        if self.last_raw_time is None:
            return True
        age_ns = (self.get_clock().now() - self.last_raw_time).nanoseconds
        return age_ns > self.raw_command_timeout_ns

    def _update(self):
        if self.estop_active or self._target_is_stale():
            desired = self.stand_positions
        else:
            desired = self.target_positions

        max_step = self.max_joint_velocity_rad_s * self.update_period_s
        delta = np.clip(desired - self.current_positions, -max_step, max_step)
        self.current_positions = np.clip(
            self.current_positions + delta,
            self.min_limits,
            self.max_limits,
        )

        msg = joint_state_from_positions(
            self.get_clock().now().to_msg(),
            self.current_positions,
        )
        self.safe_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
