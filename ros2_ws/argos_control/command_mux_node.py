"""Pick the freshest motion command from teleop, nav, or e-stop."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .ros_support import TOPICS, copy_twist, zero_twist


class CommandMuxNode(Node):
    def __init__(self):
        super().__init__("command_mux_node")

        self.declare_parameter("teleop_topic", TOPICS.teleop_cmd_vel)
        self.declare_parameter("nav_topic", TOPICS.nav_cmd_vel)
        self.declare_parameter("output_topic", TOPICS.muxed_cmd_vel)
        self.declare_parameter("source_topic", TOPICS.command_source)
        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("teleop_timeout_s", 0.25)
        self.declare_parameter("nav_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 50.0)

        teleop_topic = self.get_parameter("teleop_topic").value
        nav_topic = self.get_parameter("nav_topic").value
        output_topic = self.get_parameter("output_topic").value
        source_topic = self.get_parameter("source_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.teleop_timeout_ns = int(
            float(self.get_parameter("teleop_timeout_s").value) * 1e9
        )
        self.nav_timeout_ns = int(
            float(self.get_parameter("nav_timeout_s").value) * 1e9
        )

        self.teleop_msg = zero_twist()
        self.nav_msg = zero_twist()
        self.teleop_time = None
        self.nav_time = None
        self.estop_active = False

        self.output_pub = self.create_publisher(Twist, output_topic, 10)
        self.source_pub = self.create_publisher(String, source_topic, 10)

        self.create_subscription(Twist, teleop_topic, self._teleop_callback, 10)
        self.create_subscription(Twist, nav_topic, self._nav_callback, 10)
        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_timer(1.0 / publish_rate_hz, self._publish_muxed_command)

    def _teleop_callback(self, msg: Twist):
        self.teleop_msg = copy_twist(msg)
        self.teleop_time = self.get_clock().now()

    def _nav_callback(self, msg: Twist):
        self.nav_msg = copy_twist(msg)
        self.nav_time = self.get_clock().now()

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _is_fresh(self, stamp, timeout_ns: int) -> bool:
        if stamp is None:
            return False
        age_ns = (self.get_clock().now() - stamp).nanoseconds
        return age_ns <= timeout_ns

    def _publish_muxed_command(self):
        source = "idle"
        selected = zero_twist()

        if self.estop_active:
            source = "estop"
        elif self._is_fresh(self.teleop_time, self.teleop_timeout_ns):
            selected = copy_twist(self.teleop_msg)
            source = "teleop"
        elif self._is_fresh(self.nav_time, self.nav_timeout_ns):
            selected = copy_twist(self.nav_msg)
            source = "nav"

        self.output_pub.publish(selected)

        source_msg = String()
        source_msg.data = source
        self.source_pub.publish(source_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
