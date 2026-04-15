"""Publish a repeatable stand-to-crawl profile for the expo demo."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .ros_support import TOPICS, zero_twist


def _twist_from_values(
    *,
    linear_x: float,
    linear_y: float,
    linear_z: float,
    roll: float,
    pitch: float,
    yaw_rate: float,
) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.linear.y = float(linear_y)
    msg.linear.z = float(linear_z)
    msg.angular.x = float(roll)
    msg.angular.y = float(pitch)
    msg.angular.z = float(yaw_rate)
    return msg


class DemoCommanderNode(Node):
    def __init__(self):
        super().__init__("demo_commander_node")

        self.declare_parameter("command_topic", TOPICS.teleop_cmd_vel)
        self.declare_parameter("gait_mode_topic", TOPICS.gait_mode)
        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("startup_delay_s", 3.0)
        self.declare_parameter("stand_duration_s", 3.0)
        self.declare_parameter("crawl_duration_s", 18.0)
        self.declare_parameter("settle_duration_s", 3.0)
        self.declare_parameter("loop_profile", False)
        self.declare_parameter("crawl_mode", "crawl")
        self.declare_parameter("linear_x", 0.08)
        self.declare_parameter("linear_y", 0.0)
        self.declare_parameter("linear_z", 0.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw_rate", 0.18)

        command_topic = self.get_parameter("command_topic").value
        gait_mode_topic = self.get_parameter("gait_mode_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self.startup_delay_s = max(
            0.0, float(self.get_parameter("startup_delay_s").value)
        )
        self.stand_duration_s = max(
            0.0, float(self.get_parameter("stand_duration_s").value)
        )
        self.crawl_duration_s = max(
            0.0, float(self.get_parameter("crawl_duration_s").value)
        )
        self.settle_duration_s = max(
            0.0, float(self.get_parameter("settle_duration_s").value)
        )
        self.loop_profile = bool(self.get_parameter("loop_profile").value)
        self.crawl_mode = str(self.get_parameter("crawl_mode").value).strip().lower()
        if self.crawl_mode not in {"crawl", "trot"}:
            self.get_logger().warning(
                f"Unsupported crawl mode '{self.crawl_mode}', defaulting to crawl."
            )
            self.crawl_mode = "crawl"

        self.crawl_twist = _twist_from_values(
            linear_x=float(self.get_parameter("linear_x").value),
            linear_y=float(self.get_parameter("linear_y").value),
            linear_z=float(self.get_parameter("linear_z").value),
            roll=float(self.get_parameter("roll").value),
            pitch=float(self.get_parameter("pitch").value),
            yaw_rate=float(self.get_parameter("yaw_rate").value),
        )
        self.zero_twist = zero_twist()
        self.estop_active = False
        self.last_phase_name = ""
        self.start_time_ns = self.get_clock().now().nanoseconds

        self.command_pub = self.create_publisher(Twist, command_topic, 10)
        self.gait_mode_pub = self.create_publisher(String, gait_mode_topic, 10)

        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_timer(1.0 / publish_rate_hz, self._update)

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _publish_phase(self, phase_name: str, gait_mode: str, command: Twist) -> None:
        if phase_name != self.last_phase_name:
            self.get_logger().info(f"Demo phase -> {phase_name}")
            self.last_phase_name = phase_name

        gait_msg = String()
        gait_msg.data = gait_mode
        self.gait_mode_pub.publish(gait_msg)
        self.command_pub.publish(command)

    def _phase_at_elapsed(self, elapsed_s: float):
        if self.estop_active:
            return "estop", "stand", self.zero_twist

        cycle_duration_s = (
            self.startup_delay_s
            + self.stand_duration_s
            + self.crawl_duration_s
            + self.settle_duration_s
        )
        if self.loop_profile and cycle_duration_s > 0.0:
            elapsed_s %= cycle_duration_s

        if elapsed_s < self.startup_delay_s:
            return "startup", "crouch", self.zero_twist
        elapsed_s -= self.startup_delay_s

        if elapsed_s < self.stand_duration_s:
            return "stand", "stand", self.zero_twist
        elapsed_s -= self.stand_duration_s

        if elapsed_s < self.crawl_duration_s:
            return "crawl", self.crawl_mode, self.crawl_twist
        elapsed_s -= self.crawl_duration_s

        if elapsed_s < self.settle_duration_s:
            return "settle", "stand", self.zero_twist

        return "hold", "stand", self.zero_twist

    def _update(self) -> None:
        elapsed_s = max(
            0.0, (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
        )
        phase_name, gait_mode, command = self._phase_at_elapsed(elapsed_s)
        self._publish_phase(phase_name, gait_mode, command)


def main(args=None):
    rclpy.init(args=args)
    node = DemoCommanderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
