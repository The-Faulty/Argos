"""Publish simple odometry from the muxed body command for demo mapping."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from .ros_support import TOPICS, copy_twist, zero_twist


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class CommandOdometryNode(Node):
    def __init__(self):
        super().__init__("command_odometry_node")

        self.declare_parameter("command_topic", TOPICS.muxed_cmd_vel)
        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("command_timeout_s", 0.30)

        command_topic = self.get_parameter("command_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        odometry_topic = self.get_parameter("odometry_topic").value
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.command_timeout_ns = int(
            max(0.0, float(self.get_parameter("command_timeout_s").value)) * 1e9
        )

        self.latest_command = zero_twist()
        self.last_command_time = None
        self.last_update_ns = self.get_clock().now().nanoseconds
        self.estop_active = False
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0

        self.odometry_pub = self.create_publisher(Odometry, odometry_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Twist, command_topic, self._command_callback, 10)
        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_timer(1.0 / publish_rate_hz, self._update)

    def _command_callback(self, msg: Twist):
        self.latest_command = copy_twist(msg)
        self.last_command_time = self.get_clock().now()

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _command_is_fresh(self) -> bool:
        if self.last_command_time is None:
            return False
        age_ns = (self.get_clock().now() - self.last_command_time).nanoseconds
        return age_ns <= self.command_timeout_ns

    def _active_command(self) -> Twist:
        if self.estop_active or not self._command_is_fresh():
            return zero_twist()
        return self.latest_command

    def _integrate_pose(self, dt_s: float, command: Twist) -> None:
        vx_body = float(command.linear.x)
        vy_body = float(command.linear.y)
        yaw_rate = float(command.angular.z)

        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        self.x_m += (vx_body * cos_yaw - vy_body * sin_yaw) * dt_s
        self.y_m += (vx_body * sin_yaw + vy_body * cos_yaw) * dt_s
        self.yaw_rad = _wrap_angle(self.yaw_rad + yaw_rate * dt_s)

    def _publish_odometry(self, stamp, command: Twist) -> None:
        qx, qy, qz, qw = _yaw_to_quaternion(self.yaw_rad)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x_m
        odom.pose.pose.position.y = self.y_m
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(command.linear.x)
        odom.twist.twist.linear.y = float(command.linear.y)
        odom.twist.twist.linear.z = float(command.linear.z)
        odom.twist.twist.angular.x = float(command.angular.x)
        odom.twist.twist.angular.y = float(command.angular.y)
        odom.twist.twist.angular.z = float(command.angular.z)
        self.odometry_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x_m
        transform.transform.translation.y = self.y_m
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def _update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        dt_s = max(0.0, (now_ns - self.last_update_ns) / 1e9)
        self.last_update_ns = now_ns

        command = self._active_command()
        self._integrate_pose(dt_s, command)
        self._publish_odometry(self.get_clock().now().to_msg(), command)


def main(args=None):
    rclpy.init(args=args)
    node = CommandOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
