"""Simple top-level mission state machine for bench and demo workflows."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from .mission_contract import MISSION_STATES, TOPICS


GAIT_BY_STATE = {
    "IDLE": "stand",
    "EXPLORE": "crawl",
    "DETECT": "stand",
    "REPORT": "stand",
    "CONTINUE": "crawl",
    "ESTOP": "stand",
}


class MissionOrchestratorNode(Node):
    def __init__(self):
        super().__init__("mission_orchestrator_node")

        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("gait_mode_topic", TOPICS.gait_mode)
        self.declare_parameter("state_topic", TOPICS.mission_state)
        self.declare_parameter("detection_count_topic", TOPICS.mission_detection_count)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("autostart_explore", True)
        self.declare_parameter("detection_hold_s", 2.0)
        self.declare_parameter("report_hold_s", 3.0)
        self.declare_parameter("continue_hold_s", 1.0)

        estop_topic = self.get_parameter("estop_topic").value
        gait_mode_topic = self.get_parameter("gait_mode_topic").value
        state_topic = self.get_parameter("state_topic").value
        detection_count_topic = self.get_parameter("detection_count_topic").value

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.autostart_explore = bool(self.get_parameter("autostart_explore").value)
        self.detection_hold_s = float(self.get_parameter("detection_hold_s").value)
        self.report_hold_s = float(self.get_parameter("report_hold_s").value)
        self.continue_hold_s = float(self.get_parameter("continue_hold_s").value)

        self.current_state = "IDLE"
        self.state_enter_ns = self.get_clock().now().nanoseconds
        self.estop_active = False
        self.detection_count = 0

        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.gait_mode_pub = self.create_publisher(String, gait_mode_topic, 10)

        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_subscription(Int32, detection_count_topic, self._detection_callback, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._update)

        self._publish_gait_for_state(self.current_state)

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _detection_callback(self, msg: Int32):
        self.detection_count = max(0, int(msg.data))

    def _elapsed_s(self) -> float:
        return max(0.0, (self.get_clock().now().nanoseconds - self.state_enter_ns) / 1e9)

    def _publish_gait_for_state(self, state: str):
        gait = String()
        gait.data = GAIT_BY_STATE[state]
        self.gait_mode_pub.publish(gait)

    def _set_state(self, new_state: str):
        if new_state not in MISSION_STATES:
            self.get_logger().warning(f"Ignoring invalid mission state '{new_state}'")
            return
        if new_state == self.current_state:
            return

        self.current_state = new_state
        self.state_enter_ns = self.get_clock().now().nanoseconds
        self._publish_gait_for_state(new_state)
        self.get_logger().info(f"Mission state -> {new_state}")

    def _next_state(self) -> Optional[str]:
        if self.estop_active:
            return "ESTOP"

        if self.current_state == "ESTOP":
            return "EXPLORE" if self.autostart_explore else "IDLE"

        if self.current_state == "IDLE":
            return "EXPLORE" if self.autostart_explore else None

        if self.current_state in {"EXPLORE", "CONTINUE"} and self.detection_count > 0:
            return "DETECT"

        elapsed_s = self._elapsed_s()
        if self.current_state == "DETECT" and elapsed_s >= self.detection_hold_s:
            return "REPORT"
        if self.current_state == "REPORT" and elapsed_s >= self.report_hold_s:
            return "CONTINUE"
        if self.current_state == "CONTINUE" and elapsed_s >= self.continue_hold_s:
            return "EXPLORE"
        return None

    def _publish_status(self):
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

    def _update(self):
        next_state = self._next_state()
        if next_state is not None:
            self._set_state(next_state)
        self._publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
