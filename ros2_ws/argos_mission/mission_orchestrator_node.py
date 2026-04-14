"""Simple top-level mission state machine for bench and demo workflows."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from .mission_contract import MissionState, TOPICS


# Gait to use for each mission state — stand still while detecting/reporting,
# crawl during exploration phases.
GAIT_BY_STATE = {
    MissionState.IDLE:     "stand",
    MissionState.EXPLORE:  "crawl",
    MissionState.DETECT:   "stand",
    MissionState.REPORT:   "stand",
    MissionState.CONTINUE: "crawl",
    MissionState.ESTOP:    "stand",
}


class MissionOrchestratorNode(Node):
    """Runs the mission state machine and tells the control stack what gait to use."""

    def __init__(self):
        super().__init__("mission_orchestrator_node")

        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("gait_mode_topic", TOPICS.gait_mode)
        self.declare_parameter("state_topic", TOPICS.mission_state)
        self.declare_parameter("detection_count_topic", TOPICS.mission_detection_count)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("autostart_explore", True)
        # How long to stay in each transient state before advancing
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

        self.current_state = MissionState.IDLE
        self.state_enter_ns = self.get_clock().now().nanoseconds
        self.estop_active = False
        self.detection_count = 0

        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.gait_mode_pub = self.create_publisher(String, gait_mode_topic, 10)

        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_subscription(Int32, detection_count_topic, self._detection_callback, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._update)

        # Kick the control stack into the right gait for the starting state
        self._publish_gait_for_state(self.current_state)

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _detection_callback(self, msg: Int32):
        # Clamp at zero so a buggy publisher can't go negative
        self.detection_count = max(0, int(msg.data))

    def _elapsed_s(self) -> float:
        """Seconds since the current state was entered."""
        return max(0.0, (self.get_clock().now().nanoseconds - self.state_enter_ns) / 1e9)

    def _publish_gait_for_state(self, state: MissionState):
        """Tell the control stack which gait to use for this state."""
        gait = String()
        gait.data = GAIT_BY_STATE[state]
        self.gait_mode_pub.publish(gait)

    def _set_state(self, new_state: MissionState):
        """Transition to a new mission state and reset the state timer."""
        if new_state == self.current_state:
            return

        self.current_state = new_state
        self.state_enter_ns = self.get_clock().now().nanoseconds
        self._publish_gait_for_state(new_state)
        self.get_logger().info(f"Mission state -> {new_state.value}")

    def _next_state(self) -> Optional[MissionState]:
        """Return the next state to transition to, or None to stay put."""
        # E-stop always wins
        if self.estop_active:
            return MissionState.ESTOP

        # Clear e-stop by resuming exploration
        if self.current_state == MissionState.ESTOP:
            return MissionState.EXPLORE if self.autostart_explore else MissionState.IDLE

        if self.current_state == MissionState.IDLE:
            return MissionState.EXPLORE if self.autostart_explore else None

        # Any detection during exploration triggers the detect phase
        if self.current_state in {MissionState.EXPLORE, MissionState.CONTINUE}:
            if self.detection_count > 0:
                return MissionState.DETECT

        # Timed transitions for the detect → report → continue → explore loop
        elapsed_s = self._elapsed_s()
        if self.current_state == MissionState.DETECT and elapsed_s >= self.detection_hold_s:
            return MissionState.REPORT
        if self.current_state == MissionState.REPORT and elapsed_s >= self.report_hold_s:
            return MissionState.CONTINUE
        if self.current_state == MissionState.CONTINUE and elapsed_s >= self.continue_hold_s:
            return MissionState.EXPLORE

        return None

    def _publish_status(self):
        """Publish the current state string so other nodes can react."""
        state_msg = String()
        state_msg.data = self.current_state.value
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
