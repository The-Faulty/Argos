"""Turn body commands into joint targets for crouch, stand, crawl, and trot."""

from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Int32MultiArray, String
from transforms3d.euler import euler2mat

from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics
from .control_core import GaitController, State, StanceController, SwingController
from .ros_support import (
    TOPICS,
    clamp,
    crouch_joint_matrix,
    euler_from_imu,
    int32_multiarray_from_values,
    joint_state_from_matrix,
    matrix_from_pose_array,
    pose_array_from_matrix,
    values_from_int32_multiarray,
    zero_twist,
)


@dataclass
class MotionCommand:
    horizontal_velocity: np.ndarray
    yaw_rate: float
    height: float
    pitch: float
    roll: float


class GaitPlannerNode(Node):
    def __init__(self):
        super().__init__("gait_planner_node")

        self.declare_parameter("command_topic", TOPICS.muxed_cmd_vel)
        self.declare_parameter("gait_mode_topic", TOPICS.gait_mode)
        self.declare_parameter("joint_command_raw_topic", TOPICS.joint_command_raw)
        self.declare_parameter("imu_topic", TOPICS.imu_raw)
        self.declare_parameter("estop_topic", TOPICS.estop)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("default_mode", "crouch")
        self.declare_parameter("max_height_offset_m", 0.08)
        self.declare_parameter("use_imu_stabilization", True)
        self.declare_parameter("imu_filter_alpha", 0.15)
        self.declare_parameter("stabilization_roll_gain", 0.6)
        self.declare_parameter("stabilization_pitch_gain", 0.6)
        self.declare_parameter("stabilization_max_correction_rad", 0.25)
        self.declare_parameter("enable_push_recovery", True)
        self.declare_parameter("push_recovery_tilt_threshold_rad", 0.18)
        self.declare_parameter("push_recovery_hold_s", 1.0)
        self.declare_parameter("recovery_stance_scale_x", 1.05)
        self.declare_parameter("recovery_stance_scale_y", 1.12)
        self.declare_parameter("recovery_height_offset_m", -0.01)
        self.declare_parameter("foothold_candidate_topic", TOPICS.foothold_candidates)
        self.declare_parameter(
            "foothold_contact_modes_topic", TOPICS.foothold_contact_modes
        )
        self.declare_parameter("foothold_adjusted_topic", TOPICS.foothold_adjusted)
        self.declare_parameter("foothold_status_topic", TOPICS.foothold_status)
        self.declare_parameter("enable_foothold_adjustments", True)
        self.declare_parameter("block_on_unsafe_footholds", False)
        self.declare_parameter("foothold_update_timeout_s", 0.4)

        command_topic = self.get_parameter("command_topic").value
        gait_mode_topic = self.get_parameter("gait_mode_topic").value
        joint_command_raw_topic = self.get_parameter("joint_command_raw_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        self.max_height_offset_m = float(
            self.get_parameter("max_height_offset_m").value
        )
        self.use_imu_stabilization = bool(
            self.get_parameter("use_imu_stabilization").value
        )
        self.imu_filter_alpha = clamp(
            float(self.get_parameter("imu_filter_alpha").value), 0.0, 1.0
        )
        self.stabilization_roll_gain = max(
            0.0, float(self.get_parameter("stabilization_roll_gain").value)
        )
        self.stabilization_pitch_gain = max(
            0.0, float(self.get_parameter("stabilization_pitch_gain").value)
        )
        self.stabilization_max_correction_rad = max(
            0.0, float(self.get_parameter("stabilization_max_correction_rad").value)
        )
        self.enable_push_recovery = bool(
            self.get_parameter("enable_push_recovery").value
        )
        self.push_recovery_tilt_threshold_rad = max(
            0.0, float(self.get_parameter("push_recovery_tilt_threshold_rad").value)
        )
        self.push_recovery_hold_ns = int(
            max(0.0, float(self.get_parameter("push_recovery_hold_s").value)) * 1e9
        )
        self.recovery_stance_scale_x = max(
            1.0, float(self.get_parameter("recovery_stance_scale_x").value)
        )
        self.recovery_stance_scale_y = max(
            1.0, float(self.get_parameter("recovery_stance_scale_y").value)
        )
        self.recovery_height_offset_m = float(
            self.get_parameter("recovery_height_offset_m").value
        )
        self.enable_foothold_adjustments = bool(
            self.get_parameter("enable_foothold_adjustments").value
        )
        self.block_on_unsafe_footholds = bool(
            self.get_parameter("block_on_unsafe_footholds").value
        )
        self.foothold_update_timeout_ns = int(
            max(0.0, float(self.get_parameter("foothold_update_timeout_s").value)) * 1e9
        )
        foothold_candidate_topic = self.get_parameter("foothold_candidate_topic").value
        foothold_contact_modes_topic = self.get_parameter(
            "foothold_contact_modes_topic"
        ).value
        foothold_adjusted_topic = self.get_parameter("foothold_adjusted_topic").value
        foothold_status_topic = self.get_parameter("foothold_status_topic").value

        self.config = Configuration()
        self.state = State()
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.current_twist = zero_twist()
        self.current_mode = str(self.get_parameter("default_mode").value).strip().lower()
        self.estop_active = False
        self.imu_euler = np.zeros(3, dtype=float)
        self.imu_received = False
        self.push_recovery_until_ns = 0
        self.push_recovery_active = False
        self.latest_adjusted_footholds: np.ndarray | None = None
        self.latest_foothold_status = np.zeros(4, dtype=int)
        self.latest_foothold_update_ns = 0
        self.foothold_blocked = False

        self.state.foot_locations = (
            self.config.default_stance
            + np.array([0.0, 0.0, self.config.default_z_ref])[:, np.newaxis]
        )

        self.raw_joint_pub = self.create_publisher(
            JointState, joint_command_raw_topic, 10
        )
        self.foothold_candidate_pub = self.create_publisher(
            PoseArray, foothold_candidate_topic, 10
        )
        self.foothold_contact_modes_pub = self.create_publisher(
            Int32MultiArray, foothold_contact_modes_topic, 10
        )
        self.create_subscription(Twist, command_topic, self._command_callback, 10)
        self.create_subscription(String, gait_mode_topic, self._mode_callback, 10)
        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_subscription(Imu, imu_topic, self._imu_callback, 10)
        self.create_subscription(
            PoseArray, foothold_adjusted_topic, self._foothold_adjusted_callback, 10
        )
        self.create_subscription(
            Int32MultiArray, foothold_status_topic, self._foothold_status_callback, 10
        )
        self.create_timer(1.0 / update_rate_hz, self._update)

        self._apply_gait_profile(self.current_mode)

    def _command_callback(self, msg: Twist):
        self.current_twist = msg

    def _mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if not mode:
            return
        if mode not in {"crouch", "stand", "crawl", "trot"}:
            self.get_logger().warning(f"Ignoring unsupported gait mode '{mode}'")
            return
        if mode != self.current_mode:
            self.current_mode = mode
            self.state.ticks = 0
            self._apply_gait_profile(mode)
            self.get_logger().info(f"Switched gait mode to {mode}")

    def _estop_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def _imu_callback(self, msg: Imu):
        sample = np.asarray(euler_from_imu(msg), dtype=float)
        if not self.imu_received:
            self.imu_euler = sample
            self.imu_received = True
            return
        self.imu_euler = (
            (1.0 - self.imu_filter_alpha) * self.imu_euler
            + self.imu_filter_alpha * sample
        )

    def _foothold_adjusted_callback(self, msg: PoseArray):
        try:
            self.latest_adjusted_footholds = matrix_from_pose_array(msg)
            self.latest_foothold_update_ns = self.get_clock().now().nanoseconds
        except ValueError as exc:
            self.get_logger().warning(
                f"Ignoring malformed foothold adjustment message: {exc}"
            )

    def _foothold_status_callback(self, msg: Int32MultiArray):
        try:
            self.latest_foothold_status = values_from_int32_multiarray(
                msg, expected_size=4
            )
            self.latest_foothold_update_ns = self.get_clock().now().nanoseconds
        except ValueError as exc:
            self.get_logger().warning(
                f"Ignoring malformed foothold status message: {exc}"
            )

    def _apply_gait_profile(self, mode: str):
        if mode == "crawl":
            # Keep every phase non-zero so each single-leg swing column is reachable.
            self.config.contact_phases = np.array(
                [
                    [0, 1, 1, 1],
                    [1, 0, 1, 1],
                    [1, 1, 0, 1],
                    [1, 1, 1, 0],
                ]
            )
            self.config.swing_time = 0.25
            self.config.overlap_time = self.config.swing_time
            self.config.z_clearance = 0.04
        elif mode == "trot":
            self.config.contact_phases = np.array(
                [
                    [1, 1, 1, 0],
                    [1, 0, 0, 1],
                    [1, 0, 0, 1],
                    [1, 1, 1, 0],
                ]
            )
            self.config.swing_time = 0.15
            self.config.overlap_time = 0.05
            self.config.z_clearance = 0.05
        else:
            self.config.contact_phases = np.ones((4, 4), dtype=int)
            self.config.swing_time = 0.25
            self.config.overlap_time = 0.0
            self.config.z_clearance = 0.04

    def _build_command(self, push_recovery_active: bool) -> MotionCommand:
        twist = (
            zero_twist()
            if self.estop_active or push_recovery_active
            else self.current_twist
        )
        return MotionCommand(
            horizontal_velocity=np.array(
                [
                    clamp(twist.linear.x, -self.config.max_x_velocity, self.config.max_x_velocity),
                    clamp(twist.linear.y, -self.config.max_y_velocity, self.config.max_y_velocity),
                ],
                dtype=float,
            ),
            yaw_rate=clamp(
                twist.angular.z,
                -self.config.max_yaw_rate,
                self.config.max_yaw_rate,
            ),
            height=self.config.default_z_ref
            + clamp(twist.linear.z, -self.max_height_offset_m, self.max_height_offset_m),
            pitch=clamp(twist.angular.y, -self.config.max_pitch, self.config.max_pitch),
            roll=clamp(twist.angular.x, -self.config.max_pitch, self.config.max_pitch),
        )

    def _update_push_recovery(self) -> bool:
        if not (self.enable_push_recovery and self.imu_received):
            self.push_recovery_active = False
            return False

        now_ns = self.get_clock().now().nanoseconds
        roll, pitch, _ = self.imu_euler
        tilt_exceeded = (
            abs(roll) >= self.push_recovery_tilt_threshold_rad
            or abs(pitch) >= self.push_recovery_tilt_threshold_rad
        )
        if tilt_exceeded:
            self.push_recovery_until_ns = now_ns + self.push_recovery_hold_ns

        active = now_ns < self.push_recovery_until_ns
        if active and not self.push_recovery_active:
            self.get_logger().warning(
                "Push recovery active: holding a widened stand until tilt settles."
            )
        elif self.push_recovery_active and not active:
            self.get_logger().info("Push recovery cleared; resuming requested gait.")
        self.push_recovery_active = active
        return active

    def _imu_stabilisation_matrix(self) -> np.ndarray:
        if not (self.use_imu_stabilization and self.imu_received):
            return np.eye(3, dtype=float)

        roll, pitch, _ = self.imu_euler
        correction = euler2mat(
            clamp(
                -roll * self.stabilization_roll_gain,
                -self.stabilization_max_correction_rad,
                self.stabilization_max_correction_rad,
            ),
            clamp(
                -pitch * self.stabilization_pitch_gain,
                -self.stabilization_max_correction_rad,
                self.stabilization_max_correction_rad,
            ),
            0.0,
        )
        return correction.T

    def _foot_transform_matrix(self, command: MotionCommand) -> np.ndarray:
        return self._imu_stabilisation_matrix() @ euler2mat(
            command.roll, command.pitch, 0.0
        )

    def _apply_foot_transform(
        self, foot_locations: np.ndarray, command: MotionCommand
    ) -> np.ndarray:
        return self._foot_transform_matrix(command) @ foot_locations

    def _stabilise_with_imu(self, foot_locations: np.ndarray) -> np.ndarray:
        return self._imu_stabilisation_matrix() @ foot_locations

    def _stand_foot_locations(
        self, height: float, push_recovery_active: bool
    ) -> np.ndarray:
        stance = self.config.default_stance.copy()
        if push_recovery_active:
            stance[0, :] *= self.recovery_stance_scale_x
            stance[1, :] *= self.recovery_stance_scale_y
            height += self.recovery_height_offset_m
        return stance + np.array([0.0, 0.0, height])[:, np.newaxis]

    def _nominal_touchdown_location(
        self, leg_index: int, command: MotionCommand
    ) -> np.ndarray:
        touchdown = self.swing_controller.raibert_touchdown_location(
            leg_index, command
        ).copy()
        touchdown[2] = command.height
        return touchdown

    def _candidate_touchdown_locations(
        self, command: MotionCommand, contact_modes: np.ndarray
    ) -> np.ndarray:
        candidates = self.state.foot_locations.copy()
        for leg_index in range(4):
            if contact_modes[leg_index] == 0:
                candidates[:, leg_index] = self._nominal_touchdown_location(
                    leg_index, command
                )
        return candidates

    def _publish_foothold_candidates(
        self,
        candidate_planned: np.ndarray,
        contact_modes: np.ndarray,
        foot_transform: np.ndarray,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        candidate_base = foot_transform @ candidate_planned
        self.foothold_candidate_pub.publish(
            pose_array_from_matrix(stamp, candidate_base)
        )
        self.foothold_contact_modes_pub.publish(
            int32_multiarray_from_values(contact_modes)
        )

    def _foothold_data_fresh(self) -> bool:
        if self.latest_adjusted_footholds is None or self.foothold_update_timeout_ns <= 0:
            return False
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self.latest_foothold_update_ns) <= self.foothold_update_timeout_ns

    def _set_foothold_blocked(self, blocked: bool) -> None:
        if blocked and not self.foothold_blocked:
            self.get_logger().warning(
                "Foothold checker rejected the active touchdown; holding gait phase."
            )
        elif self.foothold_blocked and not blocked:
            self.get_logger().info("Foothold checker cleared; resuming gait.")
        self.foothold_blocked = blocked

    def _resolved_touchdown(
        self,
        leg_index: int,
        nominal_touchdown: np.ndarray,
        foot_transform: np.ndarray,
    ) -> np.ndarray:
        if not (self.enable_foothold_adjustments and self._foothold_data_fresh()):
            return nominal_touchdown
        if self.latest_adjusted_footholds is None:
            return nominal_touchdown
        if self.latest_foothold_status[leg_index] <= 0:
            return nominal_touchdown

        adjusted_base = self.latest_adjusted_footholds[:, leg_index]
        return foot_transform.T @ adjusted_base

    def _should_block_unsafe_footholds(self, contact_modes: np.ndarray) -> bool:
        if not (self.block_on_unsafe_footholds and self._foothold_data_fresh()):
            return False
        swing_mask = np.asarray(contact_modes, dtype=int) == 0
        return bool(np.any(swing_mask) and np.any(self.latest_foothold_status[swing_mask] < 0))

    def _step_gait(
        self,
        command: MotionCommand,
        contact_modes: np.ndarray,
        foot_transform: np.ndarray,
    ) -> np.ndarray:
        new_foot_locations = np.zeros((3, 4), dtype=float)

        for leg_index in range(4):
            if contact_modes[leg_index] == 1:
                new_location = self.stance_controller.next_foot_location(
                    leg_index, self.state, command
                )
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(self.state.ticks)
                    / max(1, self.config.swing_ticks)
                )
                nominal_touchdown = self._nominal_touchdown_location(
                    leg_index, command
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    self.state,
                    command,
                    touchdown_override=self._resolved_touchdown(
                        leg_index, nominal_touchdown, foot_transform
                    ),
                )
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations

    def _update(self):
        push_recovery_active = self._update_push_recovery()
        command = self._build_command(push_recovery_active)
        foothold_blocked = False
        contact_modes = np.ones(4, dtype=int)
        foot_transform = self._foot_transform_matrix(command)

        try:
            if self.current_mode == "crouch":
                angles = crouch_joint_matrix(self.config)
                # Keep the nominal stand footprint in state so the next standing
                # or walking mode starts from a known body-frame reference.
                planned = self._stand_foot_locations(
                    self.config.default_z_ref,
                    push_recovery_active=False,
                )
                contact_modes = np.ones(4, dtype=int)
                candidate_planned = planned.copy()
                rotated = planned.copy()
            else:
                if self.current_mode == "stand" or self.estop_active or push_recovery_active:
                    planned = self._stand_foot_locations(
                        command.height,
                        push_recovery_active=push_recovery_active,
                    )
                    contact_modes = np.ones(4, dtype=int)
                    candidate_planned = planned.copy()
                else:
                    contact_modes = self.gait_controller.contacts(self.state.ticks)
                    candidate_planned = self._candidate_touchdown_locations(
                        command, contact_modes
                    )
                    foothold_blocked = self._should_block_unsafe_footholds(
                        contact_modes
                    )
                    if foothold_blocked:
                        planned = self.state.foot_locations.copy()
                    else:
                        planned = self._step_gait(
                            command, contact_modes, foot_transform
                        )

                rotated = foot_transform @ planned
                angles = four_legs_inverse_kinematics(rotated, self.config)
        except ValueError as exc:
            self.get_logger().warning(f"Skipping gait update: {exc}")
            return

        self._set_foothold_blocked(foothold_blocked)
        self._publish_foothold_candidates(candidate_planned, contact_modes, foot_transform)
        self.state.foot_locations = planned
        self.state.rotated_foot_locations = rotated
        self.state.joint_angles = angles
        self.state.height = command.height
        self.state.roll = command.roll
        self.state.pitch = command.pitch
        self.state.yaw_rate = command.yaw_rate
        self.state.horizontal_velocity = command.horizontal_velocity
        if not foothold_blocked:
            self.state.ticks += 1

        msg = joint_state_from_matrix(self.get_clock().now().to_msg(), angles)
        self.raw_joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
