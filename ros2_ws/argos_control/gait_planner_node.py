"""Turn body commands into joint targets for crouch, stand, crawl, and trot."""

from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from transforms3d.euler import euler2mat

from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics
from .control_core import GaitController, State, StanceController, SwingController
from .ros_support import (
    TOPICS,
    clamp,
    crouch_joint_matrix,
    euler_from_imu,
    joint_state_from_matrix,
    zero_twist,
)


@dataclass
class MotionCommand:
    """One sanitized motion request for a single control tick."""
    horizontal_velocity: np.ndarray  # [vx, vy] in m/s
    yaw_rate: float                  # rad/s
    height: float                    # absolute body height in meters (negative = below hip)
    pitch: float                     # body pitch in radians
    roll: float                      # body roll in radians


class GaitPlannerNode(Node):
    """Converts cmd_vel Twist messages into raw joint angles for the safety node.

    Data flow: cmd_vel -> build_command -> step_gait / stand -> IK -> raw joint publish
    Modes: crouch, stand, crawl (one foot), trot (diagonal pairs)
    """

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

        command_topic = self.get_parameter("command_topic").value
        gait_mode_topic = self.get_parameter("gait_mode_topic").value
        joint_command_raw_topic = self.get_parameter("joint_command_raw_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        self.max_height_offset_m = float(self.get_parameter("max_height_offset_m").value)
        self.use_imu_stabilization = bool(self.get_parameter("use_imu_stabilization").value)
        # Clamp filter alpha to [0, 1] so bad param values don't blow up the filter
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
        self.enable_push_recovery = bool(self.get_parameter("enable_push_recovery").value)
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

        # Seed foot locations to the default stand height so first IK solve is valid
        self.state.foot_locations = (
            self.config.default_stance
            + np.array([0.0, 0.0, self.config.default_z_ref])[:, np.newaxis]
        )

        self.raw_joint_pub = self.create_publisher(JointState, joint_command_raw_topic, 10)
        self.create_subscription(Twist, command_topic, self._command_callback, 10)
        self.create_subscription(String, gait_mode_topic, self._mode_callback, 10)
        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_subscription(Imu, imu_topic, self._imu_callback, 10)
        self.create_timer(1.0 / update_rate_hz, self._update)

        self._apply_gait_profile(self.current_mode)

    def _command_callback(self, msg: Twist):
        # Just store it — the timer loop reads it every tick
        self.current_twist = msg

    def _mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if not mode:
            return
        if mode not in {"crouch", "stand", "crawl", "trot"}:
            self.get_logger().warning(f"Ignoring unsupported gait mode '{mode}'")
            return
        if mode == self.current_mode:
            return

        prev_mode = self.current_mode
        self.current_mode = mode
        self._apply_gait_profile(mode)

        # Only reset the tick counter when starting a walking gait from a static pose.
        # If we're already walking (crawl ↔ trot), leave ticks running so legs in
        # mid-swing don't freeze or jump.
        static_modes = {"crouch", "stand"}
        if prev_mode in static_modes and mode not in static_modes:
            self.state.ticks = 0

        self.get_logger().info(f"Switched gait mode: {prev_mode} -> {mode}")

    def _estop_callback(self, msg: Bool):
        # Immediately freeze motion when e-stop fires
        self.estop_active = bool(msg.data)

    def _imu_callback(self, msg: Imu):
        sample = np.asarray(euler_from_imu(msg), dtype=float)
        if not self.imu_received:
            # Seed the filter with the first sample instead of starting at zero
            self.imu_euler = sample
            self.imu_received = True
            return
        # Low-pass filter — alpha controls how fast the estimate tracks new readings
        self.imu_euler = (
            (1.0 - self.imu_filter_alpha) * self.imu_euler
            + self.imu_filter_alpha * sample
        )

    def _apply_gait_profile(self, mode: str):
        """Set contact phases, timing, and foot clearance for the requested gait."""
        if mode == "crawl":
            # Crawl: one foot off at a time — slowest but most stable on rough ground
            self.config.contact_phases = np.array(
                [[0, 1, 1, 1], [1, 0, 1, 1], [1, 1, 0, 1], [1, 1, 1, 0]]
            )
            self.config.swing_time = 0.25
            self.config.overlap_time = 0.0
            self.config.z_clearance = 0.04
        elif mode == "trot":
            # Trot: diagonal pairs (FR+RL, FL+RR) move together — faster, less stable
            self.config.contact_phases = np.array(
                [[1, 1, 1, 0], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 0]]
            )
            self.config.swing_time = 0.15
            self.config.overlap_time = 0.05
            self.config.z_clearance = 0.05
        else:
            # Stand / crouch: all feet on the ground, no swing needed
            self.config.contact_phases = np.ones((4, 4), dtype=int)
            self.config.swing_time = 0.25
            self.config.overlap_time = 0.0
            self.config.z_clearance = 0.04

    def _build_command(self, push_recovery_active: bool) -> MotionCommand:
        """Build a sanitized MotionCommand from the latest incoming Twist."""
        # Zero velocity during e-stop or push recovery — let the balance logic take over
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
            yaw_rate=clamp(twist.angular.z, -self.config.max_yaw_rate, self.config.max_yaw_rate),
            height=self.config.default_z_ref
            + clamp(twist.linear.z, -self.max_height_offset_m, self.max_height_offset_m),
            pitch=clamp(twist.angular.y, -self.config.max_pitch, self.config.max_pitch),
            roll=clamp(twist.angular.x, -self.config.max_pitch, self.config.max_pitch),
        )

    def _update_push_recovery(self) -> bool:
        """Check body tilt and hold a wide stance if the robot was pushed. Returns True while active."""
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
            # Keep pushing the hold window forward as long as tilt is still detected
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

    def _stabilise_with_imu(self, foot_locations: np.ndarray) -> np.ndarray:
        """Rotate foot targets to counteract measured body tilt.

        Multiplies tilt by the gain then clamps so one bad IMU reading can't
        flip the robot over.
        """
        if not self.imu_received:
            return foot_locations

        roll, pitch, _ = self.imu_euler
        # Negate the tilt so the correction pushes back against the lean
        rmat = euler2mat(
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
        return rmat.T @ foot_locations

    def _stand_foot_locations(self, height: float, push_recovery_active: bool) -> np.ndarray:
        """Build a foot position matrix for standing, widened if push recovery is active."""
        stance = self.config.default_stance.copy()
        if push_recovery_active:
            # Widen and lower slightly — gives a more stable base when the robot gets bumped
            stance[0, :] *= self.recovery_stance_scale_x
            stance[1, :] *= self.recovery_stance_scale_y
            height += self.recovery_height_offset_m
        return stance + np.array([0.0, 0.0, height])[:, np.newaxis]

    def _step_gait(self, command: MotionCommand) -> np.ndarray:
        """Advance all four legs by one tick and return updated foot positions."""
        contact_modes = self.gait_controller.contacts(self.state.ticks)
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
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, self.state, command
                )
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations

    def _update(self):
        """Main 50 Hz control loop: plan foot positions, run IK, and publish joint angles."""
        push_recovery_active = self._update_push_recovery()
        command = self._build_command(push_recovery_active)

        try:
            if self.current_mode == "crouch":
                # Crouch uses zero angles directly — no IK needed
                angles = crouch_joint_matrix(self.config)
                # Keep foot locations at default stand height so the next mode
                # transition has a valid starting pose for the swing controller
                planned = self._stand_foot_locations(
                    self.config.default_z_ref, push_recovery_active=False
                )
                rotated = planned.copy()
            else:
                if self.current_mode == "stand" or self.estop_active or push_recovery_active:
                    planned = self._stand_foot_locations(
                        command.height, push_recovery_active=push_recovery_active
                    )
                else:
                    # Active gait — advance each leg one tick
                    planned = self._step_gait(command)

                # Apply operator body tilt (roll/pitch joystick) then IMU correction
                rotated = euler2mat(command.roll, command.pitch, 0.0) @ planned
                if self.use_imu_stabilization:
                    rotated = self._stabilise_with_imu(rotated)
                angles = four_legs_inverse_kinematics(rotated, self.config)

        except ValueError as exc:
            # IK workspace miss — skip this tick rather than crashing the node
            self.get_logger().warning(f"Skipping gait update: {exc}")
            return

        self.state.foot_locations = planned
        self.state.rotated_foot_locations = rotated
        self.state.joint_angles = angles
        self.state.height = command.height
        self.state.roll = command.roll
        self.state.pitch = command.pitch
        self.state.yaw_rate = command.yaw_rate
        self.state.horizontal_velocity = command.horizontal_velocity
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
