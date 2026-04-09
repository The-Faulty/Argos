"""ROS 2 gait planner that turns body commands into joint targets."""

from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from transforms3d.euler import euler2mat

from .Config import Configuration
from .Gaits import GaitController
from .Kinematics import four_legs_inverse_kinematics
from .State import State
from .StanceController import StanceController
from .SwingLegController import SwingController
from .ros_contract import TOPICS
from .ros_helpers import clamp, euler_from_imu, joint_state_from_matrix, zero_twist


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
        self.declare_parameter("default_mode", "stand")
        self.declare_parameter("max_height_offset_m", 0.08)
        self.declare_parameter("use_imu_stabilization", True)

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

        self.config = Configuration()
        self.state = State()
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.current_twist = zero_twist()
        self.current_mode = str(self.get_parameter("default_mode").value).strip().lower()
        self.estop_active = False
        self.imu_euler = (0.0, 0.0, 0.0)

        self.state.foot_locations = (
            self.config.default_stance
            + np.array([0.0, 0.0, self.config.default_z_ref])[:, np.newaxis]
        )

        self.raw_joint_pub = self.create_publisher(
            JointState, joint_command_raw_topic, 10
        )
        self.create_subscription(Twist, command_topic, self._command_callback, 10)
        self.create_subscription(String, gait_mode_topic, self._mode_callback, 10)
        self.create_subscription(Bool, estop_topic, self._estop_callback, 10)
        self.create_subscription(Imu, imu_topic, self._imu_callback, 10)
        self.create_timer(1.0 / update_rate_hz, self._update)

        self._apply_gait_profile(self.current_mode)

    def _command_callback(self, msg: Twist):
        self.current_twist = msg

    def _mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if not mode:
            return
        if mode not in {"stand", "crawl", "trot"}:
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
        self.imu_euler = euler_from_imu(msg)

    def _apply_gait_profile(self, mode: str):
        if mode == "crawl":
            self.config.contact_phases = np.array(
                [
                    [0, 1, 1, 1],
                    [1, 0, 1, 1],
                    [1, 1, 0, 1],
                    [1, 1, 1, 0],
                ]
            )
            self.config.swing_time = 0.25
            self.config.overlap_time = 0.0
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

    def _build_command(self) -> MotionCommand:
        twist = zero_twist() if self.estop_active else self.current_twist
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

    def _stabilise_with_imu(self, foot_locations: np.ndarray) -> np.ndarray:
        _, pitch, roll = self.imu_euler
        correction_factor = 0.5
        max_tilt = 0.4
        rmat = euler2mat(
            correction_factor * clamp(-roll, -max_tilt, max_tilt),
            correction_factor * clamp(-pitch, -max_tilt, max_tilt),
            0.0,
        )
        return rmat.T @ foot_locations

    def _step_gait(self, command: MotionCommand) -> np.ndarray:
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
        command = self._build_command()

        try:
            if self.current_mode == "stand" or self.estop_active:
                planned = (
                    self.config.default_stance
                    + np.array([0.0, 0.0, command.height])[:, np.newaxis]
                )
            else:
                planned = self._step_gait(command)

            rotated = euler2mat(command.roll, command.pitch, 0.0) @ planned
            if self.use_imu_stabilization:
                rotated = self._stabilise_with_imu(rotated)

            angles = four_legs_inverse_kinematics(rotated, self.config)
        except ValueError as exc:
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
