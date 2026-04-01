"""
Controller.py  —  Argos SAR Quadruped Controller
=================================================
Adapted from Dingo Controller.py.

Changes from Dingo
------------------
1.  inverse_kinematics returns a (3, 4) matrix.
    Row 0 = theta_1    (hip abductor)
    Row 1 = theta_top  (upper leg, top servo)
    Row 2 = theta_bot  (lower leg, bottom servo via bell crank)

2.  publish_joint_space_command publishes all three joints per leg.

Everything else (gait FSM, stance/swing scheduling, IMU stabilisation)
is unchanged from Dingo.
"""

from Argos.ros2_ws.argos_control.Gaits import GaitController
from Argos.ros2_ws.argos_control.StanceController import StanceController
from Argos.ros2_ws.argos_control.SwingLegController import SwingController
from argos_utilities.Utilities import clipped_first_order_filter
from Argos.ros2_ws.argos_control.State import BehaviorState, State

from dingo_control.msg import TaskSpace, JointSpace, Angle

import numpy as np
from transforms3d.euler import euler2mat
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from math import degrees


class Controller:
    """Controller and planner for the Argos quadruped."""

    def __init__(self, config, inverse_kinematics):
        self.config = config
        self.inverse_kinematics = inverse_kinematics   # four_legs_inverse_kinematics

        self.task_space_pub  = rospy.Publisher(
            'task_space_goals',  TaskSpace,  queue_size=10)
        self.joint_space_pub = rospy.Publisher(
            'joint_space_goals', JointSpace, queue_size=10)

        self.smoothed_yaw = 0.0

        self.contact_modes    = np.zeros(4)
        self.gait_controller  = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller= StanceController(self.config)

        # State-machine transition tables (unchanged from Dingo)
        self.hop_transition_mapping = {
            BehaviorState.REST:       BehaviorState.HOP,
            BehaviorState.HOP:        BehaviorState.FINISHHOP,
            BehaviorState.FINISHHOP:  BehaviorState.REST,
            BehaviorState.TROT:       BehaviorState.HOP,
        }
        self.trot_transition_mapping = {
            BehaviorState.REST:       BehaviorState.TROT,
            BehaviorState.TROT:       BehaviorState.REST,
            BehaviorState.HOP:        BehaviorState.TROT,
            BehaviorState.FINISHHOP:  BehaviorState.TROT,
        }
        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST,
            BehaviorState.REST:        BehaviorState.DEACTIVATED,
        }

    # ── Gait step (unchanged from Dingo) ─────────────────────────────────
    def step_gait(self, state, command):
        contact_modes      = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode  = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(
                    leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks)
                    / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command)
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes

    # ── Task-space publisher (unchanged from Dingo) ───────────────────────
    def publish_task_space_command(self, rotated_foot_locations):
        msg = TaskSpace()
        for attr, i in [('FR_foot',0),('FL_foot',1),('RR_foot',2),('RL_foot',3)]:
            p = Point(
                rotated_foot_locations[0,i] - self.config.LEG_ORIGINS[0,i],
                rotated_foot_locations[1,i] - self.config.LEG_ORIGINS[1,i],
                rotated_foot_locations[2,i] - self.config.LEG_ORIGINS[2,i],
            )
            setattr(msg, attr, p)
        msg.header = Header(stamp=rospy.Time.now())
        self.task_space_pub.publish(msg)

    # ── Joint-space publisher (3-DOF per leg) ─────────────────────────────
    def _clamp_joint_angles(self, angle_matrix: np.ndarray) -> np.ndarray:
        """Clamp [theta_1, theta_top, theta_bot] rows using configured limits.

        Supported formats:
            (3, 4, 2): per-leg [min, max] for each joint row.
            (3, 2): global [min, max] shared across all legs.
        """
        limits = getattr(self.config, 'joint_limits_per_leg_rad', None)
        if limits is None:
            limits = getattr(self.config, 'JOINT_LIMITS_PER_LEG_RAD', None)

        if limits is not None:
            arr = np.asarray(limits, dtype=float)
            if arr.shape == (3, 4, 2):
                mins = arr[:, :, 0]
                maxs = arr[:, :, 1]
                return np.clip(angle_matrix, mins, maxs)

        limits = getattr(self.config, 'joint_limits_rad', None)
        if limits is None:
            limits = getattr(self.config, 'JOINT_LIMITS_RAD', None)
        if limits is None:
            return angle_matrix

        arr = np.asarray(limits, dtype=float)
        if arr.shape != (3, 2):
            return angle_matrix

        mins = arr[:, 0][:, np.newaxis]
        maxs = arr[:, 1][:, np.newaxis]
        return np.clip(angle_matrix, mins, maxs)

    def publish_joint_space_command(self, angle_matrix: np.ndarray):
        """
        Publish the (3, 4) joint-angle matrix.

        angle_matrix[0, i] = theta_1    for leg i  (radians)
        angle_matrix[1, i] = theta_top  for leg i  (radians)
        angle_matrix[2, i] = theta_bot  for leg i  (radians)
        """
        clamped = self._clamp_joint_angles(angle_matrix)

        msg = JointSpace()
        for attr, i in [('FR_foot',0),('FL_foot',1),('RR_foot',2),('RL_foot',3)]:
            a = Angle(
                degrees(clamped[0, i]),   # theta_1
                degrees(clamped[1, i]),   # theta_top
                degrees(clamped[2, i]),   # theta_bot
            )
            setattr(msg, attr, a)
        msg.header = Header(stamp=rospy.Time.now())
        self.joint_space_pub.publish(msg)

    # ── IMU stabilisation (unchanged from Dingo) ──────────────────────────
    def stabilise_with_IMU(self, foot_locations, orientation):
        yaw, pitch, roll = orientation
        correction_factor = 0.5
        max_tilt          = 0.4
        rmat = euler2mat(
            correction_factor * np.clip(-roll,  -max_tilt, max_tilt),
            correction_factor * np.clip(-pitch, -max_tilt, max_tilt),
            0,
        )
        return rmat.T @ foot_locations

    # ── Main control loop (unchanged from Dingo) ──────────────────────────
    def run(self, state, command):
        previous_state = state.behavior_state

        if command.joystick_control_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.hop_event:
            state.behavior_state = self.hop_transition_mapping[state.behavior_state]

        if previous_state != state.behavior_state:
            rospy.loginfo("State: %s → %s", previous_state, state.behavior_state)

        if state.behavior_state == BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(state, command)

            rotated = euler2mat(command.roll, command.pitch, 0.0) @ state.foot_locations

            yaw, pitch, roll = state.euler_orientation
            correction_factor = 0.8;  max_tilt = 0.4
            rmat = euler2mat(
                correction_factor * np.clip(roll,  -max_tilt, max_tilt),
                correction_factor * np.clip(pitch, -max_tilt, max_tilt),
                0,
            )
            rotated = rmat.T @ rotated

            state.joint_angles           = self.inverse_kinematics(rotated, self.config)
            state.rotated_foot_locations  = rotated
            self.publish_task_space_command(rotated)
            self.publish_joint_space_command(state.joint_angles)

        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )
            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )
            rotated = (
                euler2mat(command.roll, command.pitch, self.smoothed_yaw)
                @ state.foot_locations
            )
            rotated = self.stabilise_with_IMU(rotated, state.euler_orientation)

            state.joint_angles           = self.inverse_kinematics(rotated, self.config)
            state.rotated_foot_locations  = rotated
            self.publish_task_space_command(rotated)
            self.publish_joint_space_command(state.joint_angles)

        state.ticks  += 1
        state.pitch   = command.pitch
        state.roll    = command.roll
        state.height  = command.height

    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        state.joint_angles = self.inverse_kinematics(state.foot_locations, self.config)
        return state.joint_angles
