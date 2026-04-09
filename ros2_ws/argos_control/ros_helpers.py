"""Helpers shared by the ROS 2 Argos control nodes."""

from math import sqrt

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from transforms3d.euler import quat2euler

from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics
from .ros_contract import JOINT_NAMES


def clamp(value, low, high):
    return max(low, min(high, value))


def zero_twist():
    return Twist()


def copy_twist(msg: Twist) -> Twist:
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.linear.z = msg.linear.z
    out.angular.x = msg.angular.x
    out.angular.y = msg.angular.y
    out.angular.z = msg.angular.z
    return out


def default_foot_locations(config: Configuration) -> np.ndarray:
    return config.default_stance + np.array([0.0, 0.0, config.default_z_ref])[:, np.newaxis]


def stand_joint_matrix(config: Configuration) -> np.ndarray:
    return four_legs_inverse_kinematics(default_foot_locations(config), config)


def matrix_to_ordered_positions(angle_matrix: np.ndarray) -> np.ndarray:
    positions = []
    for leg_index in range(4):
        positions.extend(
            float(angle_matrix[row, leg_index])
            for row in range(3)
        )
    return np.asarray(positions, dtype=float)


def ordered_positions_to_matrix(positions) -> np.ndarray:
    arr = np.asarray(list(positions), dtype=float)
    if arr.size != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint positions, got {arr.size}"
        )

    angle_matrix = np.zeros((3, 4), dtype=float)
    idx = 0
    for leg_index in range(4):
        for row in range(3):
            angle_matrix[row, leg_index] = arr[idx]
            idx += 1
    return angle_matrix


def joint_state_from_positions(stamp, positions, names=None, frame_id="base_link") -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.name = list(names or JOINT_NAMES)
    msg.position = [float(value) for value in positions]
    return msg


def joint_state_from_matrix(stamp, angle_matrix: np.ndarray, frame_id="base_link") -> JointState:
    return joint_state_from_positions(
        stamp,
        matrix_to_ordered_positions(angle_matrix),
        frame_id=frame_id,
    )


def positions_from_joint_state(msg: JointState) -> np.ndarray:
    if len(msg.position) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint positions, got {len(msg.position)}"
        )

    if not msg.name:
        return np.asarray(msg.position, dtype=float)

    index_by_name = {name: idx for idx, name in enumerate(msg.name)}
    return np.asarray(
        [msg.position[index_by_name[name]] for name in JOINT_NAMES],
        dtype=float,
    )


def joint_limit_vectors(config: Configuration):
    limits = config.joint_limits_per_leg_rad
    mins = []
    maxs = []
    for leg_index in range(4):
        for row in range(3):
            mins.append(float(limits[row, leg_index, 0]))
            maxs.append(float(limits[row, leg_index, 1]))
    return np.asarray(mins, dtype=float), np.asarray(maxs, dtype=float)


def euler_from_imu(msg) -> tuple[float, float, float]:
    q = msg.orientation
    norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    return quat2euler((q.w / norm, q.x / norm, q.y / norm, q.z / norm))

