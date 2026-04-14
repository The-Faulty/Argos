"""ROS topic names plus small helpers for the control nodes."""

from dataclasses import dataclass
from math import atan2, sqrt

import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from transforms3d.euler import quat2euler

from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics, lower_leg_angle


LEG_ORDER = ("FR", "FL", "RR", "RL")
JOINT_ROWS = ("coxa", "femur", "tibia")
JOINT_NAMES = [f"{leg}_{joint}_joint" for leg in LEG_ORDER for joint in JOINT_ROWS]

ROS_DOMAIN_ID = 42
TTY_LIDAR = "/dev/ttyLIDAR"
TTY_ESP32 = "/dev/ttyESP32"


@dataclass(frozen=True)
class Topics:
    teleop_cmd_vel: str = "/teleop/cmd_vel"
    nav_cmd_vel: str = "/nav/cmd_vel"
    muxed_cmd_vel: str = "/cmd_vel"
    command_source: str = "/command_source"
    gait_mode: str = "/gait_mode"
    estop: str = "/estop"
    joint_command_raw: str = "/joint_command/raw"
    joint_command_safe: str = "/joint_command/safe"
    joint_command: str = "/joint_command"
    joint_states: str = "/joint_states"
    imu_raw: str = "/imu/data_raw"
    gas: str = "/gas"
    foothold_candidates: str = "/foothold/candidates"
    foothold_contact_modes: str = "/foothold/contact_modes"
    foothold_adjusted: str = "/foothold/adjusted"
    foothold_status: str = "/foothold/status"
    foothold_safe: str = "/foothold/safe"
    foothold_markers: str = "/foothold/markers"


TOPICS = Topics()


def clamp(value, low, high):
    return max(low, min(high, value))


def zero_twist() -> Twist:
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


def crouch_joint_matrix(_config: Configuration) -> np.ndarray:
    return np.zeros((3, 4), dtype=float)


def stand_joint_matrix(config: Configuration) -> np.ndarray:
    return four_legs_inverse_kinematics(default_foot_locations(config), config)


def urdf_joint_matrix(angle_matrix: np.ndarray, config: Configuration) -> np.ndarray:
    """Convert control-space joint angles into the URDF's simple revolute joints."""
    urdf_matrix = np.array(angle_matrix, dtype=float, copy=True)
    params = config.leg_params
    for leg_index in range(4):
        theta_top = float(angle_matrix[1, leg_index])
        theta_bot = float(angle_matrix[2, leg_index])
        lower_angle = lower_leg_angle(theta_top, theta_bot, params)
        if lower_angle is None:
            raise ValueError(
                f"Could not convert tibia preview angle for leg {LEG_ORDER[leg_index]}"
            )
        urdf_matrix[2, leg_index] = lower_angle - theta_top
    return urdf_matrix


def matrix_to_ordered_positions(angle_matrix: np.ndarray) -> np.ndarray:
    positions = []
    for leg_index in range(4):
        for row in range(3):
            positions.append(float(angle_matrix[row, leg_index]))
    return np.asarray(positions, dtype=float)


def ordered_positions_to_matrix(positions) -> np.ndarray:
    values = np.asarray(list(positions), dtype=float)
    if values.size != len(JOINT_NAMES):
        raise ValueError(f"Expected {len(JOINT_NAMES)} joint positions, got {values.size}")

    angle_matrix = np.zeros((3, 4), dtype=float)
    idx = 0
    for leg_index in range(4):
        for row in range(3):
            angle_matrix[row, leg_index] = values[idx]
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


def pose_array_from_matrix(
    stamp,
    position_matrix: np.ndarray,
    frame_id="base_link",
) -> PoseArray:
    matrix = np.asarray(position_matrix, dtype=float)
    if matrix.shape != (3, len(LEG_ORDER)):
        raise ValueError(
            f"Expected a (3, {len(LEG_ORDER)}) position matrix, got {matrix.shape}"
        )

    msg = PoseArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for leg_index in range(len(LEG_ORDER)):
        pose = Pose()
        pose.position.x = float(matrix[0, leg_index])
        pose.position.y = float(matrix[1, leg_index])
        pose.position.z = float(matrix[2, leg_index])
        pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def matrix_from_pose_array(msg: PoseArray) -> np.ndarray:
    if len(msg.poses) != len(LEG_ORDER):
        raise ValueError(f"Expected {len(LEG_ORDER)} poses, got {len(msg.poses)}")

    matrix = np.zeros((3, len(LEG_ORDER)), dtype=float)
    for leg_index, pose in enumerate(msg.poses):
        matrix[0, leg_index] = float(pose.position.x)
        matrix[1, leg_index] = float(pose.position.y)
        matrix[2, leg_index] = float(pose.position.z)
    return matrix


def int32_multiarray_from_values(values) -> Int32MultiArray:
    msg = Int32MultiArray()
    msg.data = [int(value) for value in values]
    return msg


def values_from_int32_multiarray(msg: Int32MultiArray, expected_size=None) -> np.ndarray:
    values = np.asarray(msg.data, dtype=int)
    if expected_size is not None and values.size != expected_size:
        raise ValueError(f"Expected {expected_size} values, got {values.size}")
    return values


def positions_from_joint_state(msg: JointState) -> np.ndarray:
    if len(msg.position) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint positions, got {len(msg.position)}"
        )

    if not msg.name:
        return np.asarray(msg.position, dtype=float)

    index_by_name = {name: idx for idx, name in enumerate(msg.name)}
    return np.asarray([msg.position[index_by_name[name]] for name in JOINT_NAMES], dtype=float)


def joint_limit_vectors(config: Configuration) -> tuple[np.ndarray, np.ndarray]:
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
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        accel_norm = sqrt(ax * ax + ay * ay + az * az)
        if accel_norm < 1e-6:
            return 0.0, 0.0, 0.0

        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm
        roll = atan2(ay, az)
        pitch = atan2(-ax, sqrt(ay * ay + az * az))
        return roll, pitch, 0.0

    return quat2euler((q.w / norm, q.x / norm, q.y / norm, q.z / norm))
