"""ROS topic names plus small helpers shared by all control nodes."""

from dataclasses import dataclass
from math import atan2, sqrt

import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from transforms3d.euler import quat2euler

from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics


# Leg and joint ordering used throughout the stack.
# Every (3, 4) angle matrix follows FR / FL / RR / RL left-to-right.
LEG_ORDER = ("FR", "FL", "RR", "RL")
JOINT_ROWS = ("coxa", "femur", "tibia")
JOINT_NAMES = [f"{leg}_{joint}_joint" for leg in LEG_ORDER for joint in JOINT_ROWS]

# Hardware-level constants — ROS_DOMAIN_ID must match on every device in the network.
ROS_DOMAIN_ID = 42
TTY_LIDAR = "/dev/ttyLIDAR"
TTY_ESP32 = "/dev/ttyESP32"


@dataclass(frozen=True)
class Topics:
    """All ROS topic name strings in one place.

    Import TOPICS from here instead of hardcoding strings in each node —
    that way renaming a topic only requires changing one line.
    """
    teleop_cmd_vel: str = "/teleop/cmd_vel"   # joystick input
    nav_cmd_vel: str = "/nav/cmd_vel"          # autonomous nav input
    muxed_cmd_vel: str = "/cmd_vel"            # selected output of command_mux
    command_source: str = "/command_source"    # which input won (teleop/nav/idle)
    gait_mode: str = "/gait_mode"              # crouch / stand / crawl / trot
    estop: str = "/estop"                      # Bool — True = freeze everything
    joint_command_raw: str = "/joint_command/raw"
    joint_command_safe: str = "/joint_command/safe"
    joint_command: str = "/joint_command"
    joint_states: str = "/joint_states"
    imu_raw: str = "/imu/data_raw"
    gas: str = "/gas"


TOPICS = Topics()


def clamp(value, low, high):
    """Clamp value to [low, high]. Used everywhere to keep commands in safe ranges."""
    return max(low, min(high, value))


def zero_twist() -> Twist:
    """Return a Twist with all fields zero — used as a safe idle command."""
    return Twist()


def copy_twist(msg: Twist) -> Twist:
    """Copy a Twist message field-by-field so later mutations don't affect the stored version."""
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = msg.linear.y
    out.linear.z = msg.linear.z
    out.angular.x = msg.angular.x
    out.angular.y = msg.angular.y
    out.angular.z = msg.angular.z
    return out


def default_foot_locations(config: Configuration) -> np.ndarray:
    """Default stance footprint at the configured standing height. Returns (3, 4) array."""
    return config.default_stance + np.array([0.0, 0.0, config.default_z_ref])[:, np.newaxis]


def crouch_joint_matrix(_config: Configuration) -> np.ndarray:
    """All joints at zero — that's the physical crouch position for this robot."""
    return np.zeros((3, 4), dtype=float)


def stand_joint_matrix(config: Configuration) -> np.ndarray:
    """IK solution for the default standing pose. Used when initializing the safety node."""
    return four_legs_inverse_kinematics(default_foot_locations(config), config)


def matrix_to_ordered_positions(angle_matrix: np.ndarray) -> np.ndarray:
    """Flatten a (3, 4) joint matrix to a 1-D array matching JOINT_NAMES order."""
    positions = []
    for leg_index in range(4):
        for row in range(3):
            positions.append(float(angle_matrix[row, leg_index]))
    return np.asarray(positions, dtype=float)


def ordered_positions_to_matrix(positions) -> np.ndarray:
    """Reshape a flat 1-D positions array back into a (3, 4) angle matrix."""
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
    """Build a JointState message from a flat positions array."""
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.name = list(names or JOINT_NAMES)
    msg.position = [float(value) for value in positions]
    return msg


def joint_state_from_matrix(stamp, angle_matrix: np.ndarray, frame_id="base_link") -> JointState:
    """Build a JointState message from a (3, 4) angle matrix."""
    return joint_state_from_positions(
        stamp,
        matrix_to_ordered_positions(angle_matrix),
        frame_id=frame_id,
    )


def positions_from_joint_state(msg: JointState) -> np.ndarray:
    """Pull a flat positions array from a JointState, re-ordering to match JOINT_NAMES if needed."""
    if len(msg.position) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint positions, got {len(msg.position)}"
        )

    if not msg.name:
        # No names provided — assume the order already matches JOINT_NAMES
        return np.asarray(msg.position, dtype=float)

    # Re-order by name in case the publisher uses a different joint ordering
    index_by_name = {name: idx for idx, name in enumerate(msg.name)}
    return np.asarray([msg.position[index_by_name[name]] for name in JOINT_NAMES], dtype=float)


def joint_limit_vectors(config: Configuration) -> tuple[np.ndarray, np.ndarray]:
    """Return (min_limits, max_limits) as flat 1-D vectors matching JOINT_NAMES order."""
    limits = config.joint_limits_per_leg_rad
    mins = []
    maxs = []
    for leg_index in range(4):
        for row in range(3):
            mins.append(float(limits[row, leg_index, 0]))
            maxs.append(float(limits[row, leg_index, 1]))
    return np.asarray(mins, dtype=float), np.asarray(maxs, dtype=float)


def euler_from_imu(msg) -> tuple[float, float, float]:
    """Extract (roll, pitch, yaw) in radians from an IMU message.

    Uses the quaternion when available. Falls back to accelerometer-only
    tilt estimation if the orientation field is not populated.
    """
    q = msg.orientation
    norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        # Quaternion not populated — estimate roll/pitch from gravity vector only
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
