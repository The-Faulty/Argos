"""Shared ROS topic and naming contract for the Argos stack."""

from dataclasses import dataclass


LEG_ORDER = ("FR", "FL", "RR", "RL")
JOINT_ROWS = ("coxa", "femur", "tibia")
JOINT_NAMES = [
    f"{leg}_{joint}_joint"
    for leg in LEG_ORDER
    for joint in JOINT_ROWS
]

ROS_DOMAIN_ID = 42
TTY_LIDAR = "/dev/ttyLIDAR"
TTY_TEENSY = "/dev/ttyTEENSY"


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


TOPICS = Topics()

