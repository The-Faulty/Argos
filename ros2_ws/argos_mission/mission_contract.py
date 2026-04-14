"""Shared topic names and mission states for the Argos mission stack."""

from dataclasses import dataclass
from enum import Enum


class MissionState(str, Enum):
    """All possible mission states.

    Inherits from str so comparisons like state == "IDLE" still work.
    """
    IDLE = "IDLE"
    EXPLORE = "EXPLORE"
    DETECT = "DETECT"
    REPORT = "REPORT"
    CONTINUE = "CONTINUE"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class MissionTopics:
    estop: str = "/estop"
    gait_mode: str = "/gait_mode"
    gas: str = "/gas"
    odometry: str = "/odometry/filtered"
    thermal_image: str = "/thermal/image_raw"
    thermal_camera_info: str = "/thermal/camera_info"
    victim_detections: str = "/victim_detections"
    hazard_map: str = "/hazard_map"
    mission_state: str = "/mission/state"
    mission_detection_count: str = "/mission/detection_count"


TOPICS = MissionTopics()
