"""Argos control package.

Keep top-level imports lazy so utility modules such as Config/Kinematics can be
imported in tests without pulling in the full ROS and visualization stack.
"""

from importlib import import_module

__all__ = [
    "BehaviorState",
    "GaitController",
    "JOINT_NAMES",
    "State",
    "StanceController",
    "SwingController",
    "TOPICS",
]


def __getattr__(name):
    if name in {
        "BehaviorState",
        "GaitController",
        "State",
        "StanceController",
        "SwingController",
    }:
        module = import_module(".control_core", __name__)
        return getattr(module, name)

    if name in {"JOINT_NAMES", "TOPICS"}:
        module = import_module(".ros_support", __name__)
        return getattr(module, name)

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
