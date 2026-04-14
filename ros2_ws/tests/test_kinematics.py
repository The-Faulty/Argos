"""Pytest wrapper around the Argos kinematics self-tests.

Run from the ros2_ws directory:
    pytest tests/test_kinematics.py -v
"""

import sys
import os
from math import sqrt, radians

import numpy as np
import pytest

# Allow importing argos_control as a package when running from ros2_ws/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from argos_control.Config import Configuration
from argos_control.Kinematics import (
    _sagittal_ik,
    leg_fk,
    leg_explicit_inverse_kinematics,
    four_legs_inverse_kinematics,
)


@pytest.fixture(scope="module")
def cfg():
    """Shared Configuration instance for all tests."""
    return Configuration()


# ── Sagittal IK/FK round-trip ─────────────────────────────────────────────────

SAG_CASES = [
    (0.000, 0.200, "Nominal"),
    (0.040, 0.190, "Swing fwd +40mm"),
    (0.080, 0.170, "Wide fwd"),
    (-0.040, 0.190, "Swing back -40mm"),
    (-0.080, 0.160, "Wide back"),
    (0.000, 0.170, "Raised"),
    (0.060, 0.200, "Fwd reach"),
    (-0.060, 0.200, "Back reach"),
]


@pytest.mark.parametrize("x, y, name", SAG_CASES)
def test_sagittal_ik_fk_roundtrip(cfg, x, y, name):
    """IK solution should reproduce the target foot position via FK to within 1 mm."""
    P = cfg.leg_params
    sol = _sagittal_ik(x, y, P)
    assert sol is not None, f"{name}: IK returned None (target unreachable)"

    fk = leg_fk(sol[0], sol[1], P)
    assert fk is not None, f"{name}: FK failed on the IK solution"

    err = sqrt((fk[0] - x) ** 2 + (fk[1] - y) ** 2)
    assert err < 0.001, f"{name}: FK error {err * 1000:.3f} mm exceeds 1 mm tolerance"


# ── Full 3-DOF IK at default stance ──────────────────────────────────────────

LEG_NAMES = ("FR", "FL", "RR", "RL")


@pytest.mark.parametrize("leg_index", range(4))
def test_full_ik_default_stance(cfg, leg_index):
    """All four legs should solve IK for the default stand height."""
    Z_TEST = cfg.default_z_ref
    delta_x, delta_y = cfg.delta_x, cfg.delta_y
    front_x, rear_x = cfg.front_leg_x_shift, cfg.rear_leg_x_shift

    stance = np.array([
        [ delta_x + front_x,  delta_x + front_x, -delta_x + rear_x, -delta_x + rear_x],
        [-delta_y, delta_y, -delta_y, delta_y],
        [Z_TEST, Z_TEST, Z_TEST, Z_TEST],
    ])

    r_leg = stance[:, leg_index] - cfg.LEG_ORIGINS[:, leg_index]
    sol = leg_explicit_inverse_kinematics(r_leg, leg_index, cfg)
    assert sol is not None, f"{LEG_NAMES[leg_index]}: IK failed at default stance z={Z_TEST:.3f} m"
    assert sol.shape == (3,), f"{LEG_NAMES[leg_index]}: expected (3,) angles, got {sol.shape}"


def test_four_legs_ik_output_shape(cfg):
    """four_legs_inverse_kinematics should return a (3, 4) matrix."""
    stance = cfg.default_stance + np.array([0.0, 0.0, cfg.default_z_ref])[:, np.newaxis]
    out = four_legs_inverse_kinematics(stance, cfg)
    assert out.shape == (3, 4), f"Expected shape (3, 4), got {out.shape}"


def test_four_legs_ik_within_joint_limits(cfg):
    """All solved angles should respect the configured joint limits."""
    stance = cfg.default_stance + np.array([0.0, 0.0, cfg.default_z_ref])[:, np.newaxis]
    out = four_legs_inverse_kinematics(stance, cfg)
    limits = cfg.joint_limits_per_leg_rad  # shape (3, 4, 2)

    for leg in range(4):
        for row in range(3):
            lo, hi = limits[row, leg, 0], limits[row, leg, 1]
            angle = out[row, leg]
            assert lo <= angle <= hi, (
                f"{LEG_NAMES[leg]} joint {row}: angle {np.degrees(angle):.1f}° "
                f"outside limits [{np.degrees(lo):.1f}°, {np.degrees(hi):.1f}°]"
            )


def test_ik_raises_on_unreachable(cfg):
    """four_legs_inverse_kinematics should raise ValueError for out-of-workspace targets."""
    # Push all feet 1 m below the body — well outside the workspace
    bad_stance = cfg.default_stance + np.array([0.0, 0.0, -1.0])[:, np.newaxis]
    with pytest.raises(ValueError):
        four_legs_inverse_kinematics(bad_stance, cfg)
