"""Generate dashboard/tests/fixtures/ik_golden.json from Python Kinematics.

Run from repo root: python dashboard/tests/fixtures/gen_ik_golden.py

The JS golden test (dashboard/tests/kinematics.test.js) reads this file and
asserts the JS port matches within 1e-4 rad per joint. Re-run this script
whenever Config.py leg geometry or Kinematics.py changes.
"""

import itertools
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
REPO = HERE.parents[3]
sys.path.insert(0, str(REPO / "ros2_ws"))

import numpy as np

from argos_control.Config import Configuration
from argos_control.Kinematics import four_legs_inverse_kinematics

# Avoid importing ros_support (pulls in geometry_msgs). Hard-code the order
# here — the robot_config.test.js cross-checks it against the Python source.
LEG_ORDER = ("FR", "FL", "RR", "RL")


def main() -> None:
    cfg = Configuration()
    base = cfg.default_stance.copy()                 # (3, 4) body-frame feet
    base[2, :] = cfg.default_z_ref                   # force z = -0.189

    # Sweep a small body-frame patch around the default stance. Keep offsets
    # conservative: the coupled bell-crank envelope is very sensitive to z,
    # so we hold z fixed at default_z_ref and vary x/y only. This still hits
    # all three joints on all four legs — the rear leg x-shift + hip phi
    # rotation mean every sample produces a distinct (coxa, femur, tibia)
    # triple per leg.
    dx_range = (-0.005, 0.0, 0.005, 0.010, 0.015)
    dy_range = (-0.005, 0.0, 0.005)
    dz_range = (0.0,)

    cases = []
    for (dx, dy, dz) in itertools.product(dx_range, dy_range, dz_range):
        feet = base.copy()
        # Apply the same body-frame delta to every leg — this exercises all
        # four legs at once, which is the only IK path the dashboard uses.
        feet[0, :] += dx
        feet[1, :] += dy
        feet[2, :] += dz
        try:
            angles = four_legs_inverse_kinematics(feet, cfg)
        except ValueError:
            continue
        cases.append({
            "feet_body_m": feet.tolist(),
            "angles_rad": angles.tolist(),           # shape (3, 4)
        })

    out = {
        "leg_order": list(LEG_ORDER),
        "z_ref_m": float(cfg.default_z_ref),
        "cases": cases,
    }
    dest = HERE.parent / "ik_golden.json"
    dest.write_text(json.dumps(out, indent=2))
    print(f"Wrote {len(cases)} cases to {dest}")


if __name__ == "__main__":
    main()
