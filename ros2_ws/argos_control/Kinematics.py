"""Inverse and forward kinematics for the Argos leg."""

import logging
from math import pi, acos, atan2, sqrt, sin, cos, fmod, asin, radians

import numpy as np

log = logging.getLogger(__name__)
LEG_NAMES = ("FR", "FL", "RR", "RL")


def _point_to_rad(p1, p2):
    """Wrap atan2(p2, p1) into [0, 2*pi]."""
    return (atan2(p2, p1) + 2 * pi) % (2 * pi)


def _rotx(angle):
    """3x3 rotation matrix about the X axis."""
    c, s = cos(angle), sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _circle_intersect(P1, r1, P2, r2):
    """Intersect two circles. Returns (A, B) or (None, None)."""
    dx, dy = P2[0] - P1[0], P2[1] - P1[1]
    d = sqrt(dx * dx + dy * dy)
    if d < 1e-9 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return None, None

    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h = sqrt(max(0.0, r1 * r1 - a * a))
    mx = P1[0] + a * dx / d
    my = P1[1] + a * dy / d
    px, py = -dy / d, dx / d
    return (
        np.array([mx + h * px, my + h * py]),
        np.array([mx - h * px, my - h * py]),
    )


def _hip_abductor_ik(r_body_foot, leg_index, L1_hip, phi):
    """Solve the hip abductor and return (theta_1, x_sag, y_sag)."""
    is_right = leg_index in (0, 2)

    x, y, z = float(r_body_foot[0]), float(r_body_foot[1]), float(r_body_foot[2])
    if is_right:
        y = -y

    # Rotate into the coxa frame before solving the lateral joint.
    R1 = pi / 2 - phi
    rf = _rotx(-R1) @ np.array([x, y, z])
    x, y, z = rf

    len_A = sqrt(y * y + z * z)
    if len_A < 1e-9:
        log.warning("hip_abductor_ik: near-singular lateral projection; clamping")
        len_A = 1e-9

    a_1 = _point_to_rad(y, z)
    asin_arg = sin(phi) * L1_hip / len_A
    asin_arg = max(-1.0, min(1.0, asin_arg))
    a_2 = asin(asin_arg)
    a_3 = pi - a_2 - phi
    theta_1 = a_1 + a_3
    if theta_1 >= 2 * pi:
        theta_1 = fmod(theta_1, 2 * pi)

    # Shift to the femur origin, then project into the sagittal plane.
    offset = np.array([0.0, L1_hip * cos(theta_1), L1_hip * sin(theta_1)])
    translated = np.array([x, y, z]) - offset
    R2 = theta_1 + phi - pi / 2
    sag = _rotx(-R2) @ translated
    x_, _, z_ = sag

    return theta_1, x_, -z_


def _sagittal_ik(x_sag, y_sag, P, _hint=None):
    """Solve the two sagittal joints for one leg.

    Uses a numerical sweep of ``theta_bot`` through the forward-kinematics
    chain to guarantee FK round-trip consistency with the coupled bell-crank
    linkage.  An optional ``_hint`` (previous ``theta_bot``) narrows the
    search to ±12° for warm-started real-time use.
    """
    L1, L2 = P["L1"], P["L2"]

    d = sqrt(x_sag * x_sag + y_sag * y_sag)
    max_r = (L1 + L2) * 0.99
    if d > max_r:
        scale = max_r / d
        x_sag *= scale
        y_sag *= scale
        d = max_r
        log.warning("sagittal_ik: foot clamped to %.4f m", max_r)

    min_r = abs(L1 - L2) * 1.01
    if d < min_r:
        if d < 1e-9:
            x_sag, y_sag, d = 0.0, min_r, min_r
        else:
            scale = min_r / d
            x_sag *= scale
            y_sag *= scale
            d = min_r
        log.warning("sagittal_ik: foot clamped to inner radius %.4f m", min_r)

    # 2-link IK for the upper-leg angle (directly driven by top servo).
    cos_b = max(-1.0, min(1.0, (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)))
    t_upper = atan2(x_sag, y_sag) - acos(cos_b)

    best_bot, best_err = None, float("inf")

    def _sweep(lo_deg, hi_deg, step_deg):
        nonlocal best_bot, best_err
        bd = lo_deg
        while bd <= hi_deg:
            t_bot = bd * pi / 180.0
            fk = leg_fk(t_upper, t_bot, P)
            if fk is not None:
                err = sqrt((fk[0] - x_sag) ** 2 + (fk[1] - y_sag) ** 2)
                if err < best_err:
                    best_err = err
                    best_bot = t_bot
            bd += step_deg

    # Warm-start narrow search when a hint is available.
    if _hint is not None:
        hd = _hint * 180.0 / pi
        _sweep(hd - 12, hd + 12, 0.5)
        if best_bot is not None and best_err < 0.002:
            cd = best_bot * 180.0 / pi
            _sweep(cd - 1, cd + 1, 0.1)
            return t_upper, best_bot

    # Full coarse sweep, then fine refine.
    best_bot, best_err = None, float("inf")
    _sweep(-90, 90, 2)
    if best_bot is None or best_err > 0.01:
        return None
    cd = best_bot * 180.0 / pi
    _sweep(cd - 2, cd + 2, 0.1)
    if best_err > 0.002:
        return None
    return t_upper, best_bot


# Per-leg warm-start cache: stores the last solved theta_bot for each leg index so
# the next solve can start a narrow search instead of a full sweep.
# Not thread-safe, but ROS 2 nodes are single-threaded by default so this is fine.
_ik_hint = {}


def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """Full 3-DOF IK for one leg. Returns None when the solve fails."""
    P = config.leg_params
    theta_1, x_sag, y_sag = _hip_abductor_ik(
        r_body_foot, leg_index, P["L1_hip"], P["phi"]
    )

    sag = _sagittal_ik(x_sag, y_sag, P, _hint=_ik_hint.get(leg_index))
    if sag is None:
        log.warning(
            "leg_ik: sagittal IK failed for %s at x=%.4f m, y=%.4f m",
            LEG_NAMES[leg_index],
            x_sag,
            y_sag,
        )
        return None

    theta_top, theta_bot = sag
    _ik_hint[leg_index] = theta_bot
    return np.array([theta_1, theta_top, theta_bot])


def four_legs_inverse_kinematics(r_body_foot, config):
    """Solve IK for all four legs."""
    angles = np.zeros((3, 4))
    for i in range(4):
        r_leg = r_body_foot[:, i] - config.LEG_ORIGINS[:, i]
        leg_angles = leg_explicit_inverse_kinematics(r_leg, i, config)
        if leg_angles is None:
            raise ValueError(
                f"{LEG_NAMES[i]} target is outside the IK workspace "
                f"(x={r_leg[0]:+.4f}, y={r_leg[1]:+.4f}, z={r_leg[2]:+.4f} m)"
            )
        angles[:, i] = leg_angles
    return angles


def leg_fk(theta_top, theta_bot, params):
    """Sagittal FK for one leg. Returns foot (x_sag, y_sag) in meters, or None."""
    L1, L2 = params["L1"], params["L2"]
    O = np.zeros(2)

    knee = np.array([L1 * sin(theta_top), L1 * cos(theta_top)])
    horn = np.array([params["rh"] * sin(theta_bot), params["rh"] * cos(theta_bot)])

    A, B = _circle_intersect(O, params["rbr"], horn, params["Lr1"])
    if A is None:
        return None

    def _bcl_x(c):
        return sin(atan2(c[0], c[1]) + params["abc"])

    bc_right = A if _bcl_x(A) < _bcl_x(B) else B

    phi_r = atan2(bc_right[0], bc_right[1])
    phi_l = phi_r + params["abc"]
    bc_left = np.array([params["rbl"] * sin(phi_l), params["rbl"] * cos(phi_l)])

    A, B = _circle_intersect(knee, params["dko"], bc_left, params["Lr2"])
    if A is None:
        return None

    def _diff(c):
        ld = knee - c
        d = (atan2(ld[0], ld[1]) - theta_top) % (2 * pi)
        return d - 2 * pi if d > pi else d

    dA, dB = _diff(A), _diff(B)
    if 0 < dA < pi:
        rod2 = A
    elif 0 < dB < pi:
        rod2 = B
    else:
        rod2 = A if abs(dA - pi / 2) < abs(dB - pi / 2) else B

    ld = (knee - rod2) / np.linalg.norm(knee - rod2)
    foot = knee + L2 * ld
    return float(foot[0]), float(foot[1])


if __name__ == "__main__":

    class _Cfg:
        # Real Argos geometry (must match Config.py)
        L1_HIP = 0.035563
        PHI = radians(20.156)
        L_UPPER = 0.127063
        L_LOWER = 0.127183
        R_BC_LEFT = 0.04
        R_BC_RIGHT = 0.04
        ALPHA_BC = radians(90.0)
        L_ROD1 = 0.03
        L_ROD2 = 0.150
        R_HORN = 0.024
        D_KNEE_OFFSET = 0.030023
        LEG_FB = 0.11165
        LEG_LR = 0.061
        LEG_ORIGINS = np.array([
            [ LEG_FB,  LEG_FB, -LEG_FB, -LEG_FB],
            [-LEG_LR,  LEG_LR, -LEG_LR,  LEG_LR],
            [0, 0, 0, 0],
        ])

        @property
        def leg_params(self):
            return dict(
                L1_hip=self.L1_HIP,
                phi=self.PHI,
                L1=self.L_UPPER,
                L2=self.L_LOWER,
                rbl=self.R_BC_LEFT,
                rbr=self.R_BC_RIGHT,
                abc=self.ALPHA_BC,
                Lr1=self.L_ROD1,
                Lr2=self.L_ROD2,
                rh=self.R_HORN,
                dko=self.D_KNEE_OFFSET,
            )

    cfg = _Cfg()
    P = cfg.leg_params

    print("=" * 66)
    print("  Argos 3-DOF Kinematics self-test")
    print("  theta_1 = hip abductor | theta_top = upper leg | theta_bot = bell-crank")
    print("=" * 66)

    print("\n  Sagittal IK/FK round-trip:")
    sag_tests = [
        (0.000, 0.200, "Nominal"),
        (0.040, 0.190, "Swing fwd +40mm"),
        (0.080, 0.170, "Wide fwd"),
        (-0.040, 0.190, "Swing back -40mm"),
        (-0.080, 0.160, "Wide back"),
        (0.000, 0.170, "Raised"),
        (0.060, 0.200, "Fwd reach"),
        (-0.060, 0.200, "Back reach"),
    ]
    print(f"  {'Pose':>22}   {'theta_top':>10}  {'theta_bot':>10}  FK err")
    print("  " + "-" * 60)
    all_ok = True
    for x, y, name in sag_tests:
        sol = _sagittal_ik(x, y, P)
        if sol is None:
            print(f"  {name:<22}  UNREACHABLE")
            all_ok = False
            continue
        fk = leg_fk(sol[0], sol[1], P)
        if fk is None:
            print(f"  {name:<22}  FK failed")
            all_ok = False
            continue
        err = sqrt((fk[0] - x) ** 2 + (fk[1] - y) ** 2)
        ok = "OK" if err < 0.001 else f"{err * 1000:.4f} mm"
        if err >= 0.001:
            all_ok = False
        print(
            f"  {name:<22}   {np.degrees(sol[0]):+9.3f}°"
            f"  {np.degrees(sol[1]):+9.3f}°  {ok}"
        )
    print(f"\n  Sagittal: {'All passed' if all_ok else 'Failures found'}")

    # Use the real default stance from Config.py (delta_x=0.117, delta_y=0.1106)
    # NOTE: valid stance z range with real geometry is approx -0.185 to -0.105 m.
    # Config.py default_z_ref=-0.25 is OUTSIDE the reachable workspace.
    Z_TEST = -0.165
    print(f"\n  Full 3-DOF IK - default stance z={Z_TEST*1000:.0f} mm:")
    delta_x, delta_y = 0.117, 0.1106
    front_x, rear_x = 0.00, -0.04
    stance = np.array([
        [ delta_x + front_x,  delta_x + front_x, -delta_x + rear_x, -delta_x + rear_x],
        [-delta_y, delta_y, -delta_y, delta_y],
        [Z_TEST, Z_TEST, Z_TEST, Z_TEST],
    ])
    names = ["FR", "FL", "RR", "RL"]
    print(f"  {'Leg':>3}  {'theta_1':>10}  {'theta_top':>10}  {'theta_bot':>10}")
    print("  " + "-" * 48)
    for i, name in enumerate(names):
        r = stance[:, i] - cfg.LEG_ORIGINS[:, i]
        sol = leg_explicit_inverse_kinematics(r, i, cfg)
        if sol is None:
            print(f"  {name}   UNREACHABLE")
            continue
        print(
            f"  {name}   {np.degrees(sol[0]):+9.2f}°"
            f"  {np.degrees(sol[1]):+10.2f}°  {np.degrees(sol[2]):+10.2f}°"
        )

    print("\n  four_legs_inverse_kinematics output shape:", end=" ")
    out = four_legs_inverse_kinematics(stance, cfg)
    print(f"{out.shape}  (expected (3, 4))")
    print(f"  Row 0 theta_1   (deg): {np.degrees(out[0]).round(2)}")
    print(f"  Row 1 theta_top (deg): {np.degrees(out[1]).round(2)}")
    print(f"  Row 2 theta_bot (deg): {np.degrees(out[2]).round(2)}")