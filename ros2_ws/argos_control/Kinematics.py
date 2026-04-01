"""
Kinematics.py  —  Argos 3-DOF Inverse/Forward Kinematics
=========================================================
3 DOF per leg → (3, 4) joint-angle matrix:
    Row 0:  theta_1    hip abductor  (lateral,  from Dingo)
    Row 1:  theta_top  upper leg     (sagittal, top servo direct drive)
    Row 2:  theta_bot  lower leg     (sagittal, bottom servo via bell crank)

All geometry comes from config.leg_params — do not hardcode values here.

Hip abductor (theta_1) — Dingo method
--------------------------------------
Solves the lateral DOF first, then projects the foot position onto the
sagittal plane for the coupled 2-DOF bell-crank IK.

Sagittal IK — 5-step coupled bell-crank
-----------------------------------------
(See argos_coupled_ik.py or single_leg_test.py for full documentation.)

Sagittal coordinate mapping from 3-D body frame
-------------------------------------------------
After the hip abductor solve the foot position in the sagittal plane is:
    x_sag =  x_   (forward  — same sign as Dingo x)
    y_sag = -z_   (downward — negate Dingo z because Dingo z is UPWARD)

Known FK bugs from earlier versions — both fixed here
------------------------------------------------------
1. _circle_intersect used wrong perpendicular vector:
       WRONG  A = (mx − h·py,  my + h·px)   (projects along connecting line)
       RIGHT  A = (mx + h·px,  my + h·py)   (projects perpendicular to it)

2. FK rod2 branch selection was unreliable across all foot positions.
   Fixed by: pick rod2 where  0 < (θ_lower − θ_top) < π
   (selects the elbow-back assembly mode consistently).
"""

import numpy as np
from math import pi, acos, atan2, sqrt, sin, cos, fmod, asin, radians
import logging

log = logging.getLogger(__name__)

# Last valid [theta_1, theta_top, theta_bot] per leg index.
_LAST_VALID_LEG_ANGLES = {}


# ══════════════════════════════════════════════════════════════════════════════
#  UTILITIES
# ══════════════════════════════════════════════════════════════════════════════

def _point_to_rad(p1, p2):
    """atan2(p2, p1) wrapped to [0, 2π]."""
    return (atan2(p2, p1) + 2*pi) % (2*pi)


def _rotx(angle):
    """3×3 rotation matrix about the X axis."""
    c, s = cos(angle), sin(angle)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]])


def _circle_intersect(P1, r1, P2, r2):
    """
    Intersect two circles.  Returns (A, B) or (None, None).

    Invariants: |A − P1| = r1,  |A − P2| = r2  (same for B).

    The perpendicular unit vector to P1→P2 is (px, py) = (−dy/d, dx/d).
    A = Pmid + h·(px, py),  B = Pmid − h·(px, py).
    Note: the previous bug was writing (mx−h·py, my+h·px) which projects
    ALONG the connecting line, not perpendicular to it.
    """
    dx, dy = P2[0]-P1[0], P2[1]-P1[1]
    d = sqrt(dx*dx + dy*dy)
    if d < 1e-9 or d > r1+r2+1e-9 or d < abs(r1-r2)-1e-9:
        return None, None
    a  = (r1*r1 - r2*r2 + d*d) / (2.0*d)
    h  = sqrt(max(0.0, r1*r1 - a*a))
    mx = P1[0] + a*dx/d;  my = P1[1] + a*dy/d
    px, py = -dy/d, dx/d          # perpendicular unit vector
    return (np.array([mx+h*px, my+h*py]),
            np.array([mx-h*px, my-h*py]))


# ══════════════════════════════════════════════════════════════════════════════
#  HIP ABDUCTOR (theta_1)  —  ported from Dingo
# ══════════════════════════════════════════════════════════════════════════════

def _hip_abductor_ik(r_body_foot, leg_index, L1_hip, phi):
    """
    Solve theta_1 (hip abductor) and return (theta_1, x_sag, y_sag).

    Parameters
    ----------
    r_body_foot : (3,) ndarray  metres, body frame (x=fwd, y=lat-L+, z=up)
    leg_index   : int           0=FR, 1=FL, 2=RR, 3=RL
    L1_hip      : float         coxa link length (m)
    phi         : float         coxa angle from horizontal (rad)

    Returns
    -------
    theta_1 : float  hip abductor angle (rad)
    x_sag   : float  forward component in sagittal plane (m)
    y_sag   : float  downward component in sagittal plane (m)
    """
    is_right = 1 if leg_index in (0, 2) else 0   # FR=0, FL=1, RR=2, RL=3

    x, y, z = float(r_body_foot[0]), float(r_body_foot[1]), float(r_body_foot[2])
    if is_right:
        y = -y

    # Rotate frame to align with coxa link direction
    R1 = pi/2 - phi
    rf = _rotx(-R1) @ np.array([x, y, z])
    x, y, z = rf

    len_A = sqrt(y*y + z*z)
    if len_A < 1e-9:
        log.warning("hip_abductor_ik: near-singular lateral projection; clamping")
        len_A = 1e-9

    a_1   = _point_to_rad(y, z)
    asin_arg = sin(phi) * L1_hip / len_A
    asin_arg = max(-1.0, min(1.0, asin_arg))
    a_2   = asin(asin_arg)
    a_3   = pi - a_2 - phi
    theta_1 = a_1 + a_3
    if theta_1 >= 2*pi:
        theta_1 = fmod(theta_1, 2*pi)

    # Remove hip offset to get foot in femur-origin frame
    offset     = np.array([0.0, L1_hip*cos(theta_1), L1_hip*sin(theta_1)])
    translated = np.array([x, y, z]) - offset

    # Rotate to sagittal plane (x-z plane after this rotation)
    R2  = theta_1 + phi - pi/2
    sag = _rotx(-R2) @ translated
    x_, _, z_ = sag          # y_ ≈ 0 (projected out)

    # Coordinate mapping:
    #   x_sag =  x_   forward (Dingo x = Argos x)
    #   y_sag = -z_   downward (Dingo z is UP → negate)
    return theta_1, x_, -z_


# ══════════════════════════════════════════════════════════════════════════════
#  SAGITTAL IK (theta_top, theta_bot)
# ══════════════════════════════════════════════════════════════════════════════

def _sagittal_ik(x_sag, y_sag, P):
    """
    2-DOF coupled bell-crank IK in the sagittal plane.

    Parameters
    ----------
    x_sag, y_sag : float  foot position (m) relative to top servo shaft.
                          x_sag forward, y_sag downward.
    P            : dict   geometry from config.leg_params.

    Returns
    -------
    (theta_top, theta_bot) in radians, or None.
    """
    L1, L2 = P['L1'], P['L2']
    O = np.zeros(2)

    d = sqrt(x_sag*x_sag + y_sag*y_sag)
    max_r = (L1 + L2) * 0.99
    if d > max_r:
        scale = max_r / d;  x_sag *= scale;  y_sag *= scale;  d = max_r
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

    cos_k    = max(-1.0, min(1.0, (L1*L1+L2*L2-d*d)/(2*L1*L2)))
    knee_int = acos(cos_k)
    alpha    = atan2(x_sag, y_sag)
    cos_b    = max(-1.0, min(1.0, (L1*L1+d*d-L2*L2)/(2*L1*d)))
    t_upper  = alpha - acos(cos_b)          # elbow-back

    # ── Step 3: rod2 attachment above knee ───────────────────────────────
    knee = np.array([L1*sin(t_upper), L1*cos(t_upper)])
    t_lo = t_upper + (pi - knee_int)
    rod2_pt = knee - P['dko'] * np.array([sin(t_lo), cos(t_lo)])

    # ── Step 4: bell crank (loop 2) ───────────────────────────────────────
    A, B = _circle_intersect(O, P['rbl'], rod2_pt, P['Lr2'])
    if A is None:
        return None
    bc_left  = A if A[0] <= B[0] else B    # −X side
    phi_l    = atan2(bc_left[0], bc_left[1])
    phi_r    = phi_l - P['abc']            # right arm CW from left
    bc_right = np.array([P['rbr']*sin(phi_r), P['rbr']*cos(phi_r)])

    # ── Step 5: bottom servo (loop 1) ────────────────────────────────────
    A, B = _circle_intersect(O, P['rh'], bc_right, P['Lr1'])
    if A is None:
        return None
    horn_tip  = A if A[0] >= B[0] else B   # +X side
    return t_upper, atan2(horn_tip[0], horn_tip[1])


# ══════════════════════════════════════════════════════════════════════════════
#  SINGLE-LEG IK  (full 3-DOF)
# ══════════════════════════════════════════════════════════════════════════════

def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """
    Full 3-DOF IK for one leg.

    Parameters
    ----------
    r_body_foot : (3,) ndarray  body-frame foot position (m).
                                Dingo convention: x=fwd, y=lat-L+, z=up.
    leg_index   : int           0=FR, 1=FL, 2=RR, 3=RL.
    config      : Configuration

    Returns
    -------
    (3,) ndarray  [theta_1, theta_top, theta_bot] radians,
    or np.zeros(3) if infeasible (with a warning logged).
    """
    P = config.leg_params

    theta_1, x_sag, y_sag = _hip_abductor_ik(
        r_body_foot, leg_index, P['L1_hip'], P['phi'])

    sag = _sagittal_ik(x_sag, y_sag, P)
    if sag is None:
        if leg_index in _LAST_VALID_LEG_ANGLES:
            log.warning("leg_ik: sagittal IK infeasible for leg %d at sag (%.4f, %.4f); "
                        "using last valid angles", leg_index, x_sag, y_sag)
            return _LAST_VALID_LEG_ANGLES[leg_index].copy()

        log.warning("leg_ik: sagittal IK infeasible for leg %d at sag (%.4f, %.4f); "
                    "using guarded fallback", leg_index, x_sag, y_sag)
        return np.array([theta_1, 0.0, 0.0])

    theta_top, theta_bot = sag
    out = np.array([theta_1, theta_top, theta_bot])
    _LAST_VALID_LEG_ANGLES[leg_index] = out.copy()
    return out


# ══════════════════════════════════════════════════════════════════════════════
#  ALL-LEGS WRAPPER  (drop-in for Dingo's four_legs_inverse_kinematics)
# ══════════════════════════════════════════════════════════════════════════════

def four_legs_inverse_kinematics(r_body_foot, config):
    """
    IK for all four legs.

    Parameters
    ----------
    r_body_foot : (3, 4) ndarray  body-frame foot positions (m).
                                  Columns: [FR, FL, RR, RL].
    config      : Configuration

    Returns
    -------
    (3, 4) ndarray  joint angles (rad).
        Row 0: theta_1    hip abductor
        Row 1: theta_top  upper leg
        Row 2: theta_bot  lower leg (bell crank)
    """
    angles = np.zeros((3, 4))
    for i in range(4):
        r_leg = r_body_foot[:, i] - config.LEG_ORIGINS[:, i]
        angles[:, i] = leg_explicit_inverse_kinematics(r_leg, i, config)
    return angles


# ══════════════════════════════════════════════════════════════════════════════
#  FORWARD KINEMATICS  (sagittal plane only, for verification)
# ══════════════════════════════════════════════════════════════════════════════

def leg_fk(theta_top, theta_bot, params):
    """
    Sagittal FK for one leg.  Returns foot (x_sag, y_sag) in metres, or None.
    """
    L1, L2 = params['L1'], params['L2']
    O = np.zeros(2)

    knee = np.array([L1*sin(theta_top), L1*cos(theta_top)])
    horn = np.array([params['rh']*sin(theta_bot), params['rh']*cos(theta_bot)])

    A, B = _circle_intersect(O, params['rbr'], horn, params['Lr1'])
    if A is None:
        return None
    def _bcl_x(c): return sin(atan2(c[0], c[1]) + params['abc'])
    bc_right = A if _bcl_x(A) < _bcl_x(B) else B

    phi_r   = atan2(bc_right[0], bc_right[1])
    phi_l   = phi_r + params['abc']
    bc_left = np.array([params['rbl']*sin(phi_l), params['rbl']*cos(phi_l)])

    A, B = _circle_intersect(knee, params['dko'], bc_left, params['Lr2'])
    if A is None:
        return None

    def _diff(c):
        ld = knee - c
        d  = (atan2(ld[0], ld[1]) - theta_top) % (2*pi)
        return d - 2*pi if d > pi else d

    dA, dB = _diff(A), _diff(B)
    if 0 < dA < pi:      rod2 = A
    elif 0 < dB < pi:    rod2 = B
    else: rod2 = A if abs(dA-pi/2) < abs(dB-pi/2) else B

    ld   = (knee - rod2) / np.linalg.norm(knee - rod2)
    foot = knee + L2 * ld
    return float(foot[0]), float(foot[1])


# ══════════════════════════════════════════════════════════════════════════════
#  SELF-TEST
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':

    class _Cfg:
        L1_HIP=0.05162; PHI=radians(73.917)
        L_UPPER=0.130; L_LOWER=0.150
        R_BC_LEFT=0.035; R_BC_RIGHT=0.050; ALPHA_BC=radians(60.0)
        L_ROD1=0.065; L_ROD2=0.120; R_HORN=0.035; D_KNEE_OFFSET=0.030
        LEG_ORIGINS=np.zeros((3,4))
        @property
        def leg_params(self):
            return dict(L1_hip=self.L1_HIP, phi=self.PHI,
                        L1=self.L_UPPER, L2=self.L_LOWER,
                        rbl=self.R_BC_LEFT, rbr=self.R_BC_RIGHT,
                        abc=self.ALPHA_BC, Lr1=self.L_ROD1,
                        Lr2=self.L_ROD2, rh=self.R_HORN,
                        dko=self.D_KNEE_OFFSET)

    cfg = _Cfg(); P = cfg.leg_params

    print("=" * 66)
    print("  Argos 3-DOF Kinematics self-test")
    print("  θ_1 = hip abductor | θ_top = upper leg | θ_bot = bell-crank")
    print("=" * 66)

    # ── Sagittal IK/FK round-trip ─────────────────────────────────────────
    print("\n  Sagittal IK/FK round-trip:")
    sag_tests = [
        ( 0.000,  0.200, "Nominal"),
        ( 0.040,  0.190, "Swing fwd +40mm"),
        ( 0.080,  0.170, "Wide fwd"),
        (-0.040,  0.190, "Swing back -40mm"),
        (-0.080,  0.160, "Wide back"),
        ( 0.000,  0.170, "Raised"),
        ( 0.112,  0.200, "Rear leg fwd reach"),
        (-0.112,  0.200, "Front leg back reach"),
    ]
    print(f"  {'Pose':>22}   {'θ_top':>8}  {'θ_bot':>8}  FK err")
    print("  " + "─"*54)
    all_ok = True
    for x, y, name in sag_tests:
        sol = _sagittal_ik(x, y, P)
        if sol is None:
            print(f"  {name:<22}  UNREACHABLE"); continue
        fk = leg_fk(sol[0], sol[1], P)
        if fk is None:
            print(f"  {name:<22}  FK failed"); all_ok=False; continue
        err = sqrt((fk[0]-x)**2 + (fk[1]-y)**2)
        ok  = "✓" if err < 1e-6 else f"✗ {err*1000:.4f}mm"
        if err >= 1e-6: all_ok=False
        print(f"  {name:<22}   {np.degrees(sol[0]):+7.3f}°  {np.degrees(sol[1]):+7.3f}°  {ok}")
    print(f"\n  Sagittal: {'All passed ✓' if all_ok else 'FAILURES ✗'}")

    # ── Full 3-DOF all legs ───────────────────────────────────────────────
    print("\n  Full 3-DOF IK — default stance z=-0.200m:")
    LEG_ORIGINS = np.array([
        [ 0.11165,  0.11165, -0.11165, -0.11165],
        [-0.061,    0.061,   -0.061,    0.061  ],
        [0, 0, 0, 0],
    ])
    names = ['FR','FL','RR','RL']
    print(f"  {'Leg':>3}  {'θ_1':>8}  {'θ_top':>9}  {'θ_bot':>9}")
    print("  " + "─"*44)
    all_ok2 = True
    for i, name in enumerate(names):
        r = np.array([LEG_ORIGINS[0,i], LEG_ORIGINS[1,i], -0.200])
        sol = leg_explicit_inverse_kinematics(r, i, cfg)
        fk  = leg_fk(sol[1], sol[2], P)
        ok  = "✓" if fk else "?"
        print(f"  {name}   {np.degrees(sol[0]):+7.2f}°  "
              f"{np.degrees(sol[1]):+8.2f}°  {np.degrees(sol[2]):+8.2f}°  {ok}")

    print(f"\n  four_legs_inverse_kinematics output shape: ", end="")
    r_body = np.zeros((3,4)); r_body[2,:] = -0.200
    r_body[0,:] = LEG_ORIGINS[0,:]; r_body[1,:] = LEG_ORIGINS[1,:]
    out = four_legs_inverse_kinematics(r_body, cfg)
    print(f"{out.shape}  (expected (3, 4))")
    print(f"  Row 0 θ_1   (deg): {np.degrees(out[0]).round(2)}")
    print(f"  Row 1 θ_top (deg): {np.degrees(out[1]).round(2)}")
    print(f"  Row 2 θ_bot (deg): {np.degrees(out[2]).round(2)}")