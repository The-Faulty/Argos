"""Geometry and gait settings for the Argos quadruped.

All distances are in meters. If the hardware changes, the leg geometry section
is the main place to update.
"""

import numpy as np
import math as m


class Configuration:
    def __init__(self):

        # ── Input limits ──────────────────────────────────────────────────
        self.max_x_velocity  = 1.2
        self.max_y_velocity  = 0.5
        self.max_yaw_rate    = 2.0
        self.max_pitch       = np.radians(30.0)

        # ── Motion filter params ──────────────────────────────────────────
        self.z_time_constant      = 0.02
        self.z_speed              = 0.06
        self.pitch_deadband       = 0.05
        self.pitch_time_constant  = 0.25
        self.max_pitch_rate       = 0.3
        self.roll_speed           = 0.1
        self.yaw_time_constant    = 0.3
        self.max_stance_yaw       = 1.2
        self.max_stance_yaw_rate  = 1.0

        # ── Stance geometry ───────────────────────────────────────────────
        self.delta_x           = 0.117
        self.rear_leg_x_shift  = -0.04
        self.front_leg_x_shift =  0.00
        self.delta_y           = 0.1106
        self.default_z_ref     = -0.165  # Valid range: ~-0.185 to -0.105 m
        # NOTE: previous value of -0.25 is outside the IK workspace
        # for the real Argos bell-crank geometry.

        # ── Swing params ──────────────────────────────────────────────────
        self.z_clearance = 0.07
        self.alpha = 0.5
        self.beta  = 0.5

        # ── Gait params ───────────────────────────────────────────────────
        self.dt             = 0.01
        self.num_phases     = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0],
             [1, 0, 1, 1],
             [1, 0, 1, 1],
             [1, 1, 1, 0]]
        )
        self.overlap_time = 0.04
        self.swing_time   = 0.07

        # ── Body / leg-origin geometry ────────────────────────────────────
        self.LEG_FB = 0.11165
        self.LEG_LR = 0.061
        self.LEG_ORIGINS = np.array([
            [ self.LEG_FB,  self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
            [-self.LEG_LR,  self.LEG_LR, -self.LEG_LR,  self.LEG_LR],
            [0, 0, 0, 0],
        ])

        # Leg geometry. Update these values if the printed parts or linkage
        # dimensions change.

        # Hip abductor geometry:
        # L1_HIP = coxa servo shaft to femur pivot.
        # PHI = neutral coxa angle measured from the body lateral axis.

        self.L1_HIP = 0.035563   # m  Hip abductor link length (coxa)
        self.PHI    = m.radians(20.156)

        # Sagittal linkage geometry:
        # L_UPPER = top servo shaft to knee hinge.
        # L_LOWER = knee hinge to foot tip.
        # R_BC_LEFT / R_BC_RIGHT / ALPHA_BC describe the bell crank.
        # L_ROD1 / L_ROD2 / R_HORN describe the lower linkage drive.
        # D_KNEE_OFFSET is the rod-2 pickup point above the knee.

        self.L_UPPER = 0.127063        # m  top servo shaft → knee hinge
        self.L_LOWER = 0.127183        # m  knee hinge → foot tip

        self.R_BC_LEFT   = 0.04    # m  bell crank left  arm (→ Rod 2)
        self.R_BC_RIGHT  = 0.04    # m  bell crank right arm (← Rod 1)
        self.ALPHA_BC    = m.radians(90.0)

        self.L_ROD1 = 0.03         # m  Rod 1 length
        self.L_ROD2 = 0.150         # m  Rod 2 length

        self.R_HORN = 0.024         # m  bottom servo horn arm length

        self.D_KNEE_OFFSET = 0.030023  # m  Rod 2 pivot: above knee along lower leg

        # Global joint limits (radians) in row order [theta_1, theta_top, theta_bot].
        # Tightened to match the coupled bell-crank workspace (validated via
        # numerical IK sweep — see Kinematics.py self-test).
        self.JOINT_LIMITS_RAD = np.array([
            [m.radians(-45.0),  m.radians(45.0)],    # hip abductor
            [m.radians(-75.0),  m.radians(10.0)],    # upper leg (theta_top)
            [m.radians(-20.0),  m.radians(80.0)],    # lower servo (theta_bot)
        ])

        # Per-leg joint limits (radians) with shape (3, 4, 2):
        #   axis 0 = joint row [theta_1, theta_top, theta_bot]
        #   axis 1 = leg column [FR, FL, RR, RL]
        #   axis 2 = [min, max]
        # Default: replicate global limits across all legs, then customize by leg.
        self.JOINT_LIMITS_PER_LEG_RAD = np.repeat(
            self.JOINT_LIMITS_RAD[:, np.newaxis, :], 4, axis=1
        )

        # Compatibility aliases for older code paths.
        self.L1 = self.L1_HIP
        self.L2 = self.L_UPPER
        self.L3 = self.L_LOWER
        self.phi = self.PHI

        # ── Inertial properties ───────────────────────────────────────────
        self.FRAME_MASS  = 0.560
        self.MODULE_MASS = 0.080
        self.LEG_MASS    = 0.030
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS) * 4
        self.FRAME_INERTIA  = tuple(
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3)))
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)
        leg_mass = 0.010
        leg_x    = (1/12) * self.L_UPPER**2 * leg_mass
        self.LEG_INERTIA = (leg_x, leg_x, 1e-6)

    # ── Stance ────────────────────────────────────────────────────────────
    @property
    def default_stance(self):
        return np.array([
            [
                self.delta_x + self.front_leg_x_shift,
                self.delta_x + self.front_leg_x_shift,
               -self.delta_x + self.rear_leg_x_shift,
               -self.delta_x + self.rear_leg_x_shift,
            ],
            [-self.delta_y,  self.delta_y, -self.delta_y,  self.delta_y],
            [0, 0, 0, 0],
        ])

    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    # ── Gait tick helpers ─────────────────────────────────────────────────
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array([
            self.overlap_ticks, self.swing_ticks,
            self.overlap_ticks, self.swing_ticks,
        ])

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

    # ── Geometry dict for IK/FK ───────────────────────────────────────────
    @property
    def leg_params(self) -> dict:
        """All kinematic parameters in one dict for Kinematics.py."""
        return dict(
            # Hip abductor
            L1_hip = self.L1_HIP,
            phi    = self.PHI,
            # Sagittal links
            L1     = self.L_UPPER,
            L2     = self.L_LOWER,
            # Bell crank
            rbl    = self.R_BC_LEFT,
            rbr    = self.R_BC_RIGHT,
            abc    = self.ALPHA_BC,
            # Rods and horn
            Lr1    = self.L_ROD1,
            Lr2    = self.L_ROD2,
            rh     = self.R_HORN,
            # Knee pivot
            dko    = self.D_KNEE_OFFSET,
        )

    @property
    def joint_limits_rad(self):
        """(3, 2) array: [min, max] radians for [theta_1, theta_top, theta_bot]."""
        return self.JOINT_LIMITS_RAD.copy()

    @property
    def joint_limits_per_leg_rad(self):
        """(3, 4, 2) array of per-leg [min, max] radians for each joint row."""
        return self.JOINT_LIMITS_PER_LEG_RAD.copy()