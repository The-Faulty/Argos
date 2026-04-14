"""All robot geometry and gait settings in one place.

All distances are in meters. If the hardware changes, update the leg
geometry section below — everything else derives from it.
"""

import numpy as np
import math as m


class Configuration:
    """Stores every tunable parameter for the Argos quadruped."""

    def __init__(self):

        # ── Velocity limits ───────────────────────────────────────────────
        self.max_x_velocity  = 1.2
        self.max_y_velocity  = 0.5
        self.max_yaw_rate    = 2.0
        self.max_pitch       = np.radians(30.0)

        # ── Motion filter params ──────────────────────────────────────────
        # These control how quickly the body responds to height/pitch/yaw commands.
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
        # delta_x/delta_y = front-back and left-right foot spread from body center
        self.delta_x           = 0.117
        self.rear_leg_x_shift  = -0.04   # rear legs shifted back slightly
        self.front_leg_x_shift =  0.00
        self.delta_y           = 0.1106
        self.default_z_ref     = -0.165  # standing height (valid range: ~-0.185 to -0.105 m)
        # NOTE: -0.25 m is outside the IK workspace for the real bell-crank geometry

        # ── Swing params ──────────────────────────────────────────────────
        self.z_clearance = 0.07   # how high the foot lifts during swing
        self.alpha = 0.5          # Raibert touchdown prediction weight
        self.beta  = 0.5          # Raibert yaw prediction weight

        # ── Gait timing ───────────────────────────────────────────────────
        self.dt             = 0.01        # control loop period (s)
        self.num_phases     = 4
        # Contact pattern for trot: 1 = foot on ground, 0 = foot in swing.
        # Rows = legs (FR, FL, RR, RL), columns = phases.
        self.contact_phases = np.array(
            [[1, 1, 1, 0],
             [1, 0, 1, 1],
             [1, 0, 1, 1],
             [1, 1, 1, 0]]
        )
        self.overlap_time = 0.04   # time both feet are grounded at phase transitions
        self.swing_time   = 0.07   # time one foot spends in the air

        # ── Leg origin offsets from body center ───────────────────────────
        self.LEG_FB = 0.11165   # forward/back distance to hip pivot
        self.LEG_LR = 0.061     # left/right distance to hip pivot
        self.LEG_ORIGINS = np.array([
            [ self.LEG_FB,  self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
            [-self.LEG_LR,  self.LEG_LR, -self.LEG_LR,  self.LEG_LR],
            [0, 0, 0, 0],
        ])

        # ── Leg geometry ──────────────────────────────────────────────────
        # Update these if the printed parts or linkage dimensions change.

        # Hip abductor: coxa servo shaft to femur pivot.
        # PHI = neutral coxa angle measured from the body lateral axis.
        self.L1_HIP = 0.035563
        self.PHI    = m.radians(20.156)

        # Sagittal links
        self.L_UPPER = 0.127063   # top servo shaft → knee hinge
        self.L_LOWER = 0.127183   # knee hinge → foot tip

        # Bell-crank geometry (couples bottom servo to the knee joint)
        self.R_BC_LEFT   = 0.04              # left arm length (→ Rod 2)
        self.R_BC_RIGHT  = 0.04              # right arm length (← Rod 1)
        self.ALPHA_BC    = m.radians(90.0)   # angle between the two arms

        # Push-rod lengths and horn radius
        self.L_ROD1 = 0.03
        self.L_ROD2 = 0.150
        self.R_HORN = 0.024

        # Rod 2 attaches above the knee pivot by this distance
        self.D_KNEE_OFFSET = 0.030023

        # ── Joint limits ──────────────────────────────────────────────────
        # Tightened to the coupled bell-crank workspace (validated via IK sweep).
        self.JOINT_LIMITS_RAD = np.array([
            [m.radians(-45.0),  m.radians(45.0)],   # hip abductor
            [m.radians(-75.0),  m.radians(10.0)],   # upper leg (theta_top)
            [m.radians(-20.0),  m.radians(80.0)],   # lower servo (theta_bot)
        ])

        # Per-leg limits — same as global by default, customize per leg if needed.
        # Shape: (3 joints, 4 legs, 2 = [min, max])
        self.JOINT_LIMITS_PER_LEG_RAD = np.repeat(
            self.JOINT_LIMITS_RAD[:, np.newaxis, :], 4, axis=1
        )

        # Aliases so older code paths don't break
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
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)
        leg_mass = 0.010
        leg_x    = (1/12) * self.L_UPPER**2 * leg_mass
        self.LEG_INERTIA = (leg_x, leg_x, 1e-6)

    # ── Stance ────────────────────────────────────────────────────────────

    @property
    def default_stance(self):
        """Foot positions (body frame, z=0) for a neutral standing pose."""
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
        """Number of ticks both feet are grounded at each phase transition."""
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        """Number of ticks one foot spends in the air."""
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        """Total ticks a foot spends on the ground per full gait cycle."""
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        """Tick count for each of the four gait phases."""
        return np.array([
            self.overlap_ticks, self.swing_ticks,
            self.overlap_ticks, self.swing_ticks,
        ])

    @property
    def phase_length(self):
        """Total ticks in one complete gait cycle."""
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

    # ── IK parameter dict ─────────────────────────────────────────────────

    @property
    def leg_params(self) -> dict:
        """All kinematic parameters packed into a dict for Kinematics.py."""
        return dict(
            L1_hip = self.L1_HIP,
            phi    = self.PHI,
            L1     = self.L_UPPER,
            L2     = self.L_LOWER,
            rbl    = self.R_BC_LEFT,
            rbr    = self.R_BC_RIGHT,
            abc    = self.ALPHA_BC,
            Lr1    = self.L_ROD1,
            Lr2    = self.L_ROD2,
            rh     = self.R_HORN,
            dko    = self.D_KNEE_OFFSET,
        )

    @property
    def joint_limits_rad(self):
        """(3, 2) array of [min, max] radians for each joint row."""
        return self.JOINT_LIMITS_RAD.copy()

    @property
    def joint_limits_per_leg_rad(self):
        """(3, 4, 2) array of per-leg [min, max] radians."""
        return self.JOINT_LIMITS_PER_LEG_RAD.copy()
