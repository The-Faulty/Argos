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
        self.delta_x           = 0.100
        self.rear_leg_x_shift  = -0.04   # rear legs shifted back slightly
        self.front_leg_x_shift =  0.00
        self.delta_y           = 0.1106
        self.default_z_ref     = -0.189  # neutral stand height inside the measured top/bot envelope

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
        # Bench-tested direct-servo travel with the linkage fully assembled.
        # These are absolute horn commands in the servo's native 0..180 degree space.
        self.SERVO_CENTER_DEG = 90.0
        self.SERVO_LIMITS_DEG = np.array([
            [45.0, 135.0],   # hip servo
            [50.0, 115.0],   # top servo
            [5.0, 180.0],    # bottom/bell-crank servo
        ])
        self.COUPLED_TOP_SAMPLES_DEG = np.array([50.0, 70.0, 90.0, 102.0, 115.0])
        # Treat the 70 deg forward point like the 50 deg point; the measured 5 deg
        # difference looked like binding-detection noise rather than a real trend.
        self.COUPLED_BOT_MIN_SAMPLES_DEG = np.array([10.0, 10.0, 40.0, 65.0, 85.0])
        self.COUPLED_BOT_MAX_SAMPLES_DEG = np.array([95.0, 125.0, 180.0, 180.0, 180.0])
        # Use the measured hard stops by default. If you want extra bench margin
        # later, increase this in the single-leg tool while testing.
        self.BENCH_COUPLED_MARGIN_DEG = 0.0

        # Control-space actuator limits expressed relative to the 90 degree neutral.
        # The top/bottom pair is further constrained by the coupled linkage envelope
        # captured in the measured sample points above.
        self.JOINT_LIMITS_DEG = np.array([
            [-45.0, 45.0],   # hip abductor
            [-40.0, 25.0],   # upper leg (theta_top)
            [-85.0, 90.0],   # lower servo (theta_bot)
        ])
        self.JOINT_LIMITS_RAD = np.radians(self.JOINT_LIMITS_DEG)

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
    def joint_limits_deg(self):
        """(3, 2) array of [min, max] degrees in control-space coordinates."""
        return self.JOINT_LIMITS_DEG.copy()

    @property
    def servo_limits_deg(self):
        """(3, 2) array of [min, max] direct-servo degrees measured on the bench."""
        return self.SERVO_LIMITS_DEG.copy()

    @property
    def joint_limits_per_leg_rad(self):
        """(3, 4, 2) array of per-leg actuator [min, max] radians."""
        return self.JOINT_LIMITS_PER_LEG_RAD.copy()

    def coupled_bot_limits_servo_deg(
        self,
        top_servo_deg: float,
        margin_deg: float = 0.0,
    ) -> tuple[float, float]:
        """Return the allowed bottom-servo window for a given top-servo angle."""
        top_lo, top_hi = self.SERVO_LIMITS_DEG[1]
        clamped_top = float(np.clip(top_servo_deg, top_lo, top_hi))
        margin_deg = max(0.0, float(margin_deg))
        bot_min_samples = np.clip(
            self.COUPLED_BOT_MIN_SAMPLES_DEG + margin_deg,
            self.SERVO_LIMITS_DEG[2, 0],
            self.SERVO_LIMITS_DEG[2, 1],
        )
        bot_max_samples = np.clip(
            self.COUPLED_BOT_MAX_SAMPLES_DEG - margin_deg,
            self.SERVO_LIMITS_DEG[2, 0],
            self.SERVO_LIMITS_DEG[2, 1],
        )
        bot_min = float(np.interp(clamped_top, self.COUPLED_TOP_SAMPLES_DEG, bot_min_samples))
        bot_max = float(np.interp(clamped_top, self.COUPLED_TOP_SAMPLES_DEG, bot_max_samples))
        if bot_min > bot_max:
            midpoint = 0.5 * (bot_min + bot_max)
            bot_min = midpoint
            bot_max = midpoint
        return bot_min, bot_max

    def coupled_bot_limits_joint_deg(
        self,
        top_joint_deg: float,
        margin_deg: float = 0.0,
    ) -> tuple[float, float]:
        """Return the allowed bottom-joint window in control-space degrees."""
        bot_min, bot_max = self.coupled_bot_limits_servo_deg(
            self.SERVO_CENTER_DEG + float(top_joint_deg),
            margin_deg=margin_deg,
        )
        return bot_min - self.SERVO_CENTER_DEG, bot_max - self.SERVO_CENTER_DEG

    def coupled_bot_limits_joint_rad(
        self,
        top_joint_rad: float,
        margin_deg: float = 0.0,
    ) -> tuple[float, float]:
        """Return the allowed bottom-joint window in control-space radians."""
        bot_min_deg, bot_max_deg = self.coupled_bot_limits_joint_deg(
            np.degrees(float(top_joint_rad)),
            margin_deg=margin_deg,
        )
        return np.radians(bot_min_deg), np.radians(bot_max_deg)

    def clamp_joint_matrix(
        self,
        angle_matrix: np.ndarray,
        coupled_margin_deg: float = 0.0,
    ) -> np.ndarray:
        """Clamp a (3, N) control-space joint matrix to the measured hardware envelope."""
        matrix = np.asarray(angle_matrix, dtype=float).copy()
        if matrix.ndim != 2 or matrix.shape[0] != 3:
            raise ValueError(f"Expected a (3, N) angle matrix, got {matrix.shape}")

        hip_lo, hip_hi = self.JOINT_LIMITS_RAD[0]
        top_lo, top_hi = self.JOINT_LIMITS_RAD[1]
        bot_lo_global, bot_hi_global = self.JOINT_LIMITS_RAD[2]

        for leg_index in range(matrix.shape[1]):
            matrix[0, leg_index] = float(np.clip(matrix[0, leg_index], hip_lo, hip_hi))
            matrix[1, leg_index] = float(np.clip(matrix[1, leg_index], top_lo, top_hi))
            bot_lo, bot_hi = self.coupled_bot_limits_joint_rad(
                matrix[1, leg_index],
                margin_deg=coupled_margin_deg,
            )
            bot_lo = max(bot_lo_global, bot_lo)
            bot_hi = min(bot_hi_global, bot_hi)
            if bot_lo > bot_hi:
                midpoint = 0.5 * (bot_lo + bot_hi)
                bot_lo = midpoint
                bot_hi = midpoint
            matrix[2, leg_index] = float(np.clip(matrix[2, leg_index], bot_lo, bot_hi))
        return matrix

    def validate_servo_triplet_deg(
        self,
        hip_deg: float,
        top_deg: float,
        bot_deg: float,
        coupled_margin_deg: float = 0.0,
    ) -> tuple[bool, str]:
        """Check whether a direct servo command is inside the measured hardware envelope."""
        (_, _, _), changed, reason = self.clamp_servo_triplet_deg(
            hip_deg,
            top_deg,
            bot_deg,
            coupled_margin_deg=coupled_margin_deg,
        )
        return (not changed), reason

    def clamp_servo_triplet_deg(
        self,
        hip_deg: float,
        top_deg: float,
        bot_deg: float,
        coupled_margin_deg: float = 0.0,
    ) -> tuple[tuple[float, float, float], bool, str]:
        """Clamp a direct servo command to the measured hardware envelope."""
        hip_lo, hip_hi = self.SERVO_LIMITS_DEG[0]
        top_lo, top_hi = self.SERVO_LIMITS_DEG[1]
        bot_lo_hard, bot_hi_hard = self.SERVO_LIMITS_DEG[2]

        requested_hip = float(hip_deg)
        requested_top = float(top_deg)
        requested_bot = float(bot_deg)
        clamped_hip = float(np.clip(requested_hip, hip_lo, hip_hi))
        clamped_top = float(np.clip(requested_top, top_lo, top_hi))

        bot_lo, bot_hi = self.coupled_bot_limits_servo_deg(
            clamped_top,
            margin_deg=coupled_margin_deg,
        )
        bot_lo = max(bot_lo_hard, bot_lo)
        bot_hi = min(bot_hi_hard, bot_hi)
        clamped_bot = float(np.clip(requested_bot, bot_lo, bot_hi))

        reasons = []
        if abs(clamped_hip - requested_hip) > 1e-9:
            reasons.append(
                f"hip {requested_hip:.1f}->{clamped_hip:.1f} deg "
                f"(safe range [{hip_lo:.1f}, {hip_hi:.1f}])"
            )
        if abs(clamped_top - requested_top) > 1e-9:
            reasons.append(
                f"top {requested_top:.1f}->{clamped_top:.1f} deg "
                f"(safe range [{top_lo:.1f}, {top_hi:.1f}])"
            )
        if abs(clamped_bot - requested_bot) > 1e-9:
            reasons.append(
                f"bot {requested_bot:.1f}->{clamped_bot:.1f} deg "
                f"(safe range [{bot_lo:.1f}, {bot_hi:.1f}] for top={clamped_top:.1f})"
            )

        changed = bool(reasons)
        return (clamped_hip, clamped_top, clamped_bot), changed, "; ".join(reasons)
