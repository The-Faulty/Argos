"""Small control primitives shared by the ROS 2 gait code."""

from enum import Enum

import numpy as np
from transforms3d.euler import euler2mat


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3


class State:
    """Tracks the controller state from one update to the next."""

    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.25
        self.pitch = 0.0
        self.roll = 0.0
        self.joystick_control_active = 1
        self.behavior_state = BehaviorState.REST
        self.euler_orientation = [0.0, 0.0, 0.0]
        self.ticks = 0
        self.foot_locations = np.zeros((3, 4))
        self.rotated_foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
        self.speed_factor = 1
        self.currently_estopped = 0


class GaitController:
    """Maps controller ticks to stance/swing phases."""

    def __init__(self, config):
        self.config = config

    def phase_index(self, ticks: int) -> int:
        """Which phase (0 to num_phases-1) the given tick falls in."""
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for phase_index in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[phase_index]
            if phase_time < phase_sum:
                return phase_index
        raise RuntimeError("Could not resolve gait phase from controller ticks.")

    def subphase_ticks(self, ticks: int) -> int:
        """Ticks elapsed since the start of the current phase."""
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for phase_index in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[phase_index]
            if phase_time < phase_sum:
                return phase_time - phase_sum + self.config.phase_ticks[phase_index]
        raise RuntimeError("Could not resolve gait subphase from controller ticks.")

    def contacts(self, ticks: int) -> np.ndarray:
        """Contact mask for all four legs at the given tick (1 = grounded)."""
        return self.config.contact_phases[:, self.phase_index(ticks)]


class StanceController:
    """Moves a stance foot opposite the body motion so the body advances."""

    def __init__(self, config):
        self.config = config

    def position_delta(self, leg_index: int, state: State, command) -> tuple[np.ndarray, np.ndarray]:
        """Compute translation and rotation to apply to a grounded foot this tick."""
        z = state.foot_locations[2, leg_index]
        velocity = np.array(
            [
                -command.horizontal_velocity[0],
                -command.horizontal_velocity[1],
                # Slowly bring the foot to the commanded height
                (state.height - z) / self.config.z_time_constant,
            ]
        )
        delta_position = velocity * self.config.dt
        delta_rotation = euler2mat(0.0, 0.0, -command.yaw_rate * self.config.dt)
        return delta_position, delta_rotation

    def next_foot_location(self, leg_index: int, state: State, command) -> np.ndarray:
        """Return the stance foot position after applying one tick of body motion."""
        foot_location = state.foot_locations[:, leg_index]
        delta_position, delta_rotation = self.position_delta(leg_index, state, command)
        return delta_rotation @ foot_location + delta_position


class SwingController:
    """Moves a swing foot toward the next touchdown point."""

    def __init__(self, config):
        self.config = config

    def raibert_touchdown_location(self, leg_index: int, command) -> np.ndarray:
        """Predict where the foot should land using the Raibert heuristic."""
        delta_xy = (
            self.config.alpha
            * self.config.stance_ticks
            * self.config.dt
            * command.horizontal_velocity
        )
        delta_position = np.array([delta_xy[0], delta_xy[1], 0.0])
        yaw = (
            self.config.beta
            * self.config.stance_ticks
            * self.config.dt
            * command.yaw_rate
        )
        return euler2mat(0.0, 0.0, yaw) @ self.config.default_stance[:, leg_index] + delta_position

    def swing_height(self, swing_phase: float) -> float:
        """Triangle height profile: rises to z_clearance at mid-swing, back to 0 at touchdown."""
        if swing_phase < 0.5:
            return swing_phase / 0.5 * self.config.z_clearance
        return self.config.z_clearance * (1.0 - (swing_phase - 0.5) / 0.5)

    def next_foot_location(
        self,
        swing_phase: float,
        leg_index: int,
        state: State,
        command,
        touchdown_override: np.ndarray | None = None,
    ) -> np.ndarray:
        """Interpolate the foot toward the predicted touchdown point."""
        if not 0.0 <= swing_phase <= 1.0:
            raise ValueError("Swing phase must stay between 0 and 1.")

        foot_location = state.foot_locations[:, leg_index]
        touchdown = (
            np.asarray(touchdown_override, dtype=float)
            if touchdown_override is not None
            else self.raibert_touchdown_location(leg_index, command)
        )
        time_left = self.config.dt * self.config.swing_ticks * (1.0 - swing_phase)
        # XY moves toward touchdown; Z is driven by the swing height profile
        swing_velocity = (touchdown - foot_location) / time_left * np.array([1.0, 1.0, 0.0])
        delta_foot_location = swing_velocity * self.config.dt
        z_vector = np.array([0.0, 0.0, self.swing_height(swing_phase) + command.height])
        return foot_location * np.array([1.0, 1.0, 0.0]) + z_vector + delta_foot_location
