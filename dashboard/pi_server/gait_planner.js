// Gait planner — JS port of ros2_ws/argos_control/gait_planner_node.py.
//
// Runs a fixed 50 Hz tick that consumes a Twist (x, y, yaw m/s, rad/s) plus the
// latest IMU euler angles and produces a 12-element servo angle vector ready
// for serial_bridge.setAllServoAnglesDeg().
//
// Modes:
//   idle                  — hold last commanded servos, no updates
//   crouch                — hold zero-angle pose (servos at center)
//   stand / extend        — hold default stance feet positions, optional IMU tilt comp
//   crawl / trot          — periodic gait with contact phases
//   direct_*              — caller bypasses the planner and pushes commands itself
//   animation_playback    — caller drives feet from a clip
//
// The planner does not own the serial port; it returns vectors and the
// mode_controller decides whether to publish.

import {
  LEG_IDS,
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_ORIGINS,
  DEFAULT_STANCE,
  DEFAULT_Z_REF,
  DEFAULT_JOINT_LIMITS_RAD,
  STABILIZER_PARAM_BOUNDS,
  GAIT_TUNABLE_PARAMS,
} from "../shared/robot-config.js";
import { fourLegsInverseKinematics } from "../shared/argos_kinematics.js";
import { applyServoCal, clampServoDeg } from "../shared/servo_cal.js";

const TICK_HZ = 50.0;
const TICK_DT = 1.0 / TICK_HZ;

// Default contact phase patterns (4 columns = phases, rows = legs FR/FL/RR/RL).
// 1 = stance (foot down), 0 = swing (foot in air).
const CONTACT_PHASES = {
  trot: [
    [1, 0, 1, 1], // FR
    [1, 1, 1, 0], // FL
    [1, 1, 1, 0], // RR
    [1, 0, 1, 1], // RL
  ],
  crawl: [
    [1, 1, 1, 0], // FR — swings on phase 4
    [1, 0, 1, 1], // FL — swings on phase 2
    [1, 1, 0, 1], // RR — swings on phase 3
    [0, 1, 1, 1], // RL — swings on phase 1
  ],
};

const PHASE_TIMES = {
  // ms per phase: 3 stance phases + 1 swing phase per leg
  trot:  { stance_ms: 75,  swing_ms: 150 },
  crawl: { stance_ms: 250, swing_ms: 250 },
};

export class GaitPlanner {
  constructor() {
    this.config = makeDefaultConfig();
    this.mode = "idle";
    this.twist = { x: 0, y: 0, yaw: 0 };
    this.imu = { roll: 0, pitch: 0, yaw: 0 };
    this.imuFiltered = { roll: 0, pitch: 0 };
    this.tickCount = 0;
    this.gaitTickMs = 0;
    this.feet = makeStanceFeet(this.config); // 4×3 in body frame
    this.lastJointAnglesRad = makeZeroJoints();
    this.lastServoAnglesDeg = jointAnglesRadToServoDeg(this.lastJointAnglesRad);
  }

  // ─── External knobs ──────────────────────────────────────────────────

  setMode(mode) {
    if (this.mode === mode) return;
    this.mode = mode;
    this.tickCount = 0;
    this.gaitTickMs = 0;
    this.feet = makeStanceFeet(this.config);
  }

  setTwist({ x = 0, y = 0, yaw = 0 } = {}) {
    this.twist = {
      x: clampNum(x, -0.4, 0.4),
      y: clampNum(y, -0.4, 0.4),
      yaw: clampNum(yaw, -this.config.rotate_rate_max, this.config.rotate_rate_max),
    };
  }

  setImu({ roll = 0, pitch = 0, yaw = 0 } = {}) {
    this.imu = { roll, pitch, yaw };
  }

  updateConfig(patch = {}) {
    Object.assign(this.config, patch);
    this.feet = makeStanceFeet(this.config);
  }

  // Returns { servoAnglesDeg, jointAnglesRad, feet } or null when planner is
  // disabled (e.g. mode is idle/direct_*/animation_playback).
  step() {
    this.tickCount++;
    if (this.mode === "idle") return null;
    if (this.mode === "crouch") return this._crouchPose();
    if (this.mode === "stand" || this.mode === "extend") return this._stancePose(this.mode);
    if (this.mode === "crawl" || this.mode === "trot") return this._gaitPose(this.mode);
    return null; // direct_* / animation_playback handled elsewhere
  }

  _crouchPose() {
    // Zero angles → all servos at center (90 deg).
    const jointAnglesRad = makeZeroJoints();
    const servoAnglesDeg = jointAnglesRadToServoDeg(jointAnglesRad);
    this.lastJointAnglesRad = jointAnglesRad;
    this.lastServoAnglesDeg = servoAnglesDeg;
    return { servoAnglesDeg, jointAnglesRad, feet: makeStanceFeet(this.config) };
  }

  _stancePose(mode) {
    const feet = makeStanceFeet(this.config, mode === "extend" ? -0.02 : 0);
    this._applyImuTilt(feet);
    return this._solveAndCache(feet);
  }

  _gaitPose(mode) {
    const phases = CONTACT_PHASES[mode];
    const phaseDur = gaitPhaseTimes(this.config, mode);
    const cycleMs = phaseDur.stance_ms * 3 + phaseDur.swing_ms;
    this.gaitTickMs = (this.gaitTickMs + TICK_DT * 1000) % cycleMs;

    // Determine which phase column we're in (each leg has 4 phases per cycle).
    // For simplicity we treat the cycle as 4 equal-duration slots; advanced
    // phase scheduling lives on the TODO list.
    const phaseDurMs = cycleMs / 4;
    const phaseIdx = Math.min(3, Math.floor(this.gaitTickMs / phaseDurMs));
    const phaseProgress = (this.gaitTickMs - phaseIdx * phaseDurMs) / phaseDurMs;

    const baseFeet = makeStanceFeet(this.config);
    const stride = this._strideOffsets();
    const feet = baseFeet.map((foot, legIdx) => {
      const isStance = phases[legIdx][phaseIdx] === 1;
      if (isStance) {
        // Stance: shift back along motion direction.
        return [
          foot[0] - stride[legIdx][0] * phaseProgress,
          foot[1] - stride[legIdx][1] * phaseProgress,
          foot[2],
        ];
      }
      // Swing: parabolic arc from -stride/2 to +stride/2 with z_clearance lift.
      const t = phaseProgress;
      const lift = this.config.z_clearance * 4 * t * (1 - t);
      return [
        foot[0] + stride[legIdx][0] * (t - 0.5),
        foot[1] + stride[legIdx][1] * (t - 0.5),
        foot[2] + lift,
      ];
    });
    this._applyImuTilt(feet);
    return this._solveAndCache(feet);
  }

  // Per-leg foot displacement contributing to one full gait cycle.
  _strideOffsets() {
    const { x, y, yaw } = this.twist;
    const swingSeconds = gaitPhaseTimes(this.config, this.mode).swing_ms / 1000;
    const offsets = [];
    for (let i = 0; i < 4; i++) {
      const origin = [LEG_ORIGINS[0][i], LEG_ORIGINS[1][i], 0];
      // Yaw contribution: foot moves perpendicular to its hip vector
      const yawDx = -yaw * origin[1];
      const yawDy = yaw * origin[0];
      offsets.push([
        (x + yawDx) * swingSeconds,
        (y + yawDy) * swingSeconds,
        0,
      ]);
    }
    return offsets;
  }

  _applyImuTilt(feet) {
    if (!this.config.use_imu_stabilization) return;
    const alpha = this.config.imu_filter_alpha;
    this.imuFiltered.roll  = alpha * this.imu.roll  + (1 - alpha) * this.imuFiltered.roll;
    this.imuFiltered.pitch = alpha * this.imu.pitch + (1 - alpha) * this.imuFiltered.pitch;

    const rGain = this.config.stabilization_roll_gain;
    const pGain = this.config.stabilization_pitch_gain;
    const cap = this.config.stabilization_max_correction_rad;

    const dRoll  = clampNum(-this.imuFiltered.roll  * rGain, -cap, cap);
    const dPitch = clampNum(-this.imuFiltered.pitch * pGain, -cap, cap);

    // Rotate each foot about body x (roll) and body y (pitch) by the
    // small-angle corrections. Positive roll → tilt right, push left feet down.
    const cr = Math.cos(dRoll), sr = Math.sin(dRoll);
    const cp = Math.cos(dPitch), sp = Math.sin(dPitch);
    for (let i = 0; i < 4; i++) {
      const [x, y, z] = feet[i];
      const y1 =  cr * y - sr * z;
      const z1 =  sr * y + cr * z;
      const x2 =  cp * x + sp * z1;
      const z2 = -sp * x + cp * z1;
      feet[i] = [x2, y1, z2];
    }
  }

  _solveAndCache(feet) {
    const ik = fourLegsInverseKinematics(feet, { limits: DEFAULT_JOINT_LIMITS_RAD });
    if (!ik.ok) {
      // Refuse to publish bad IK — return last good output.
      return {
        servoAnglesDeg: this.lastServoAnglesDeg,
        jointAnglesRad: this.lastJointAnglesRad,
        feet,
        error: `IK failed for ${ik.leg}: ${ik.reason}`,
      };
    }
    const jointAnglesRad = flattenJoints(ik.angles);
    const servoAnglesDeg = jointAnglesRadToServoDeg(jointAnglesRad);
    this.lastJointAnglesRad = jointAnglesRad;
    this.lastServoAnglesDeg = servoAnglesDeg;
    this.feet = feet;
    return { servoAnglesDeg, jointAnglesRad, feet };
  }
}

// ─── Helpers ──────────────────────────────────────────────────────────────

function makeDefaultConfig() {
  return {
    delta_x: GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000,
    delta_y: GAIT_TUNABLE_PARAMS.delta_y_mm.default / 1000,
    swing_time_ms: GAIT_TUNABLE_PARAMS.swing_time_ms.default,
    rotate_rate_max: GAIT_TUNABLE_PARAMS.rotate_rate_max.default,
    default_z_ref: GAIT_TUNABLE_PARAMS.default_z_ref_mm.default / 1000,
    z_clearance: 0.04,
    stabilization_roll_gain:  STABILIZER_PARAM_BOUNDS.stabilization_roll_gain.default,
    stabilization_pitch_gain: STABILIZER_PARAM_BOUNDS.stabilization_pitch_gain.default,
    stabilization_max_correction_rad: STABILIZER_PARAM_BOUNDS.stabilization_max_correction_rad.default,
    imu_filter_alpha: STABILIZER_PARAM_BOUNDS.imu_filter_alpha.default,
    use_imu_stabilization: true,
  };
}

function makeStanceFeet(cfg, zShift = 0) {
  // 4×3 array of foot positions in body frame, FR/FL/RR/RL. Keep the per-leg
  // front/rear offsets from DEFAULT_STANCE, but let the live-tuneable spread
  // parameters actually shape the footprint.
  const deltaX = cfg?.delta_x ?? GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000;
  const deltaY = cfg?.delta_y ?? GAIT_TUNABLE_PARAMS.delta_y_mm.default / 1000;
  const frontShift = DEFAULT_STANCE.FR[0] - GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000;
  const rearShift = DEFAULT_STANCE.RR[0] + GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000;
  return LEG_IDS.map((id) => {
    const isFront = id === "FR" || id === "FL";
    const isLeft = id === "FL" || id === "RL";
    const x = isFront ? deltaX + frontShift : -deltaX + rearShift;
    const y = isLeft ? deltaY : -deltaY;
    return [x, y, (cfg?.default_z_ref ?? DEFAULT_Z_REF) + zShift];
  });
}

function gaitPhaseTimes(cfg, mode) {
  const defaults = PHASE_TIMES[mode];
  if (!defaults) return { stance_ms: 0, swing_ms: 0 };
  const scale = clampNum(
    (cfg?.swing_time_ms ?? GAIT_TUNABLE_PARAMS.swing_time_ms.default)
      / GAIT_TUNABLE_PARAMS.swing_time_ms.default,
    0.25,
    4.0,
  );
  return {
    stance_ms: defaults.stance_ms * scale,
    swing_ms: defaults.swing_ms * scale,
  };
}

function makeZeroJoints() {
  return new Array(JOINT_NAMES.length).fill(0);
}

function flattenJoints(angles4x3) {
  const out = new Array(JOINT_NAMES.length);
  for (let li = 0; li < 4; li++) {
    for (let ji = 0; ji < 3; ji++) {
      out[li * 3 + ji] = angles4x3[li][ji];
    }
  }
  return out;
}

function jointAnglesRadToServoDeg(jointAnglesRad) {
  const out = new Array(JOINT_NAMES.length);
  for (let i = 0; i < JOINT_NAMES.length; i++) {
    const name = JOINT_NAMES[i];
    const raw = applyServoCal(name, jointAnglesRad[i]);
    out[i] = clampServoDeg(name, raw);
  }
  return out;
}

function clampNum(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

export const _internals = {
  TICK_HZ,
  TICK_DT,
  CONTACT_PHASES,
  PHASE_TIMES,
  gaitPhaseTimes,
  makeStanceFeet,
  jointAnglesRadToServoDeg,
};
