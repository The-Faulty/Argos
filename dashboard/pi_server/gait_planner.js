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
    // Linear cap is set so that x_max * swing_time_max equals the sagittal IK
    // reach at neutral z (~0.18 m). Bumping past 0.6 m/s only helps if you
    // also extend swing_time_ms, otherwise per-step distance is swing-bound.
    this.twist = {
      x: clampNum(x, -0.6, 0.6),
      y: clampNum(y, -0.6, 0.6),
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
    // stand: feet land directly below each hip (y_foot = y_hip = LEG_ORIGINS[1][i])
    // so the abductor IK returns coxa = 0 and all four coxa servos hold center.
    // extend keeps the wider delta_y so the body sits lower / wider on its feet.
    const feet = mode === "stand"
      ? this._standFeet()
      : makeStanceFeet(this.config, -0.02);
    this._applyImuTilt(feet);
    return this._solveAndCache(feet);
  }

  _standFeet() {
    const z = this.config.default_z_ref;
    const deltaX = this.config.delta_x;
    const frontShift = DEFAULT_STANCE.FR[0] - GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000;
    const rearShift  = DEFAULT_STANCE.RR[0] + GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000;
    return LEG_IDS.map((id, i) => {
      const isFront = id === "FR" || id === "FL";
      const x = isFront ? deltaX + frontShift : -deltaX + rearShift;
      return [x, LEG_ORIGINS[1][i], z];
    });
  }

  _gaitPose(mode) {
    const phases = CONTACT_PHASES[mode];
    const phaseDur = gaitPhaseTimes(this.config, mode);
    const cycleMs = phaseDur.stance_ms * 3 + phaseDur.swing_ms;
    this.gaitTickMs = (this.gaitTickMs + TICK_DT * 1000) % cycleMs;

    // Stride is the body-frame foot displacement over ONE full cycle, so the
    // per-step distance scales with both joystick magnitude and swing-time
    // (which scales the whole cycle). Using the full cycle period — instead
    // of just swing time — is what gives the operator a visible stride that
    // tracks the slider in Settings.
    const stride = this._strideForCycle(cycleMs / 1000);
    const baseFeet = makeStanceFeet(this.config);
    const phaseDurMs = cycleMs / 4;
    const stanceMs = cycleMs - phaseDurMs; // 3 of 4 phases under both contact patterns

    const feet = baseFeet.map((foot, legIdx) => {
      // Each leg has exactly one swing phase per cycle in CONTACT_PHASES.
      const swingPhase = phases[legIdx].indexOf(0);
      const swingStartMs = swingPhase * phaseDurMs;
      const swingEndMs   = swingStartMs + phaseDurMs;
      const t = this.gaitTickMs;
      const inSwing = t >= swingStartMs && t < swingEndMs;

      if (inSwing) {
        // Swing: -stride/2 → +stride/2 with sin-arc lift (peak at u=0.5).
        const u = (t - swingStartMs) / phaseDurMs;
        const lift = this.config.z_clearance * Math.sin(Math.PI * u);
        return [
          foot[0] + stride[legIdx][0] * (u - 0.5),
          foot[1] + stride[legIdx][1] * (u - 0.5),
          foot[2] + lift,
        ];
      }

      // Stance: continuous slide from +stride/2 (just touched down) to
      // -stride/2 (about to lift) over the FULL stance duration. Compute
      // elapsed-since-swing-end with a wrap so the slide doesn't reset at
      // the cycle boundary.
      const elapsed = (t >= swingEndMs)
        ? t - swingEndMs
        : t + (cycleMs - swingEndMs);
      const u = elapsed / stanceMs;
      return [
        foot[0] + stride[legIdx][0] * (0.5 - u),
        foot[1] + stride[legIdx][1] * (0.5 - u),
        foot[2],
      ];
    });
    this._applyImuTilt(feet);
    return this._solveAndCache(feet);
  }

  // Per-leg body-frame displacement over one full gait cycle. Yaw contribution
  // is the cross product of yaw-rate with the leg's hip position, so legs on
  // opposite sides of the body sweep in opposite directions on a turn.
  _strideForCycle(cycleSeconds) {
    const { x, y, yaw } = this.twist;
    const offsets = [];
    for (let i = 0; i < 4; i++) {
      const yawDx = -yaw * LEG_ORIGINS[1][i];
      const yawDy =  yaw * LEG_ORIGINS[0][i];
      offsets.push([
        (x + yawDx) * cycleSeconds,
        (y + yawDy) * cycleSeconds,
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
