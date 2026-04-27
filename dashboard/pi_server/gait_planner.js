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
  DEFAULT_FOOT_REACH_X,
  STABILIZER_PARAM_BOUNDS,
  GAIT_TUNABLE_PARAMS,
} from "../shared/robot-config.js";
import { fourLegsInverseKinematics, resetIkHint } from "../shared/argos_kinematics.js";
import { applyServoCal, clampServoDeg } from "../shared/servo_cal.js";

// Dropped from 50 Hz to 30 Hz: each gait tick emits 4× set_leg_servo_angles
// (~480 B) at 921600 baud. The ESP32's 32 B UART RX FIFO drains slowly when
// the firmware is mid-I²C-write to the PCA9685, so 50 Hz bursts reproducibly
// truncated frames ("unknown command type" with corrupted bodies). 30 Hz
// gives the firmware ~33 ms between bursts to drain — empirically enough.
const TICK_HZ = 30.0;
const TICK_DT = 1.0 / TICK_HZ;

const DEG2RAD = Math.PI / 180.0;

// Stand / extend pose joints. We bypass IK for these because the abductor's
// natural lateral offset means foot.y = hip.y can't be reached with coxa = 0,
// and the femur joint limits ([-50°, 0°]) make the IK reach window only
// ~75 mm wide at the historical body height. Emitting joints directly puts
// coxa at exactly 0 (servo 90°) and lets us pick body height by choosing
// femur/tibia, with no risk of the planner silently holding stale output
// because IK refused a borderline target.
const STAND_JOINTS_RAD  = [0, -28.84 * DEG2RAD,  15.80 * DEG2RAD];
const EXTEND_JOINTS_RAD = [0, -10.00 * DEG2RAD,   8.00 * DEG2RAD];

// Stride amplitude clamp (meters, peak displacement either side of base).
// The current stance has roughly 70 mm of safe body-x travel per foot. Use
// most of that window so trot reads as a deliberate step instead of a tiny
// shuffle; clampFootInReach remains the final guardrail at the hard edges.
// Lateral reach is much tighter, so y stays deliberately conservative.
const MAX_STRIDE_X = 0.040;
const MAX_STRIDE_Y = 0.005;

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
    this.jointLimitsRad = cloneLimits(DEFAULT_JOINT_LIMITS_RAD);
  }

  // ─── External knobs ──────────────────────────────────────────────────

  setMode(mode) {
    if (this.mode === mode) return;
    this.mode = mode;
    this.tickCount = 0;
    this.gaitTickMs = 0;
    this.feet = makeStanceFeet(this.config);
    // Drop the IK warm-start hints — each gait/stance has a different
    // feasible neighborhood, and a stale hint from the prior mode can
    // trap a leg in a local minimum on the first solve of the new one.
    resetIkHint();
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

  // Single envelope (coxa/femur/tibia rad triples) applied to every leg's IK.
  // Drop the warm-start hints because the previous solution may sit outside
  // the new envelope — without a reset the next solve would walk the hint
  // back into a window that no longer accepts it and stall on first tick.
  setJointLimits(limitsRad = {}) {
    const merged = cloneLimits(this.jointLimitsRad);
    for (const row of ["coxa", "femur", "tibia"]) {
      if (Array.isArray(limitsRad[row]) && limitsRad[row].length === 2) {
        merged[row] = [limitsRad[row][0], limitsRad[row][1]];
      }
    }
    this.jointLimitsRad = merged;
    resetIkHint();
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
    // Direct joint emission for both stand and extend — see STAND/EXTEND
    // _JOINTS_RAD for why this bypasses IK. The "feet" returned here are
    // the planner's last cached positions (used by the dashboard preview
    // and the fall-back to stance on mode switch); they don't drive the
    // servo command.
    const triple = mode === "stand" ? STAND_JOINTS_RAD : EXTEND_JOINTS_RAD;
    const jointAnglesRad = new Array(JOINT_NAMES.length);
    for (let i = 0; i < 4; i++) {
      jointAnglesRad[i * 3 + 0] = triple[0];
      jointAnglesRad[i * 3 + 1] = triple[1];
      jointAnglesRad[i * 3 + 2] = triple[2];
    }
    const servoAnglesDeg = jointAnglesRadToServoDeg(jointAnglesRad);
    this.lastJointAnglesRad = jointAnglesRad;
    this.lastServoAnglesDeg = servoAnglesDeg;
    return { servoAnglesDeg, jointAnglesRad, feet: makeStanceFeet(this.config) };
  }

  _gaitPose(mode) {
    const phases = CONTACT_PHASES[mode];
    const phaseDur = gaitPhaseTimes(this.config, mode);
    const cycleMs = phaseDur.stance_ms * 3 + phaseDur.swing_ms;

    // Zero twist → freeze the gait clock and emit the neutral footprint.
    // Without this, gaitTickMs keeps advancing and swing legs still lift to
    // z_clearance even with no commanded motion, so the robot trotted in
    // place after the joystick was released. The mode stays trot/crawl so
    // the next non-zero stick push resumes immediately.
    if (this.twist.x === 0 && this.twist.y === 0 && this.twist.yaw === 0) {
      this.gaitTickMs = 0;
      const baseFeet = makeStanceFeet(this.config);
      this._applyImuTilt(baseFeet);
      const clamped = baseFeet.map((f, i) => clampFootInReach(f, LEG_IDS[i]));
      return this._solveAndCache(clamped);
    }
    this.gaitTickMs = (this.gaitTickMs + TICK_DT * 1000) % cycleMs;

    const phaseDurMs = cycleMs / 4;
    const stanceMs = cycleMs - phaseDurMs; // 3 of 4 phases under both contact patterns
    // Stride amplitude during stance is body_velocity × stance_time (foot
    // must travel that far backward in body frame to keep ground contact
    // while the body advances). Using cycle time instead overshoots by the
    // duty-cycle ratio — at max twist + max swing it produced 0.45 m
    // strides, far outside the ~75 mm IK reach window. _strideForPeriod
    // also clamps each axis so unreachable joystick inputs degrade
    // gracefully instead of stalling the planner on every tick.
    const stride = this._strideForPeriod(stanceMs / 1000);
    const baseFeet = makeStanceFeet(this.config);

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
    // The base rear stance foot sits at body x = -0.140 — only 5 mm inside
    // the rear leg's reach lower bound (-0.145). Even modest backward stride
    // (forward joystick) would otherwise push the rear feet just outside
    // their reach window every tick. Snap each foot's x into its per-leg
    // window so swing/stance trajectories degrade to held-at-boundary motion
    // instead of an IK refusal that holds the whole-body last-good pose.
    const clamped = feet.map((f, i) => clampFootInReach(f, LEG_IDS[i]));
    return this._solveAndCache(clamped);
  }

  // Per-leg peak-to-peak foot displacement over one stance window (in body
  // frame). Yaw contribution is the cross product of yaw-rate with the leg's
  // hip position, so legs on opposite sides of the body sweep in opposite
  // directions on a turn. Each axis is clamped to MAX_STRIDE_* so the
  // resulting foot.x stays inside the leg's IK reach window even at max
  // joystick + max swing time.
  _strideForPeriod(periodSeconds) {
    const { x, y, yaw } = this.twist;
    const offsets = [];
    for (let i = 0; i < 4; i++) {
      const yawDx = -yaw * LEG_ORIGINS[1][i];
      const yawDy =  yaw * LEG_ORIGINS[0][i];
      const sx = (x + yawDx) * periodSeconds;
      const sy = (y + yawDy) * periodSeconds;
      offsets.push([
        clampNum(sx, -2 * MAX_STRIDE_X, 2 * MAX_STRIDE_X),
        clampNum(sy, -2 * MAX_STRIDE_Y, 2 * MAX_STRIDE_Y),
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
    const ik = fourLegsInverseKinematics(feet, { limits: this.jointLimitsRad });
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
    // Swing-foot vertical lift. With the stock femur/tibia envelope the
    // reachable z window at the neutral stance is only a few millimeters;
    // larger lifts make mid-swing IK fail and the controller reuses the last
    // good pose, which feels like the joystick walks a few steps then stops.
    // Keep the default conservative so a held joystick keeps producing fresh
    // gait poses. Larger lift should only be enabled after widening limits.
    z_clearance: 0.002,
    stabilization_roll_gain:  STABILIZER_PARAM_BOUNDS.stabilization_roll_gain.default,
    stabilization_pitch_gain: STABILIZER_PARAM_BOUNDS.stabilization_pitch_gain.default,
    stabilization_max_correction_rad: STABILIZER_PARAM_BOUNDS.stabilization_max_correction_rad.default,
    imu_filter_alpha: STABILIZER_PARAM_BOUNDS.imu_filter_alpha.default,
    // Off by default for the same reason z_clearance is small: even a 3°
    // body tilt yields ~3 mm vertical foot displacement at the lateral
    // hip-to-foot offset, which is enough to exit the z reach window every
    // tick. Re-enable from the Settings drawer once the joint envelope is
    // widened (e.g. recalibrated SERVO_LIMITS_DEG) or the gains are tuned
    // small enough that corrections stay under ~1 mm of foot displacement.
    use_imu_stabilization: false,
  };
}

// Foot positions corresponding to STAND_JOINTS_RAD = [0, -28.84°, 15.80°]
// computed by forward-kinematic-ing the linkage (coxa pinned to 0). The old
// default stance had feet at ±0.1106 m laterally, but coxa=0 can't reach
// that — IK was forced to splay legs out by ~+11° on every solve, which is
// why trot looked visibly "kicked out" compared to the stand pose. Anchoring
// to these positions makes IK produce coxa≈0 at neutral, so STAND and a
// zero-twist TROT look identical.
const STAND_FRONT_FOOT_X =  0.1000;
const STAND_REAR_FOOT_X  = -0.1233;
const STAND_FOOT_Y       =  0.0733;
const STAND_FOOT_Z       = -0.1950;

function makeStanceFeet(cfg, zShift = 0) {
  // delta_x_mm is interpreted as a fore/aft offset on top of the STAND
  // anchors so the gait's footprint depth is still tuneable. delta_y_mm is
  // a no-op now; widening it would re-introduce the splayed-coxa stance and
  // narrowing it pushes the IK into the unreachable region near the hip.
  const dxOffset = (cfg?.delta_x ?? GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000)
    - GAIT_TUNABLE_PARAMS.delta_x_mm.default / 1000;
  const dz = (cfg?.default_z_ref ?? DEFAULT_Z_REF) - DEFAULT_Z_REF;
  return LEG_IDS.map((id) => {
    const isFront = id === "FR" || id === "FL";
    const isLeft = id === "FL" || id === "RL";
    const x = isFront ? STAND_FRONT_FOOT_X + dxOffset : STAND_REAR_FOOT_X - dxOffset;
    const y = isLeft ? STAND_FOOT_Y : -STAND_FOOT_Y;
    return [x, y, STAND_FOOT_Z + dz + zShift];
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

function cloneLimits(src) {
  return {
    coxa:  [src.coxa[0],  src.coxa[1]],
    femur: [src.femur[0], src.femur[1]],
    tibia: [src.tibia[0], src.tibia[1]],
  };
}

// Clamp a foot target's x into its leg's IK reach window. Y and Z pass
// through unchanged — only the body-x dimension is the consistent failure
// mode at default stance height. Used by both the gait trajectory and the
// dashboard's direct foot-drag handler so out-of-reach inputs become
// "snap to boundary" instead of "silent IK refusal".
export function clampFootInReach(foot, legId) {
  const range = DEFAULT_FOOT_REACH_X[legId];
  if (!range) return foot;
  return [clampNum(foot[0], range[0], range[1]), foot[1], foot[2]];
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
