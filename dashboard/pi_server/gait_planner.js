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
  DEFAULT_FOOT_REACH_X_LIFTED,
  STABILIZER_PARAM_BOUNDS,
  GAIT_TUNABLE_PARAMS,
} from "../shared/robot-config.js";
import {
  fourLegsInverseKinematics,
  leg_explicit_inverse_kinematics,
  resetIkHint,
} from "../shared/argos_kinematics.js";
import { applyServoCal, clampServoDeg } from "../shared/servo_cal.js";

// Back to 50 Hz once the firmware fix landed. Earlier we had to drop to 30 Hz
// because each gait tick emits 4× set_leg_servo_angles (~480 B) at 921600
// baud and the ESP32 RX FIFO was overflowing mid-frame, dropping commands.
// The actual root cause turned out to be TX-induced RX starvation: the old
// chained-Serial.print() sendStateMessage() in firmware/argos_servo.ino was
// making ~160 driver hand-offs per state burst, starving the RX driver task
// long enough for the FIFO to overflow. The Pi still sends four leg frames per
// gait tick, and field logs showed truncated `set_leg_servo_angles` frames at
// 50 Hz. Keep auto gait at a serial-friendly rate; the firmware's own PWM
// refresh still smooths the movement between these setpoints.
const TICK_HZ = 25.0;
const TICK_DT = 1.0 / TICK_HZ;
const REACH_BLEND_STEPS = 12;
// Lift profile exponent on sin(πu). 0.65 was a "broad plateau" choice but
// the steep slopes near u=0 and u=1 demanded tibia angle changes >25°/tick
// at the swing-start lift-off, which the servo couldn't track (cap is 14.4°
// at 360°/s × 40 ms tick), so the visible peak got throttled to ~12 mm even
// at z_clearance = 22 mm. Exponent 2 (sin² profile) has zero slope at u=0
// and u=1 and concentrates the lift near u=0.5 — peak duration is shorter
// but per-tick demand stays inside the servo budget, so the foot actually
// reaches the commanded peak height.
const SWING_LIFT_PROFILE_EXP = 2.0;

const DEG2RAD = Math.PI / 180.0;

// Stand / extend pose joints. We bypass IK for these because the abductor's
// natural lateral offset means foot.y = hip.y can't be reached with coxa = 0,
// and emitting joints directly puts coxa at exactly 0 (servo 90°) so STAND
// and a zero-twist TROT/CRAWL look identical. STAND_JOINTS_RAD must match
// the IK-FK output at the gait neutral foot — STAND_FRONT_FOOT_X /
// STAND_REAR_FOOT_X / STAND_FOOT_Y / STAND_FOOT_Z below — otherwise switching
// from stand → trot snaps every leg to a different pose.
//
// CRITICAL: the IK has two valid bell-crank branches at this footprint
// (tibia ≈ +20° vs tibia ≈ +29° at z=-0.195, both yield the same foot XYZ).
// The physical linkage is calibrated to the +20° branch — earlier we tried
// the +29° branch by mistake and the robot walked backward. Stay on the
// +20° branch. Re-derive with `node scripts/probe_neutral_pose.mjs` after
// any stance change and verify tibia < 22°.
//
// Current values: front foot at (0.112, ±0.0733, -0.195), rear at
// (-0.111, ±0.0733, -0.195) — re-centered on the IK reach window so the
// stride budget is symmetric and no clampFootInReach pin is ever hit.
const STAND_JOINTS_RAD  = [0, -25.99 * DEG2RAD,  19.61 * DEG2RAD];
const EXTEND_JOINTS_RAD = [0, -10.00 * DEG2RAD,   8.00 * DEG2RAD];

// Stride amplitude clamp (meters, peak displacement either side of base).
// At the current body height (STAND_FOOT_Z = -0.195 m) and re-centered
// neutral feet (front 0.112, rear -0.111), each leg's stance reach
// window (DEFAULT_FOOT_REACH_X) is 72 mm wide and the per-side no-pin
// budget is 36 mm. 0.040 trades a 4 mm symmetric pin at both stance
// edges for a visibly bigger swing arc (80 mm peak-to-peak vs 68 mm),
// which is what the operator was asking for. The pin produces a small
// jerk at the swing→stance handoff but is well below the 21 mm pin the
// historical config (base 0.100, MAX_STRIDE_X 0.045) had — and that
// config walked, just unevenly. Stays on the validated bell-crank
// branch (tibia ~+20°) — see STAND_JOINTS_RAD comment.
const MAX_STRIDE_X = 0.040;
const MAX_STRIDE_Y = 0.025;

// Gait profiles. Each leg gets one swing window per cycle; the rest is
// stance. Phase offsets are in FR/FL/RR/RL order. Trot moves diagonal pairs;
// crawl is a conservative four-beat sequence with one leg lifted at a time.
const GAIT_PROFILES = {
  trot: {
    swingFraction: 0.34,
    phaseOffsets: [0.00, 0.50, 0.50, 0.00],
  },
  crawl: {
    swingFraction: 0.22,
    phaseOffsets: [0.75, 0.25, 0.50, 0.00],
  },
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
    return { servoAnglesDeg, jointAnglesRad, feet: makeStanceFeet(this.config), phaseTag: allStanceTag() };
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
    return { servoAnglesDeg, jointAnglesRad, feet: makeStanceFeet(this.config), phaseTag: allStanceTag() };
  }

  _gaitPose(mode) {
    const timing = gaitProfileTiming(this.config, mode);
    if (!timing) return null;
    const { cycleMs, stanceMs, swingFraction, phaseOffsets } = timing;

    // Zero twist → freeze the gait clock and emit the neutral footprint.
    // Without this, gaitTickMs keeps advancing and swing legs still lift to
    // z_clearance even with no commanded motion, so the robot trotted in
    // place after the joystick was released. The mode stays trot/crawl so
    // the next non-zero stick push resumes immediately.
    if (this.twist.x === 0 && this.twist.y === 0 && this.twist.yaw === 0) {
      this.gaitTickMs = 0;
      const baseFeet = makeStanceFeet(this.config);
      this._applyImuTilt(baseFeet);
      // All four feet are on the ground at neutral pose → stance window.
      const clamped = baseFeet.map((f, i) => clampFootInReach(f, LEG_IDS[i], "stance", this.jointLimitsRad));
      return this._solveAndCache(clamped, { lockCoxa: true, phaseTag: allStanceTag() });
    }
    this.gaitTickMs = (this.gaitTickMs + TICK_DT * 1000) % cycleMs;

    // Stride amplitude during stance is body_velocity × stance_time (foot
    // must travel that far backward in body frame to keep ground contact
    // while the body advances). Using cycle time instead overshoots by the
    // duty-cycle ratio — at max twist + max swing it produced 0.45 m
    // strides, far outside the ~75 mm IK reach window. _strideForPeriod
    // also clamps each axis so unreachable joystick inputs degrade
    // gracefully instead of stalling the planner on every tick.
    const stride = this._strideForPeriod(stanceMs / 1000);
    const baseFeet = makeStanceFeet(this.config);

    // Per-leg phase tag so clampFootInReach can pick the right reach window:
    // stance feet stay inside the tighter stance window; swing feet get the
    // wider lifted window (the linkage extends at peak lift, so the
    // reachable foot.x range shifts/widens). Computed in the same loop as
    // the position so the two stay in lock-step.
    const phaseTag = new Array(4);

    const feet = baseFeet.map((foot, legIdx) => {
      const phase = ((this.gaitTickMs / cycleMs) + phaseOffsets[legIdx]) % 1.0;
      const inSwing = phase < swingFraction;

      if (inSwing) {
        // Swing: -stride/2 -> +stride/2 with a smooth fore/aft transfer and
        // sin-arc lift (peak at u=0.5). Using the true swing duration here
        // keeps the operator's "step duration" setting literal.
        const u = phase / swingFraction;
        const eased = smoothStep(u);
        const lift = this.config.z_clearance * swingLiftProfile(u);
        phaseTag[legIdx] = "swing";
        return [
          foot[0] + stride[legIdx][0] * (eased - 0.5),
          foot[1] + stride[legIdx][1] * (eased - 0.5),
          foot[2] + lift,
        ];
      }

      // Stance: continuous slide from +stride/2 (just touched down) to
      // -stride/2 (about to lift) over the true stance duration.
      // Push-down: foot dips below stance Z at mid-stance (u=0.5) — in
      // body frame the foot goes deeper, which means the body rises (the
      // leg pushes against the ground). Without this the gait just slides
      // the foot back instead of propelling the body forward. Capped at
      // 0.003 m because the IK floor at the current stance is ~z=-0.198;
      // 5 mm of push hits IK refuse at body-frame z=-0.200.
      const u = (phase - swingFraction) / (1.0 - swingFraction);
      const push = this.config.stance_push_down * Math.sin(Math.PI * u);
      phaseTag[legIdx] = "stance";
      return [
        foot[0] + stride[legIdx][0] * (0.5 - u),
        foot[1] + stride[legIdx][1] * (0.5 - u),
        foot[2] - push,
      ];
    });
    this._applyImuTilt(feet);
    // Phase-aware reach clamp: stance feet against the tight stance window,
    // swing feet against the wider lifted window. Without the phase split,
    // every foot was clamped to the stance window and swing trajectories at
    // the edges of the stride got snapped back toward neutral, shrinking
    // the visible step amplitude even though the lifted geometry could
    // reach the target. Re-probe both windows with
    // scripts/probe_foot_reach.mjs after any z_clearance change.
    const clamped = feet.map((f, i) => clampFootInReach(f, LEG_IDS[i], phaseTag[i], this.jointLimitsRad));
    const lockCoxa = Math.abs(this.twist.y) < 1e-6 && Math.abs(this.twist.yaw) < 1e-6;
    return this._solveAndCache(clamped, { lockCoxa, phaseTag });
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

  _solveAndCache(feet, { lockCoxa = false, phaseTag = allStanceTag() } = {}) {
    const ik = fourLegsInverseKinematics(feet, { limits: this.jointLimitsRad });
    if (!ik.ok) {
      // Refuse to publish bad IK — return last good output.
      return {
        servoAnglesDeg: this.lastServoAnglesDeg,
        jointAnglesRad: this.lastJointAnglesRad,
        feet,
        phaseTag,
        error: `IK failed for ${ik.leg}: ${ik.reason}`,
      };
    }
    const jointAnglesRad = flattenJoints(ik.angles);
    if (lockCoxa) {
      for (let i = 0; i < 4; i++) jointAnglesRad[i * 3] = 0.0;
    }
    const servoAnglesDeg = jointAnglesRadToServoDeg(jointAnglesRad);
    this.lastJointAnglesRad = jointAnglesRad;
    this.lastServoAnglesDeg = servoAnglesDeg;
    this.feet = feet;
    return { servoAnglesDeg, jointAnglesRad, feet, phaseTag };
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
    // Swing-foot vertical lift. At the current body height (STAND_FOOT_Z =
    // -0.195 m), the IK ceiling is ~0.023 m before the lifted reach
    // window collapses. With the sin² lift profile the per-tick servo
    // demand stays inside the 14.4°/tick tibia cap (= 360°/s × 40 ms),
    // so the foot actually reaches the commanded peak — under the older
    // sin^0.65 profile the steep slope at u=0 spiked tibia demand to
    // 28°/tick at swing-start and the visible peak got throttled to
    // ~12 mm even when commanded 22 mm.
    //
    // If you change this value, also re-probe DEFAULT_FOOT_REACH_X_LIFTED
    // in robot-config.js with `node scripts/probe_foot_reach.mjs --z-mm
    // <stance_z_mm> --lift-mm <new_z_clearance_mm>` and update both
    // numbers together.
    z_clearance: 0.022,
    // Stance push-down (m). At u=0.5 of stance the foot dips below
    // stance_z by this amount, which in body frame means the body rises
    // — the leg is pushing into the ground. This is what propels the
    // body forward; without it the gait just slides the foot back.
    // Bounded above by IK floor at current stance (~3-4 mm before IK
    // refuses at body-frame z=-0.199).
    stance_push_down: 0.003,
    stabilization_roll_gain:  STABILIZER_PARAM_BOUNDS.stabilization_roll_gain.default,
    stabilization_pitch_gain: STABILIZER_PARAM_BOUNDS.stabilization_pitch_gain.default,
    stabilization_max_correction_rad: STABILIZER_PARAM_BOUNDS.stabilization_max_correction_rad.default,
    imu_filter_alpha: STABILIZER_PARAM_BOUNDS.imu_filter_alpha.default,
    // Off by default: body tilt correction spends the same vertical reach
    // budget used by swing lift, so enable only after checking the robot can
    // walk without IK edge hits at the chosen stance height.
    use_imu_stabilization: false,
  };
}

// Foot positions corresponding to STAND_JOINTS_RAD = [0, -25.99°, +19.61°]
// computed by forward-kinematic-ing the linkage (coxa pinned to 0). These
// sit at the per-leg stance reach window centers at body height -0.195 m,
// so MAX_STRIDE_X can use the full symmetric half-width (36 mm) without
// triggering clampFootInReach. Lateral y stays at 0.0733 — coxa=0 can't
// reach the old 0.1106 m without splaying ~11°.
//
// Body height stays at the historical -0.195 m: this is the only z that
// keeps IK on the +20° tibia branch (the validated bell-crank
// configuration) when the stance is re-centered. Raising the body to
// z=-0.194 or higher shifts IK onto the +29° branch even at base.x=0.100
// — see STAND_JOINTS_RAD comment for why that's mechanically wrong.
const STAND_FRONT_FOOT_X =  0.1120;
const STAND_REAR_FOOT_X  = -0.1110;
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

function gaitProfileTiming(cfg, mode) {
  const profile = GAIT_PROFILES[mode];
  if (!profile) return null;
  const swingMs = clampNum(
    cfg?.swing_time_ms ?? GAIT_TUNABLE_PARAMS.swing_time_ms.default,
    GAIT_TUNABLE_PARAMS.swing_time_ms.min,
    GAIT_TUNABLE_PARAMS.swing_time_ms.max,
  );
  const swingFraction = clampNum(profile.swingFraction, 0.05, 0.90);
  const cycleMs = swingMs / swingFraction;
  return {
    swingMs,
    stanceMs: cycleMs - swingMs,
    cycleMs,
    swingFraction,
    phaseOffsets: profile.phaseOffsets,
  };
}

function makeZeroJoints() {
  return new Array(JOINT_NAMES.length).fill(0);
}

function allStanceTag() {
  return ["stance", "stance", "stance", "stance"];
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

// Clamp a foot target into the leg's reachable envelope. Picks the stance
// or lifted reach window based on `phase` — swing-phase trajectories live
// at z = stance + z_clearance where the linkage's reachable foot.x range
// is shifted/widened, so clamping swing feet against the stance window
// would silently shrink step amplitude. After x is clamped, if IK still
// rejects (y/z corner of the envelope), blend that foot back toward its
// neutral stance point until IK accepts. Keeps the gait moving instead of
// freezing on the last-good whole-body servo frame.
//
// `phase` accepts "stance" | "swing"; anything else falls back to stance.
// `limits` defaults to DEFAULT_JOINT_LIMITS_RAD so callers from the
// gait_planner instance can pass `this.jointLimitsRad`.
export function clampFootInReach(foot, legId, phase = "stance", limits = DEFAULT_JOINT_LIMITS_RAD) {
  const reachTable = phase === "swing" ? DEFAULT_FOOT_REACH_X_LIFTED : DEFAULT_FOOT_REACH_X;
  const range = reachTable[legId];
  if (!range) return foot;
  const candidate = [clampNum(foot[0], range[0], range[1]), foot[1], foot[2]];
  if (isFootReachable(candidate, legId, limits)) return candidate;

  const home = neutralFootForLeg(legId);
  if (!isFootReachable(home, legId, limits)) return home;

  let lo = 0.0;
  let hi = 1.0;
  let best = home;
  for (let i = 0; i < REACH_BLEND_STEPS; i++) {
    const mid = 0.5 * (lo + hi);
    const blended = blendFoot(candidate, home, mid);
    if (isFootReachable(blended, legId, limits)) {
      best = blended;
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return best;
}

function neutralFootForLeg(legId) {
  const idx = LEG_IDS.indexOf(legId);
  return idx >= 0 ? DEFAULT_NEUTRAL_FEET[idx] : [0, 0, DEFAULT_Z_REF];
}

function isFootReachable(foot, legId, limits) {
  const idx = LEG_IDS.indexOf(legId);
  if (idx < 0) return false;
  const rLeg = [
    foot[0] - LEG_ORIGINS[0][idx],
    foot[1] - LEG_ORIGINS[1][idx],
    foot[2] - LEG_ORIGINS[2][idx],
  ];
  return leg_explicit_inverse_kinematics(rLeg, idx, { limits }) !== null;
}

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function smoothStep(t) {
  const u = clampNum(t, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

function swingLiftProfile(t) {
  const s = Math.sin(Math.PI * clampNum(t, 0.0, 1.0));
  return Math.pow(Math.max(0.0, s), SWING_LIFT_PROFILE_EXP);
}

function blendFoot(from, to, t) {
  return [
    lerp(from[0], to[0], t),
    lerp(from[1], to[1], t),
    lerp(from[2], to[2], t),
  ];
}

const DEFAULT_NEUTRAL_FEET = makeStanceFeet(makeDefaultConfig());

export const _internals = {
  TICK_HZ,
  TICK_DT,
  GAIT_PROFILES,
  gaitProfileTiming,
  makeStanceFeet,
  jointAnglesRadToServoDeg,
  swingLiftProfile,
};
