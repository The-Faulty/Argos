// Mode controller — single owner of robot mode state.
//
// Drives the gait planner's auto tick when an auto mode is active, and
// passes through direct-mode commands (foot xyz, joint angles, raw servo
// degrees) straight to the serial bridge. Decoupling lives here so the
// HTTP/WS server doesn't need to know about gait timing or IK.

import { EventEmitter } from "node:events";

import {
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  MODE_OPTIONS,
  DEFAULT_JOINT_LIMITS_RAD,
} from "../shared/robot-config.js";
import { clamp_joint_matrix, fourLegsInverseKinematics } from "../shared/argos_kinematics.js";
import { applyServoCal, clampServoDeg } from "../shared/servo_cal.js";
import { GaitPlanner, clampFootInReach, _internals as gaitInternals } from "./gait_planner.js";

const TICK_MS = 1000 / gaitInternals.TICK_HZ;

const AUTO_MODES = new Set(["crouch", "stand", "extend", "crawl", "trot"]);
const DIRECT_MODES = new Set(["direct_foot_xyz", "direct_joint_angles", "direct_servo_angles"]);
const BACKLASH_COMP_MODES = new Set(["crouch", "stand", "extend", "crawl", "trot", "direct_foot_xyz"]);
const BACKLASH_DEADBAND_DEG = Number(process.env.ARGOS_BACKLASH_DEADBAND_DEG ?? 0.12);
const BACKLASH_COMP_DEG = {
  coxa:  Number(process.env.ARGOS_BACKLASH_COXA_DEG ?? 0.4),
  femur: Number(process.env.ARGOS_BACKLASH_FEMUR_DEG ?? 1.8),
  tibia: Number(process.env.ARGOS_BACKLASH_TIBIA_DEG ?? 2.4),
};
// Skip the directional overshoot for legs in swing phase. The lift profile
// reverses sign mid-swing (peak at u=0.5), so the per-tick servo delta flips
// direction there and triggers a 1.8°/2.4° kick on femur/tibia at the top of
// every step — visible on bench as foot drag at swing entry/exit and a tipping
// moment that destabilizes crawl.
const SKIP_SWING_BACKLASH = (process.env.ARGOS_BACKLASH_SKIP_SWING ?? "1") !== "0";
// Also skip the comp for stance-phase legs when running a periodic gait.
// Backlash comp adds a 1.8°/2.4° kick every time a joint reverses direction;
// stance phases of trot/crawl reverse twice per cycle (push-down ramp peak +
// stride sweep handoff), and stacking that kick on top of an already-moving
// servo command shows up as visible tick-rate jitter even though the
// underlying gait math is smooth. Static modes (stand/extend/crouch) and
// direct_foot_xyz still benefit from the comp — those are the regimes
// where servo backlash makes a held pose visibly droop.
const SKIP_STANCE_BACKLASH_GAIT_MODES = new Set(["crawl", "trot"]);
const SKIP_STANCE_BACKLASH = (process.env.ARGOS_BACKLASH_SKIP_STANCE ?? "1") !== "0";

export class ModeController extends EventEmitter {
  constructor({ serial, persistence, getStances }) {
    super();
    if (!serial) throw new Error("ModeController requires a SerialBridge instance");
    this.serial = serial;
    this.persistence = persistence; // { saveServoSpeed, ... } or null
    this.getStances = getStances || (async () => ({}));
    this.planner = new GaitPlanner();
    this.mode = "idle";
    this.lastJointStatesDeg = new Array(JOINT_NAMES.length).fill(90); // servo degrees
    this.lastJointAnglesRad = new Array(JOINT_NAMES.length).fill(0);
    this._lastDesiredServoDeg = new Array(JOINT_NAMES.length).fill(90);
    this._backlashDir = new Array(JOINT_NAMES.length).fill(0);
    this._tickHandle = null;
  }

  start() {
    if (this._tickHandle) return;
    this._tickHandle = setInterval(() => this._tick(), TICK_MS);
  }

  stop() {
    if (this._tickHandle) {
      clearInterval(this._tickHandle);
      this._tickHandle = null;
    }
  }

  // ─── Public API the server calls ─────────────────────────────────────

  async setMode(mode) {
    if (!MODE_OPTIONS.includes(mode)) throw new Error(`Unknown mode: ${mode}`);
    this.mode = mode;
    this.planner.setMode(mode);
    this.emit("mode", mode);

    if (mode === "idle") {
      this.serial.releaseServos();
      return;
    }

    const pose = await this._resolveModePose(mode);
    if (pose) {
      this._publishServoDeg(pose.servoAnglesDeg, pose.jointAnglesRad, pose.phaseTag);
    }
  }

  async runStartupPoseSequence({
    standMs = 900,
    crouchMs = 700,
  } = {}) {
    const wasTicking = Boolean(this._tickHandle);
    if (wasTicking) this.stop();
    try {
      await this.setMode("stand");
      if (standMs > 0) await sleep(standMs);
      await this.setMode("crouch");
      if (crouchMs > 0) await sleep(crouchMs);
    } finally {
      if (wasTicking) this.start();
    }
  }

  setTwist(twist) {
    this.planner.setTwist(twist);
  }

  setImu(imu) {
    this.planner.setImu(imu);
  }

  updatePlannerConfig(patch) {
    this.planner.updateConfig(patch);
  }

  updateStabilizerParams(params) {
    if (!params || typeof params !== "object") return;
    this.planner.updateConfig({
      stabilization_roll_gain:  num(params.stabilization_roll_gain,  this.planner.config.stabilization_roll_gain),
      stabilization_pitch_gain: num(params.stabilization_pitch_gain, this.planner.config.stabilization_pitch_gain),
      stabilization_max_correction_rad: num(params.stabilization_max_correction_rad, this.planner.config.stabilization_max_correction_rad),
      imu_filter_alpha: num(params.imu_filter_alpha, this.planner.config.imu_filter_alpha),
      use_imu_stabilization:
        typeof params.use_imu_stabilization === "boolean"
          ? params.use_imu_stabilization
          : this.planner.config.use_imu_stabilization,
    });
  }

  // dashboard `foot_targets` command. targets = 4×3 matrix in body frame, m.
  async setFootTargets(targets) {
    if (this.mode !== "direct_foot_xyz") await this.setMode("direct_foot_xyz");
    // LegDetail's foot drag emits raw cursor-derived targets, which can
    // easily land outside the per-leg IK reach window. Without clamping,
    // IK refuses and the drag has no visible effect (the user's complaint).
    // Clamp x into the per-leg window so the leg tracks the cursor along
    // the boundary instead of refusing the whole frame.
    const clamped = targets.map((t, i) => clampFootInReach(t, LEG_IDS[i]));
    const ik = fourLegsInverseKinematics(clamped, { limits: this.planner.jointLimitsRad });
    if (!ik.ok) {
      this.emit("error", `IK failed for ${ik.leg}: ${ik.reason}`);
      return;
    }
    const flat = flattenJoints(ik.angles);
    const servoDeg = jointAnglesRadToServoDeg(flat);
    this._publishServoDeg(servoDeg, flat);
  }

  // dashboard `joint_angles` command. angles = 12-element array, radians.
  async setJointAngles(anglesRad) {
    if (this.mode !== "direct_joint_angles") await this.setMode("direct_joint_angles");
    if (!Array.isArray(anglesRad) || anglesRad.length !== 12) {
      this.emit("error", "joint_angles requires 12 numbers");
      return;
    }
    const clamped = clampFlatJointAnglesRad(anglesRad, this.planner.jointLimitsRad);
    const servoDeg = jointAnglesRadToServoDeg(clamped);
    this._publishServoDeg(servoDeg, clamped);
  }

  // dashboard `servo_angles` command. anglesDeg = 12-element array, 0..180.
  async setServoAnglesDeg(anglesDeg) {
    if (this.mode !== "direct_servo_angles") await this.setMode("direct_servo_angles");
    if (!Array.isArray(anglesDeg) || anglesDeg.length !== 12) {
      this.emit("error", "servo_angles requires 12 numbers");
      return;
    }
    const clamped = anglesDeg.map((deg, i) => clampServoDeg(JOINT_NAMES[i], deg));
    this._publishServoDeg(clamped, this.lastJointAnglesRad);
  }

  // dashboard `raw_servo_angles` — same as setServoAnglesDeg but skips the
  // servo_cal min/max clamp so the limit-finder script can probe past the
  // configured bench range. Still pinned to the physical 0..180 deg horn
  // domain. Firmware-side joint limits still apply; widen them via
  // setJointLimits before running this.
  async setRawServoAnglesDeg(anglesDeg) {
    if (this.mode !== "direct_servo_angles") await this.setMode("direct_servo_angles");
    if (!Array.isArray(anglesDeg) || anglesDeg.length !== 12) {
      this.emit("error", "raw_servo_angles requires 12 numbers");
      return;
    }
    const pinned = anglesDeg.map((deg) => Math.min(180, Math.max(0, deg)));
    this._publishServoDeg(pinned, this.lastJointAnglesRad);
  }

  setServoSpeedLimit(speedByRow) {
    // speedByRow = { coxa, femur, tibia } in deg/s
    for (const legId of LEG_IDS) {
      this.serial.setLegServoSpeedLimit(legId, speedByRow);
    }
  }

  setServoUpdateRate(hz) {
    this.serial.setServoUpdateRate(hz);
  }

  // limitsByName = { "FR_coxa_joint": {min_deg, max_deg}, ... }
  // The dashboard drops entries matching defaults to keep the persisted file
  // small, so the request body is partial. Fill in defaults for any leg/joint
  // not in the override set BEFORE deriving firmware sends or the planner
  // envelope — otherwise unchanged legs would either get [-180, 180] (wiping
  // firmware safety) or be skipped entirely from the intersection (so a
  // single-leg edit would silently move the planner envelope across all 4
  // legs).
  setJointLimits(limitsByName) {
    const RAD2DEG = 180.0 / Math.PI;
    const DEG2RAD = Math.PI / 180.0;
    const defaultsDeg = {
      coxa:  [DEFAULT_JOINT_LIMITS_RAD.coxa[0]  * RAD2DEG, DEFAULT_JOINT_LIMITS_RAD.coxa[1]  * RAD2DEG],
      femur: [DEFAULT_JOINT_LIMITS_RAD.femur[0] * RAD2DEG, DEFAULT_JOINT_LIMITS_RAD.femur[1] * RAD2DEG],
      tibia: [DEFAULT_JOINT_LIMITS_RAD.tibia[0] * RAD2DEG, DEFAULT_JOINT_LIMITS_RAD.tibia[1] * RAD2DEG],
    };
    const byLeg = {};
    for (const leg of LEG_IDS) {
      byLeg[leg] = {
        coxa:  [defaultsDeg.coxa[0],  defaultsDeg.coxa[1]],
        femur: [defaultsDeg.femur[0], defaultsDeg.femur[1]],
        tibia: [defaultsDeg.tibia[0], defaultsDeg.tibia[1]],
      };
    }
    for (const [name, range] of Object.entries(limitsByName || {})) {
      const [leg, joint] = name.split("_");
      if (!LEG_IDS.includes(leg) || !JOINT_ROWS.includes(joint)) continue;
      byLeg[leg][joint] = [range.min_deg, range.max_deg];
    }
    for (const leg of LEG_IDS) {
      this.serial.setLegJointLimits(leg, {
        femur: byLeg[leg].femur,
        tibia: byLeg[leg].tibia,
      });
    }
    const envelope = {};
    for (const row of JOINT_ROWS) {
      let lo = -Infinity, hi = +Infinity;
      for (const leg of LEG_IDS) {
        lo = Math.max(lo, byLeg[leg][row][0]);
        hi = Math.min(hi, byLeg[leg][row][1]);
      }
      if (lo <= hi) envelope[row] = [lo * DEG2RAD, hi * DEG2RAD];
    }
    this.planner.setJointLimits(envelope);
  }

  // Snapshot used by the WS broadcast on each new client.
  getSnapshot() {
    return {
      mode: this.mode,
      joint_states: {
        name: [...JOINT_NAMES],
        position: [...this.lastJointAnglesRad],
        position_servo_deg: [...this.lastJointStatesDeg],
      },
    };
  }

  // ─── Internal ────────────────────────────────────────────────────────

  _tick() {
    if (!AUTO_MODES.has(this.mode)) return;
    const out = this.planner.step();
    if (!out) return;
    if (out.error) this.emit("error", out.error);
    this._publishServoDeg(out.servoAnglesDeg, out.jointAnglesRad, out.phaseTag);
  }

  _publishServoDeg(servoDeg12, jointAnglesRad12, phaseTag) {
    const commandedServoDeg = this._applyBacklashCompensation(servoDeg12, phaseTag);
    this.lastJointStatesDeg = commandedServoDeg;
    this.lastJointAnglesRad = jointAnglesRad12;
    try {
      this.serial.setAllServoAnglesDeg(commandedServoDeg);
    } catch (err) {
      this.emit("error", err.message || String(err));
      return;
    }
    this.emit("joint_states", {
      name: [...JOINT_NAMES],
      position: [...jointAnglesRad12],
      position_servo_deg: [...commandedServoDeg],
    });
  }

  _applyBacklashCompensation(desiredServoDeg, phaseTag) {
    const desired = desiredServoDeg.slice();
    if (!BACKLASH_COMP_MODES.has(this.mode)) {
      this._lastDesiredServoDeg = desired;
      return desired;
    }

    const out = desired.slice();
    for (let i = 0; i < desired.length; i++) {
      const row = JOINT_ROWS[i % JOINT_ROWS.length];
      const comp = BACKLASH_COMP_DEG[row] ?? 0;
      if (!(comp > 0)) continue;

      const delta = desired[i] - this._lastDesiredServoDeg[i];
      if (Math.abs(delta) > BACKLASH_DEADBAND_DEG) {
        this._backlashDir[i] = Math.sign(delta);
      }
      // Direction tracking still runs for swing legs so the first tick after
      // touchdown has a correct delta history; only the overshoot output is
      // suppressed.
      const legPhase = phaseTag ? phaseTag[Math.floor(i / 3)] : "stance";
      if (SKIP_SWING_BACKLASH && legPhase === "swing") continue;

      const dir = this._backlashDir[i];
      if (dir) out[i] = clampServoDeg(JOINT_NAMES[i], desired[i] + dir * comp);
    }
    this._lastDesiredServoDeg = desired;
    return out;
  }

  async _resolveModePose(mode) {
    if (mode === "stand" || mode === "extend" || mode === "crouch") {
      // Prefer persisted named stances, but always fall back to the planner so
      // switching into a pose publishes immediately even on a fresh install.
      const stances = await this.getStances();
      const saved = stances[mode];
      if (saved && Array.isArray(saved.angles_rad) && saved.angles_rad.length === 12) {
        const clamped = clampFlatJointAnglesRad(saved.angles_rad, this.planner.jointLimitsRad);
        return {
          jointAnglesRad: clamped,
          servoAnglesDeg: jointAnglesRadToServoDeg(clamped),
        };
      }
    }
    if (!AUTO_MODES.has(mode)) return null;
    const out = this.planner.step();
    if (!out) return null;
    if (out.error) this.emit("error", out.error);
    return out;
  }
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

function unflattenJoints(angles12) {
  const out = [];
  for (let li = 0; li < 4; li++) {
    out.push(angles12.slice(li * 3, li * 3 + 3));
  }
  return out;
}

function clampFlatJointAnglesRad(angles12, limits = DEFAULT_JOINT_LIMITS_RAD) {
  return flattenJoints(clamp_joint_matrix(unflattenJoints(angles12), 0.0, limits));
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function jointAnglesRadToServoDeg(jointAnglesRad) {
  const out = new Array(JOINT_NAMES.length);
  for (let i = 0; i < JOINT_NAMES.length; i++) {
    const name = JOINT_NAMES[i];
    out[i] = clampServoDeg(name, applyServoCal(name, jointAnglesRad[i]));
  }
  return out;
}

function num(v, fallback) {
  return Number.isFinite(v) ? v : fallback;
}

export const _internals = { AUTO_MODES, DIRECT_MODES, jointAnglesRadToServoDeg };
