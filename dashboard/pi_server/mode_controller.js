// Mode controller — single owner of robot mode state.
//
// Drives the gait planner's 50 Hz tick when an auto mode is active, and
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
import { fourLegsInverseKinematics } from "../shared/argos_kinematics.js";
import { applyServoCal, clampServoDeg } from "../shared/servo_cal.js";
import { GaitPlanner, clampFootInReach, _internals as gaitInternals } from "./gait_planner.js";

const TICK_MS = 1000 / gaitInternals.TICK_HZ;

const AUTO_MODES = new Set(["crouch", "stand", "extend", "crawl", "trot"]);
const DIRECT_MODES = new Set(["direct_foot_xyz", "direct_joint_angles", "direct_servo_angles"]);

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
      this._publishServoDeg(pose.servoAnglesDeg, pose.jointAnglesRad);
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
    const servoDeg = jointAnglesRadToServoDeg(anglesRad);
    this._publishServoDeg(servoDeg, anglesRad);
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
    this._publishServoDeg(out.servoAnglesDeg, out.jointAnglesRad);
  }

  _publishServoDeg(servoDeg12, jointAnglesRad12) {
    this.lastJointStatesDeg = servoDeg12;
    this.lastJointAnglesRad = jointAnglesRad12;
    try {
      this.serial.setAllServoAnglesDeg(servoDeg12);
    } catch (err) {
      this.emit("error", err.message || String(err));
      return;
    }
    this.emit("joint_states", {
      name: [...JOINT_NAMES],
      position: [...jointAnglesRad12],
      position_servo_deg: [...servoDeg12],
    });
  }

  async _resolveModePose(mode) {
    if (mode === "stand" || mode === "extend" || mode === "crouch") {
      // Prefer persisted named stances, but always fall back to the planner so
      // switching into a pose publishes immediately even on a fresh install.
      const stances = await this.getStances();
      const saved = stances[mode];
      if (saved && Array.isArray(saved.angles_rad) && saved.angles_rad.length === 12) {
        return {
          jointAnglesRad: saved.angles_rad,
          servoAnglesDeg: jointAnglesRadToServoDeg(saved.angles_rad),
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
