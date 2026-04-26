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
import { GaitPlanner, _internals as gaitInternals } from "./gait_planner.js";

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
    } else if (mode === "stand" || mode === "extend" || mode === "crouch") {
      // Try saved stance first; if missing, fall back to the planner.
      const stances = await this.getStances();
      const saved = stances[mode];
      if (saved && Array.isArray(saved.angles_rad) && saved.angles_rad.length === 12) {
        const servoDeg = jointAnglesRadToServoDeg(saved.angles_rad);
        this._publishServoDeg(servoDeg, saved.angles_rad);
      }
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
    });
  }

  // dashboard `foot_targets` command. targets = 4×3 matrix in body frame, m.
  async setFootTargets(targets) {
    if (this.mode !== "direct_foot_xyz") await this.setMode("direct_foot_xyz");
    const ik = fourLegsInverseKinematics(targets, { limits: DEFAULT_JOINT_LIMITS_RAD });
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
  setJointLimits(limitsByName) {
    const byLeg = {};
    for (const leg of LEG_IDS) byLeg[leg] = {};
    for (const [name, range] of Object.entries(limitsByName)) {
      const [leg, joint] = name.split("_");
      if (!LEG_IDS.includes(leg) || !JOINT_ROWS.includes(joint)) continue;
      byLeg[leg][joint] = [range.min_deg, range.max_deg];
    }
    for (const leg of LEG_IDS) {
      const femur = byLeg[leg].femur || [-180, 180];
      const tibia = byLeg[leg].tibia || [-180, 180];
      this.serial.setLegJointLimits(leg, { femur, tibia });
    }
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
    out[i] = clampServoDeg(name, applyServoCal(name, jointAnglesRad[i]));
  }
  return out;
}

function num(v, fallback) {
  return Number.isFinite(v) ? v : fallback;
}

export const _internals = { AUTO_MODES, DIRECT_MODES, jointAnglesRadToServoDeg };
