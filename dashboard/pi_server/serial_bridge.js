// Serial bridge: Pi (Node) ↔ ESP32 (Arduino sketch in firmware/argos_servo).
//
// Wire format is newline-delimited JSON @ 921600 baud. The firmware accepts
// per-leg commands using the andy-servo conventions (front_left/front_right/
// rear_left/rear_right × hip/thigh/calf). The dashboard speaks FR/FL/RR/RL ×
// coxa/femur/tibia, so this module is the only place those naming systems
// meet — everything outside uses dashboard names.
//
// State telemetry from the firmware is re-emitted on `state` (raw, with
// dashboard-style leg/joint keys) and as a 12-element joint_states snapshot.

import { EventEmitter } from "node:events";
import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";

import {
  LEG_IDS,
  JOINT_NAMES,
  JOINT_ROWS,
  DEFAULT_SERVO_UPDATE_RATE_HZ,
} from "../shared/robot-config.js";

const DASH_TO_FW_LEG = {
  FR: "front_right",
  FL: "front_left",
  RR: "rear_right",
  RL: "rear_left",
};
const FW_TO_DASH_LEG = Object.fromEntries(
  Object.entries(DASH_TO_FW_LEG).map(([k, v]) => [v, k]),
);

// coxa ↔ hip, femur ↔ thigh, tibia ↔ calf
const DASH_TO_FW_JOINT = { coxa: "hip", femur: "thigh", tibia: "calf" };
const FW_TO_DASH_JOINT = Object.fromEntries(
  Object.entries(DASH_TO_FW_JOINT).map(([k, v]) => [v, k]),
);

const DEFAULT_PORT = process.env.ARGOS_SERIAL_PORT || "/dev/ttyUSB0";
const DEFAULT_BAUD = Number(process.env.ARGOS_SERIAL_BAUD || 921600);

export class SerialBridge extends EventEmitter {
  constructor({ path = DEFAULT_PORT, baudRate = DEFAULT_BAUD } = {}) {
    super();
    this.path = path;
    this.baudRate = baudRate;
    this.port = null;
    this.parser = null;
    this.seq = 0;
    this.connected = false;
    this.lastState = null;
    this._reopenTimer = null;
  }

  connect() {
    if (this.port) return;
    const port = new SerialPort({ path: this.path, baudRate: this.baudRate, autoOpen: false });
    const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));
    this.port = port;
    this.parser = parser;

    parser.on("data", (line) => this._onLine(line));
    port.on("open", () => {
      this.connected = true;
      this.emit("open");
      this._send({ type: "hello" });
      this._send({ type: "set_mode", mode: "direct_servo_angles" });
      this._send({ type: "get_state" });
    });
    port.on("close", () => {
      this.connected = false;
      this.emit("close");
      this._scheduleReopen();
    });
    port.on("error", (err) => this.emit("error", err));

    port.open((err) => {
      if (err) {
        this.emit("error", err);
        this._scheduleReopen();
      }
    });
  }

  disconnect() {
    if (this._reopenTimer) {
      clearTimeout(this._reopenTimer);
      this._reopenTimer = null;
    }
    if (this.port && this.port.isOpen) this.port.close();
    this.port = null;
    this.parser = null;
  }

  _scheduleReopen() {
    if (this._reopenTimer) return;
    this._reopenTimer = setTimeout(() => {
      this._reopenTimer = null;
      this.port = null;
      this.connect();
    }, 2000);
  }

  // ─── Outbound commands ────────────────────────────────────────────────

  setMode(mode) {
    this._send({ type: "set_mode", mode });
  }

  releaseServos() {
    this._send({ type: "release_servos" });
  }

  setServoUpdateRate(hz) {
    this._send({ type: "set_servo_update_rate_hz", hz });
  }

  // angles is { coxa, femur, tibia } in 0..180 servo-horn degrees.
  setLegServoAngles(dashLegId, { coxa, femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) throw new Error(`Unknown leg id: ${dashLegId}`);
    this._send({
      type: "set_leg_servo_angles",
      legId,
      hipServoDeg: coxa,
      thighServoDeg: femur,
      calfServoDeg: tibia,
    });
  }

  // 12-element flat array in JOINT_NAMES order (FR coxa, FR femur, ..., RL tibia).
  setAllServoAnglesDeg(angles12) {
    if (!Array.isArray(angles12) || angles12.length !== 12) {
      throw new Error("setAllServoAnglesDeg expects 12 numbers");
    }
    for (let leg = 0; leg < LEG_IDS.length; leg++) {
      const base = leg * 3;
      this.setLegServoAngles(LEG_IDS[leg], {
        coxa: angles12[base + 0],
        femur: angles12[base + 1],
        tibia: angles12[base + 2],
      });
    }
  }

  setLegServoSpeedLimit(dashLegId, { coxa, femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) throw new Error(`Unknown leg id: ${dashLegId}`);
    this._send({
      type: "set_leg_servo_speed_limit",
      legId,
      hipDegPerSec: coxa,
      thighDegPerSec: femur,
      calfDegPerSec: tibia,
    });
  }

  // limits = { femur: [min, max], tibia: [min, max] } in degrees.
  // Firmware only supports thigh/calf limits.
  setLegJointLimits(dashLegId, { femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) throw new Error(`Unknown leg id: ${dashLegId}`);
    this._send({
      type: "set_leg_joint_limits",
      legId,
      thighMinDeg: femur[0],
      thighMaxDeg: femur[1],
      calfMinDeg: tibia[0],
      calfMaxDeg: tibia[1],
    });
  }

  setLegServoChannelMap(dashLegId, { coxa, femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) throw new Error(`Unknown leg id: ${dashLegId}`);
    this._send({
      type: "set_leg_servo_channel_map",
      legId,
      hipChannel: coxa,
      thighChannel: femur,
      calfChannel: tibia,
    });
  }

  requestState() {
    this._send({ type: "get_state" });
  }

  // ─── Inbound parsing ──────────────────────────────────────────────────

  _onLine(line) {
    const trimmed = line.trim();
    if (!trimmed || trimmed[0] !== "{") return;
    let msg;
    try {
      msg = JSON.parse(trimmed);
    } catch (err) {
      this.emit("error", new Error(`bad JSON from firmware: ${trimmed.slice(0, 80)}`));
      return;
    }
    switch (msg.type) {
      case "state":
        this._handleState(msg.payload || {});
        break;
      case "hello_ack":
        this.emit("hello_ack", msg);
        break;
      case "ack":
        this.emit("ack", msg);
        break;
      case "error":
        this.emit("error", new Error(msg.error || "firmware error"));
        break;
      default:
        this.emit("message", msg);
    }
  }

  _handleState(fwPayload) {
    const dashState = translateState(fwPayload);
    this.lastState = dashState;
    this.emit("state", dashState);

    // Build a JointState-shaped object that matches what the dashboard
    // expects on `joint_states`: {name: [...], position: [...]}.
    const positionDeg = new Array(JOINT_NAMES.length).fill(0);
    for (let li = 0; li < LEG_IDS.length; li++) {
      const legKey = LEG_IDS[li];
      const legState = dashState.legs?.[legKey];
      if (!legState) continue;
      const cur = legState.current?.servoAnglesDeg || {};
      for (let ji = 0; ji < JOINT_ROWS.length; ji++) {
        const joint = JOINT_ROWS[ji];
        const idx = li * 3 + ji;
        positionDeg[idx] = Number.isFinite(cur[joint]) ? cur[joint] : 0;
      }
    }
    this.emit("joint_states", {
      name: [...JOINT_NAMES],
      position_servo_deg: positionDeg,
    });

    if (Number.isFinite(dashState.gasRaw)) {
      this.emit("gas", { data: dashState.gasRaw });
    }
  }

  _send(obj) {
    if (!this.port || !this.port.isOpen) return;
    const seq = ++this.seq;
    const line = JSON.stringify({ ...obj, seq }) + "\n";
    this.port.write(line);
  }
}

function translateState(fw) {
  const legs = {};
  if (fw.legs && typeof fw.legs === "object") {
    for (const [fwLegId, legData] of Object.entries(fw.legs)) {
      const dashLegId = FW_TO_DASH_LEG[fwLegId];
      if (!dashLegId) continue;
      legs[dashLegId] = {
        status: legData.status,
        lastError: legData.lastError,
        servoChannelMap: renameJointKeys(legData.servoChannelMap),
        servoSpeedLimitDegPerSec: renameJointKeys(legData.servoSpeedLimitDegPerSec),
        jointLimits: renameJointLimits(legData.jointLimits),
        desired: {
          servoAnglesDeg: renameJointKeys(legData.desired?.servoAnglesDeg),
        },
        current: {
          servoAnglesDeg: renameJointKeys(legData.current?.servoAnglesDeg),
        },
      };
    }
  }
  return {
    mode: fw.mode,
    activeAnimation: fw.activeAnimation,
    servosReleased: fw.servosReleased,
    servoUpdateRateHz: fw.servoUpdateRateHz ?? DEFAULT_SERVO_UPDATE_RATE_HZ,
    gasRaw: fw.gasRaw,
    firmwareMs: fw.firmwareMs,
    legs,
  };
}

function renameJointKeys(obj) {
  if (!obj || typeof obj !== "object") return null;
  const out = {};
  for (const [fwJoint, value] of Object.entries(obj)) {
    const dashJoint = FW_TO_DASH_JOINT[fwJoint];
    if (dashJoint) out[dashJoint] = value;
  }
  return out;
}

function renameJointLimits(limits) {
  if (!limits || typeof limits !== "object") return null;
  const out = {};
  if (limits.thighDeg) out.femurDeg = limits.thighDeg;
  if (limits.calfDeg) out.tibiaDeg = limits.calfDeg;
  return out;
}

export const _internals = { translateState, DASH_TO_FW_LEG, FW_TO_DASH_LEG };
