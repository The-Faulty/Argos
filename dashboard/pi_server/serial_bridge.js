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
// Extra spacing after each port.drain() before the next write. The ESP32's
// hardware UART RX FIFO is only 32 bytes (~0.35 ms at 921600 baud), so any
// brief interrupt-masked region in firmware (I2C transactions, etc.) can
// drop bytes at message boundaries. drain() alone leaves frames spaced
// ~1–2 ms; a few ms of additional pacing closes the gap. Tune up if you
// still see truncated frames in firmware error logs; tune down for snappier
// teleop. 0 disables.
const DEFAULT_PACING_MS = Number(process.env.ARGOS_SERIAL_PACING_MS ?? 5);

export class SerialBridge extends EventEmitter {
  constructor({ path = DEFAULT_PORT, baudRate = DEFAULT_BAUD, pacingMs = DEFAULT_PACING_MS } = {}) {
    super();
    this.path = path;
    this.baudRate = baudRate;
    this.pacingMs = Math.max(0, Number.isFinite(pacingMs) ? pacingMs : 0);
    this.port = null;
    this.parser = null;
    this.seq = 0;
    this.connected = false;
    this.lastState = null;
    this._reopenTimer = null;
    this._sentTypes = new Map(); // seq -> command type, for error attribution

    // Outbound write pacing. The 50 Hz auto-mode tick fires 4 per-leg servo
    // commands at once; sending them synchronously overruns the ESP32 RX FIFO
    // mid-message. We coalesce per-leg (only the latest pending angles for
    // each leg get sent) and await port.drain() between every write so the
    // host never outpaces the UART.
    this._pendingServoByLeg = new Map(); // fwLegId -> {hipServoDeg, thighServoDeg, calfServoDeg}
    this._lastSentServoByLeg = new Map(); // fwLegId -> last-sent angles, for dedupe
    this._pendingOther = []; // FIFO queue of non-servo command objects
    this._pumping = false;
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
      // Discard any stale queued state from before reconnect — the firmware
      // has reset too, so the dedupe cache and pending queues must clear.
      this._pendingServoByLeg.clear();
      this._lastSentServoByLeg.clear();
      this._pendingOther.length = 0;
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
    // Drop any in-flight servo targets so we don't immediately re-engage.
    this._pendingServoByLeg.clear();
    this._lastSentServoByLeg.clear();
    this._send({ type: "release_servos" });
  }

  setServoUpdateRate(hz) {
    this._send({ type: "set_servo_update_rate_hz", hz });
  }

  // angles is { coxa, femur, tibia } in 0..180 servo-horn degrees.
  // Enqueues into the per-leg pending map (latest values win) and kicks the
  // pump. Actual writes are paced by port.drain() in _pump().
  setLegServoAngles(dashLegId, { coxa, femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) throw new Error(`Unknown leg id: ${dashLegId}`);
    this._pendingServoByLeg.set(legId, {
      hipServoDeg: coxa,
      thighServoDeg: femur,
      calfServoDeg: tibia,
    });
    this._pump();
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
      case "error": {
        // Firmware shape: {"type":"error","seq":N,"message":"..."}.
        const text = msg.message || msg.error || "firmware error";
        const sentType = Number.isFinite(msg.seq) ? this._sentTypes.get(msg.seq) : undefined;
        const tag =
          Number.isFinite(msg.seq)
            ? ` (seq=${msg.seq}${sentType ? `, type=${sentType}` : ""})`
            : "";
        this.emit("error", new Error(`${text}${tag}`));
        break;
      }
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
    this._pendingOther.push(obj);
    this._pump();
  }

  // Drains queued frames serially, awaiting OS write completion between each
  // so the host doesn't outrun the UART. Non-servo commands are sent FIFO and
  // take priority over servo frames (so e.g. release_servos doesn't get stuck
  // behind a queue of pending angles).
  async _pump() {
    if (this._pumping) return;
    if (!this.port || !this.port.isOpen) return;
    this._pumping = true;
    try {
      while (this._pendingOther.length || this._pendingServoByLeg.size) {
        if (!this.port || !this.port.isOpen) return;
        if (this._pendingOther.length) {
          await this._writeOne(this._pendingOther.shift());
          if (this.pacingMs > 0) await sleep(this.pacingMs);
          continue;
        }
        const next = this._pendingServoByLeg.entries().next().value;
        if (!next) break;
        const [legId, angles] = next;
        this._pendingServoByLeg.delete(legId);
        const last = this._lastSentServoByLeg.get(legId);
        if (last && servoAnglesEqual(last, angles)) continue;
        this._lastSentServoByLeg.set(legId, angles);
        await this._writeOne({ type: "set_leg_servo_angles", legId, ...angles });
        if (this.pacingMs > 0) await sleep(this.pacingMs);
      }
    } finally {
      this._pumping = false;
    }
  }

  _writeOne(obj) {
    return new Promise((resolve) => {
      if (!this.port || !this.port.isOpen) {
        resolve();
        return;
      }
      const seq = ++this.seq;
      this._sentTypes.set(seq, obj.type);
      if (this._sentTypes.size > 256) {
        const firstKey = this._sentTypes.keys().next().value;
        this._sentTypes.delete(firstKey);
      }
      const line = JSON.stringify({ ...obj, seq }) + "\n";
      this.port.write(line, (err) => {
        if (err) {
          this.emit("error", err);
          resolve();
          return;
        }
        if (!this.port || !this.port.isOpen) {
          resolve();
          return;
        }
        this.port.drain((drainErr) => {
          if (drainErr) this.emit("error", drainErr);
          resolve();
        });
      });
    });
  }
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function servoAnglesEqual(a, b) {
  const eps = 0.05;
  return Math.abs(a.hipServoDeg - b.hipServoDeg) < eps
      && Math.abs(a.thighServoDeg - b.thighServoDeg) < eps
      && Math.abs(a.calfServoDeg - b.calfServoDeg) < eps;
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
