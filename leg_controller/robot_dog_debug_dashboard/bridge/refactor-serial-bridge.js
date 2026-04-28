import { EventEmitter } from "node:events";
import { ReadlineParser, SerialPort } from "serialport";

const DASH_TO_FW_LEG = {
  FR: "front_right",
  FL: "front_left",
  RR: "rear_right",
  RL: "rear_left",
};

const FW_TO_DASH_LEG = Object.fromEntries(
  Object.entries(DASH_TO_FW_LEG).map(([dashLegId, fwLegId]) => [fwLegId, dashLegId]),
);

const DASH_TO_FW_JOINT = {
  coxa: "hip",
  femur: "thigh",
  tibia: "calf",
};

const FW_TO_DASH_JOINT = Object.fromEntries(
  Object.entries(DASH_TO_FW_JOINT).map(([dashJoint, fwJoint]) => [fwJoint, dashJoint]),
);

const DASH_LEG_IDS = ["FR", "FL", "RR", "RL"];
const JOINT_ROWS = ["coxa", "femur", "tibia"];
const JOINT_NAMES = [
  "FR_coxa_joint", "FR_femur_joint", "FR_tibia_joint",
  "FL_coxa_joint", "FL_femur_joint", "FL_tibia_joint",
  "RR_coxa_joint", "RR_femur_joint", "RR_tibia_joint",
  "RL_coxa_joint", "RL_femur_joint", "RL_tibia_joint",
];

const DEFAULT_SERVO_UPDATE_RATE_HZ = 50;
const SERVO_CENTER_DEG = 90;
const DEG2RAD = Math.PI / 180;

const DEFAULT_PORT = process.env.ARGOS_SERIAL_PORT || "/dev/ttyUSB0";
const DEFAULT_BAUD = Number(process.env.ARGOS_SERIAL_BAUD || 921600);
const DEFAULT_PACING_MS = Number(process.env.ARGOS_SERIAL_PACING_MS ?? 12);
const DEFAULT_ACK_TIMEOUT_MS = Number(process.env.ARGOS_SERIAL_ACK_TIMEOUT_MS ?? 0);

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function createDeferred() {
  let resolve;
  let reject;
  const promise = new Promise((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return { promise, resolve, reject };
}

function servoAnglesEqual(a, b) {
  const eps = 0.05;
  return Math.abs(a.hipServoDeg - b.hipServoDeg) < eps
    && Math.abs(a.thighServoDeg - b.thighServoDeg) < eps
    && Math.abs(a.calfServoDeg - b.calfServoDeg) < eps;
}

function cloneServoCommand(command) {
  return {
    hipServoDeg: command.hipServoDeg,
    thighServoDeg: command.thighServoDeg,
    calfServoDeg: command.calfServoDeg,
  };
}

function inverseServoCal(_jointName, servoDeg) {
  return (servoDeg - SERVO_CENTER_DEG) * DEG2RAD;
}

function renameJointKeys(obj) {
  if (!obj || typeof obj !== "object") {
    return null;
  }

  const out = {};
  for (const [fwJoint, value] of Object.entries(obj)) {
    const dashJoint = FW_TO_DASH_JOINT[fwJoint];
    if (dashJoint) {
      out[dashJoint] = value;
    }
  }
  return out;
}

function renameJointLimits(limits) {
  if (!limits || typeof limits !== "object") {
    return null;
  }

  const out = {};
  if (limits.thighDeg) {
    out.femurDeg = limits.thighDeg;
  }
  if (limits.calfDeg) {
    out.tibiaDeg = limits.calfDeg;
  }
  return out;
}

function translateState(fw) {
  const legs = {};
  if (fw.legs && typeof fw.legs === "object") {
    for (const [fwLegId, legData] of Object.entries(fw.legs)) {
      const dashLegId = FW_TO_DASH_LEG[fwLegId];
      if (!dashLegId) {
        continue;
      }

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
    imu: fw.imu ?? null,
    firmwareMs: fw.firmwareMs,
    legs,
  };
}

export class SerialBridge extends EventEmitter {
  constructor({
    path = DEFAULT_PORT,
    baudRate = DEFAULT_BAUD,
    pacingMs = DEFAULT_PACING_MS,
    ackTimeoutMs = DEFAULT_ACK_TIMEOUT_MS,
    serialPortFactory = (options) => new SerialPort(options),
    parserFactory = (port) => port.pipe(new ReadlineParser({ delimiter: "\n" })),
  } = {}) {
    super();
    this.path = path;
    this.baudRate = baudRate;
    this.pacingMs = Math.max(0, Number.isFinite(pacingMs) ? pacingMs : 0);
    this.ackTimeoutMs = Math.max(0, Number.isFinite(ackTimeoutMs) ? ackTimeoutMs : 0);
    this.serialPortFactory = serialPortFactory;
    this.parserFactory = parserFactory;
    this.port = null;
    this.parser = null;
    this.seq = 0;
    this.connected = false;
    this.lastState = null;
    this._reopenTimer = null;
    this._manualClose = false;
    this._sentTypes = new Map();
    this._pendingServoByLeg = new Map();
    this._lastSentServoByLeg = new Map();
    this._pendingOther = [];
    this._pendingAcks = new Map();
    this._pumping = false;
  }

  connect() {
    if (this.port) {
      return;
    }

    this._manualClose = false;

    const port = this.serialPortFactory({
      path: this.path,
      baudRate: this.baudRate,
      autoOpen: false,
    });
    const parser = this.parserFactory(port);

    this.port = port;
    this.parser = parser;

    parser.on("data", (line) => this._onLine(line));
    port.on("open", () => {
      this.connected = true;
      this._pendingServoByLeg.clear();
      this._lastSentServoByLeg.clear();
      this._pendingOther.length = 0;
      this._clearPendingAcks();
      this.emit("open");
      this.enqueueRawCommand({ type: "hello" }).catch(() => {});
      this.enqueueRawCommand({ type: "set_mode", mode: "direct_servo_angles" }).catch(() => {});
      this.enqueueRawCommand({ type: "get_state" }).catch(() => {});
    });
    port.on("close", () => {
      this.connected = false;
      this._clearPendingAcks();
      const disconnectError = new Error("Serial bridge closed.");
      this._rejectQueuedServo(disconnectError);
      this._rejectQueuedOther(disconnectError);
      this.emit("close");
      if (!this._manualClose) {
        this._scheduleReopen();
      }
    });
    port.on("error", (error) => this.emit("error", error));

    port.open((error) => {
      if (error) {
        this.emit("error", error);
        if (!this._manualClose) {
          this._scheduleReopen();
        }
      }
    });
  }

  disconnect() {
    this._manualClose = true;
    if (this._reopenTimer) {
      clearTimeout(this._reopenTimer);
      this._reopenTimer = null;
    }

    const port = this.port;
    this.port = null;
    this.parser = null;

    if (port?.isOpen) {
      port.close();
    }
  }

  setMode(mode) {
    return this.enqueueRawCommand({ type: "set_mode", mode });
  }

  releaseServos() {
    this._resolveQueuedServo();
    this._pendingServoByLeg.clear();
    this._lastSentServoByLeg.clear();
    return this.enqueueRawCommand({ type: "release_servos" });
  }

  setServoUpdateRate(hz) {
    return this.enqueueRawCommand({ type: "set_servo_update_rate_hz", hz });
  }

  setLegServoAngles(dashLegId, { coxa, femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) {
      throw new Error(`Unknown leg id: ${dashLegId}`);
    }

    const deferred = createDeferred();
    const previous = this._pendingServoByLeg.get(legId);
    if (previous) {
      previous.deferred.resolve();
    }

    this._pendingServoByLeg.set(legId, {
      deferred,
      command: {
        hipServoDeg: coxa,
        thighServoDeg: femur,
        calfServoDeg: tibia,
      },
    });
    this._pump().catch((error) => this.emit("error", error));
    return deferred.promise;
  }

  setAllServoAnglesDeg(angles12) {
    if (!Array.isArray(angles12) || angles12.length !== 12) {
      throw new Error("setAllServoAnglesDeg expects 12 numbers");
    }

    return Promise.all(
      DASH_LEG_IDS.map((legId, legIndex) => this.setLegServoAngles(legId, {
        coxa: angles12[(legIndex * 3) + 0],
        femur: angles12[(legIndex * 3) + 1],
        tibia: angles12[(legIndex * 3) + 2],
      })),
    );
  }

  setLegServoSpeedLimit(dashLegId, { coxa, femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) {
      throw new Error(`Unknown leg id: ${dashLegId}`);
    }

    return this.enqueueRawCommand({
      type: "set_leg_servo_speed_limit",
      legId,
      hipDegPerSec: coxa,
      thighDegPerSec: femur,
      calfDegPerSec: tibia,
    });
  }

  setLegJointLimits(dashLegId, { femur, tibia }) {
    const legId = DASH_TO_FW_LEG[dashLegId];
    if (!legId) {
      throw new Error(`Unknown leg id: ${dashLegId}`);
    }

    return this.enqueueRawCommand({
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
    if (!legId) {
      throw new Error(`Unknown leg id: ${dashLegId}`);
    }

    return this.enqueueRawCommand({
      type: "set_leg_servo_channel_map",
      legId,
      hipChannel: coxa,
      thighChannel: femur,
      calfChannel: tibia,
    });
  }

  requestState() {
    return this.enqueueRawCommand({ type: "get_state" });
  }

  enqueueRawCommand(command) {
    const deferred = createDeferred();
    this._pendingOther.push({
      deferred,
      command: { ...command },
    });
    this._pump().catch((error) => this.emit("error", error));
    return deferred.promise;
  }

  _scheduleReopen() {
    if (this._reopenTimer) {
      return;
    }

    this._reopenTimer = setTimeout(() => {
      this._reopenTimer = null;
      this.port = null;
      this.connect();
    }, 2000);
  }

  _rejectQueuedServo(error) {
    for (const entry of this._pendingServoByLeg.values()) {
      entry.deferred.reject(error);
    }
    this._pendingServoByLeg.clear();
  }

  _resolveQueuedServo() {
    for (const entry of this._pendingServoByLeg.values()) {
      entry.deferred.resolve();
    }
  }

  _rejectQueuedOther(error) {
    while (this._pendingOther.length) {
      this._pendingOther.shift().deferred.reject(error);
    }
  }

  _onLine(line) {
    const trimmed = line.trim();
    if (!trimmed || trimmed[0] !== "{") {
      return;
    }

    let message;
    try {
      message = JSON.parse(trimmed);
    } catch {
      this.emit("error", new Error(`bad JSON from firmware: ${trimmed.slice(0, 80)}`));
      return;
    }

    switch (message.type) {
      case "state":
        this._handleState(message.payload || {});
        break;
      case "hello_ack":
        this._resolveAck(message.seq);
        this.emit("hello_ack", message);
        break;
      case "ack":
        this._resolveAck(message.seq);
        this.emit("ack", message);
        break;
      case "error": {
        const text = message.message || message.error || "firmware error";
        const sentType = Number.isFinite(message.seq) ? this._sentTypes.get(message.seq) : undefined;
        const tag = Number.isFinite(message.seq)
          ? ` (seq=${message.seq}${sentType ? `, type=${sentType}` : ""})`
          : "";
        this._resolveAck(message.seq);
        this.emit("error", new Error(`${text}${tag}`));
        break;
      }
      default:
        this.emit("message", message);
        break;
    }
  }

  _handleState(fwPayload) {
    const dashState = translateState(fwPayload);
    this.lastState = dashState;
    this.emit("state", dashState);

    const positionDeg = new Array(JOINT_NAMES.length).fill(0);
    const positionRad = new Array(JOINT_NAMES.length).fill(0);

    for (let legIndex = 0; legIndex < DASH_LEG_IDS.length; legIndex += 1) {
      const legKey = DASH_LEG_IDS[legIndex];
      const legState = dashState.legs?.[legKey];
      if (!legState) {
        continue;
      }

      const currentServoAngles = legState.current?.servoAnglesDeg || {};
      for (let jointIndex = 0; jointIndex < JOINT_ROWS.length; jointIndex += 1) {
        const joint = JOINT_ROWS[jointIndex];
        const index = (legIndex * 3) + jointIndex;
        const jointName = JOINT_NAMES[index];
        positionDeg[index] = Number.isFinite(currentServoAngles[joint]) ? currentServoAngles[joint] : 0;
        const jointRad = inverseServoCal(jointName, positionDeg[index]);
        positionRad[index] = Number.isFinite(jointRad) ? jointRad : 0;
      }
    }

    this.emit("joint_states", {
      name: [...JOINT_NAMES],
      position: positionRad,
      position_servo_deg: positionDeg,
    });

    if (Number.isFinite(dashState.gasRaw)) {
      this.emit("gas", { data: dashState.gasRaw });
    }

    const firmwareImu = dashState.imu;
    if (firmwareImu?.present === true) {
      this.emit("firmware_imu", {
        accel: [firmwareImu.ax, firmwareImu.ay, firmwareImu.az],
        gyro: [firmwareImu.gx, firmwareImu.gy, firmwareImu.gz],
        mag: [firmwareImu.mx, firmwareImu.my, firmwareImu.mz],
        roll: firmwareImu.roll,
        pitch: firmwareImu.pitch,
        yaw: firmwareImu.yaw,
      });
    }
  }

  async _pump() {
    if (this._pumping || !this.port || !this.port.isOpen) {
      return;
    }

    this._pumping = true;
    try {
      while (this._pendingOther.length || this._pendingServoByLeg.size) {
        if (!this.port || !this.port.isOpen) {
          return;
        }

        if (this._pendingOther.length) {
          const entry = this._pendingOther.shift();
          try {
            await this._writeOne(entry.command);
            entry.deferred.resolve();
          } catch (error) {
            entry.deferred.reject(error);
            this.emit("error", error);
          }
          if (this.pacingMs > 0) {
            await sleep(this.pacingMs);
          }
          continue;
        }

        const next = this._pendingServoByLeg.entries().next().value;
        if (!next) {
          break;
        }

        const [legId, entry] = next;
        this._pendingServoByLeg.delete(legId);

        const previous = this._lastSentServoByLeg.get(legId);
        if (previous && servoAnglesEqual(previous, entry.command)) {
          entry.deferred.resolve();
          continue;
        }

        try {
          await this._writeOne({
            type: "set_leg_servo_angles",
            legId,
            ...entry.command,
          });
          this._lastSentServoByLeg.set(legId, cloneServoCommand(entry.command));
          entry.deferred.resolve();
        } catch (error) {
          entry.deferred.reject(error);
          this.emit("error", error);
        }

        if (this.pacingMs > 0) {
          await sleep(this.pacingMs);
        }
      }
    } finally {
      this._pumping = false;
      if ((this._pendingOther.length || this._pendingServoByLeg.size) && this.port?.isOpen) {
        queueMicrotask(() => {
          this._pump().catch((error) => this.emit("error", error));
        });
      }
    }
  }

  _writeOne(command) {
    return new Promise((resolve, reject) => {
      if (!this.port || !this.port.isOpen) {
        reject(new Error("Serial bridge is not connected."));
        return;
      }

      const seq = ++this.seq;
      const waitForAck = this._waitForAck(seq);
      this._sentTypes.set(seq, command.type);
      if (this._sentTypes.size > 256) {
        const firstKey = this._sentTypes.keys().next().value;
        this._sentTypes.delete(firstKey);
      }

      const line = `${JSON.stringify({ ...command, seq })}\n`;
      this.port.write(line, (error) => {
        if (error) {
          this._resolveAck(seq);
          reject(error);
          return;
        }

        if (!this.port || !this.port.isOpen) {
          this._resolveAck(seq);
          reject(new Error("Serial bridge is not connected."));
          return;
        }

        this.port.drain((drainError) => {
          if (drainError) {
            this._resolveAck(seq);
            reject(drainError);
            return;
          }

          waitForAck.then(resolve, reject);
        });
      });
    });
  }

  _waitForAck(seq) {
    if (!(this.ackTimeoutMs > 0)) {
      return Promise.resolve();
    }

    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        this._pendingAcks.delete(seq);
        resolve();
      }, this.ackTimeoutMs);
      this._pendingAcks.set(seq, { resolve, timer });
    });
  }

  _resolveAck(seq) {
    if (!Number.isFinite(seq)) {
      return;
    }

    const pending = this._pendingAcks.get(seq);
    if (!pending) {
      return;
    }

    clearTimeout(pending.timer);
    this._pendingAcks.delete(seq);
    pending.resolve();
  }

  _clearPendingAcks() {
    for (const pending of this._pendingAcks.values()) {
      clearTimeout(pending.timer);
      pending.resolve();
    }
    this._pendingAcks.clear();
  }
}

export const _internals = {
  DASH_TO_FW_JOINT,
  DASH_TO_FW_LEG,
  FW_TO_DASH_JOINT,
  FW_TO_DASH_LEG,
  JOINT_NAMES,
  JOINT_ROWS,
  servoAnglesEqual,
  translateState,
};
