import express from "express";
import http from "http";
import { WebSocketServer } from "ws";
import { interpolateFullBodyClip, validateClip } from "../shared/animation.js";
import {
  buildLegPoseFromFoot,
  buildLegPoseFromJointAngles,
  buildLegPoseFromServoAngles,
  createNeutralCalibration,
  normalizeJointLimits,
} from "../shared/kinematics.js";
import {
  DEFAULT_FULL_BODY_CLIP,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_LEG_COMMAND,
  DEFAULT_SERVO_CHANNEL_MAP,
  LEG_IDS,
} from "../shared/robot-config.js";
import { validateCommand } from "../shared/protocol.js";

const PORT = Number(process.env.PORT || 8787);
const TELEMETRY_INTERVAL_MS = 100;
const BUILTIN_STATUS_INTERVAL_MS = 400;
const ANIMATION_STATUS_INTERVAL_MS = 250;
const CONTROL_INTERVAL_MS = 20;

const PCA9685_ADDRESS = parseIntegerEnv("PI_PCA9685_ADDRESS", 0x40);
const PCA9685_FREQUENCY = Number(process.env.PI_PCA9685_FREQUENCY || 50);
const PI_I2C_BUS = Number(process.env.PI_I2C_BUS || 1);
const SERVO_PWM_MIN_TICKS = Number(process.env.PI_SERVO_PWM_MIN_TICKS || 102);
const SERVO_PWM_MAX_TICKS = Number(process.env.PI_SERVO_PWM_MAX_TICKS || 512);
const SERVO_SUPPLY_VOLTS = Number(process.env.PI_SERVO_SUPPLY_VOLTS || 7.4);
const SERVO_SPEED_SAFETY_FACTOR = 0.8;
const MOTION_TIME_SCALE = 1.15;
const SERVO_MIN_DEG = 0;
const SERVO_MAX_DEG = 180;
const THIGH_CENTER_DEG = Number(process.env.PI_THIGH_CENTER_DEG || 90);
const CALF_CENTER_DEG = Number(process.env.PI_CALF_CENTER_DEG || 90);
const THIGH_SIGN = Number(process.env.PI_THIGH_SIGN || 1);
const CALF_SIGN = Number(process.env.PI_CALF_SIGN || -1);
const DRY_RUN = process.env.PI_DOG_DRY_RUN === "1";
const calibration = createNeutralCalibration();

const MODE_OPTIONS = new Set([
  "idle",
  "direct_foot_xy",
  "direct_joint_angles",
  "direct_servo_angles",
  "builtin_walk",
  "builtin_crouch",
  "animation_playback",
]);

function parseIntegerEnv(key, fallback) {
  const raw = process.env[key];
  if (!raw) {
    return fallback;
  }

  if (raw.startsWith("0x") || raw.startsWith("0X")) {
    const parsed = Number.parseInt(raw, 16);
    return Number.isFinite(parsed) ? parsed : fallback;
  }

  const parsed = Number.parseInt(raw, 10);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function logControllerError(context, error) {
  if (!error) {
    return;
  }

  const code = error.code ? ` (${error.code})` : "";
  console.error(`${context}${code}: ${error.message}`);
}

function safeSendWs(client, message) {
  try {
    if (client.readyState === 1) {
      client.send(message, (error) => {
        if (error) {
          logControllerError("WebSocket send failed", error);
          try {
            client.close();
          } catch {
            // ignore close errors on dead sockets
          }
        }
      });
    }
  } catch (error) {
    logControllerError("WebSocket send threw", error);
  }
}

function safeJson(response, statusCode, payload) {
  if (response.destroyed || response.writableEnded || response.headersSent) {
    return;
  }

  try {
    response.status(statusCode).json(payload);
  } catch (error) {
    logControllerError("HTTP response write failed", error);
  }
}

function servoSpeedDegPerSecFromVoltage(volts) {
  let secPer60 = 0.13;
  if (volts >= 8.4) {
    secPer60 = 0.1;
  } else if (volts >= 7.4) {
    const t = (volts - 7.4) / (8.4 - 7.4);
    secPer60 = 0.11 + t * (0.1 - 0.11);
  } else if (volts >= 5.0) {
    const t = (volts - 5.0) / (7.4 - 5.0);
    secPer60 = 0.13 + t * (0.11 - 0.13);
  }
  return 60 / secPer60;
}

function applyServoCalibration(modelDeg, centerDeg, sign) {
  return centerDeg + sign * (modelDeg - 90);
}

function removeServoCalibration(rawDeg, centerDeg, sign) {
  if (!sign) {
    return 90;
  }
  return 90 + (rawDeg - centerDeg) / sign;
}

function servoDegToPcaTicks(servoDeg) {
  const clamped = clamp(servoDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const ratio = clamped / 180;
  return Math.round(SERVO_PWM_MIN_TICKS + ratio * (SERVO_PWM_MAX_TICKS - SERVO_PWM_MIN_TICKS));
}

function builtinWalkTarget(legIndex, timeSec) {
  const phaseOffsets = [0, 0.5, 0.5, 0];
  const cycle = ((timeSec / 1.6) + phaseOffsets[legIndex]) % 1;
  const x = -18 + (18 - -18) * cycle;
  const y =
    cycle < 0.45
      ? 10 * Math.sin((cycle / 0.45) * Math.PI)
      : -6 * Math.sin(((cycle - 0.45) / 0.55) * Math.PI);
  return { x, y };
}

function builtinCrouchTarget(timeSec) {
  const t = clamp(timeSec / 0.9, 0, 1);
  const eased = t * t * (3 - 2 * t);
  return {
    x: 0,
    y: 0 + (-28 - 0) * eased,
  };
}

function createPseudoPort() {
  return {
    path: "raspberry-pi-controller",
    manufacturer: "Raspberry Pi",
    serialNumber: `i2c-${PI_I2C_BUS}`,
  };
}

function createStatus() {
  const legs = {};
  for (const legId of LEG_IDS) {
    const desired = buildLegPoseFromFoot(DEFAULT_LEG_COMMAND.foot, calibration, {
      jointLimits: DEFAULT_JOINT_LIMITS,
    });
    legs[legId] = {
      desired,
      current: desired,
      status: "idle",
      lastError: "",
      servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
      jointLimits: clone(DEFAULT_JOINT_LIMITS),
    };
  }

  return {
    connected: false,
    connectedPort: null,
    mode: "idle",
    servosReleased: false,
    activeAnimation: null,
    lastAck: null,
    lastError: null,
    firmwareMs: 0,
    ports: [createPseudoPort()],
    legs,
  };
}

class Pca9685Driver {
  constructor({ address, frequencyHz, busNumber, dryRun }) {
    this.address = address;
    this.frequencyHz = frequencyHz;
    this.busNumber = busNumber;
    this.dryRun = dryRun;
    this.bus = null;
    this.mock = dryRun;
  }

  async connect() {
    if (this.dryRun) {
      console.log("Raspberry Pi controller running in dry-run mode; PCA9685 writes are disabled.");
      return;
    }

    try {
      const i2cBus = await import("i2c-bus");
      this.bus = await i2cBus.openPromisified(this.busNumber);
      await this.initializePca9685();
      return;
    } catch (error) {
      this.mock = true;
      console.warn(`Falling back to dry-run mode because I2C initialization failed: ${error.message}`);
    }
  }

  async initializePca9685() {
    const MODE1 = 0x00;
    const MODE2 = 0x01;
    const PRESCALE = 0xfe;
    const RESTART = 0x80;
    const SLEEP = 0x10;
    const OUTDRV = 0x04;

    const prescaleValue = Math.round(25_000_000 / (4096 * this.frequencyHz)) - 1;
    const oldMode = await this.bus.readByte(this.address, MODE1);
    const sleepMode = (oldMode & 0x7f) | SLEEP;

    await this.bus.writeByte(this.address, MODE1, sleepMode);
    await this.bus.writeByte(this.address, PRESCALE, prescaleValue);
    await this.bus.writeByte(this.address, MODE2, OUTDRV);
    await this.bus.writeByte(this.address, MODE1, oldMode);
    await sleep(5);
    await this.bus.writeByte(this.address, MODE1, oldMode | RESTART);
  }

  async setPwm(channel, on, off) {
    if (this.mock || !this.bus) {
      return;
    }

    const LED0_ON_L = 0x06;
    const register = LED0_ON_L + 4 * channel;
    await this.bus.writeByte(this.address, register, on & 0xff);
    await this.bus.writeByte(this.address, register + 1, on >> 8);
    await this.bus.writeByte(this.address, register + 2, off & 0xff);
    await this.bus.writeByte(this.address, register + 3, off >> 8);
  }

  async setServoAngle(channel, servoDeg) {
    await this.setPwm(channel, 0, servoDegToPcaTicks(servoDeg));
  }

  async releaseChannel(channel) {
    await this.setPwm(channel, 0, 4096);
  }

  async close() {
    if (this.bus) {
      await this.bus.close();
      this.bus = null;
    }
  }
}

class ServoActuator {
  constructor(channel, centerDeg, sign) {
    const initialRawDeg = clamp(centerDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    this.channel = channel;
    this.centerDeg = centerDeg;
    this.sign = sign;
    this.startRawDeg = initialRawDeg;
    this.targetRawDeg = initialRawDeg;
    this.lastEstimateRawDeg = initialRawDeg;
    this.lastWrittenRawDeg = Number.NaN;
    this.moveStartMs = Date.now();
    this.moveDurationMs = 1;
    this.released = false;
  }

  updateChannel(channel) {
    this.channel = channel;
    this.lastWrittenRawDeg = Number.NaN;
  }

  estimate(nowMs) {
    if (this.released) {
      return this.lastEstimateRawDeg;
    }

    const elapsedMs = Math.max(0, nowMs - this.moveStartMs);
    if (elapsedMs >= this.moveDurationMs) {
      this.lastEstimateRawDeg = this.targetRawDeg;
      return this.targetRawDeg;
    }

    const t = clamp(elapsedMs / this.moveDurationMs, 0, 1);
    const eased = t * t * (3 - 2 * t);
    this.lastEstimateRawDeg = this.startRawDeg + (this.targetRawDeg - this.startRawDeg) * eased;
    return this.lastEstimateRawDeg;
  }

  commandModelAngle(modelDeg) {
    const nowMs = Date.now();
    const targetRawDeg = clamp(
      applyServoCalibration(modelDeg, this.centerDeg, this.sign),
      SERVO_MIN_DEG,
      SERVO_MAX_DEG,
    );
    const currentRawDeg = this.estimate(nowMs);
    this.startRawDeg = currentRawDeg;
    this.targetRawDeg = targetRawDeg;
    this.moveStartMs = nowMs;
    this.moveDurationMs = Math.max(
      CONTROL_INTERVAL_MS,
      (Math.abs(this.targetRawDeg - currentRawDeg) / (servoSpeedDegPerSecFromVoltage(SERVO_SUPPLY_VOLTS) * SERVO_SPEED_SAFETY_FACTOR)) * 1000 * MOTION_TIME_SCALE,
    );
    this.released = false;
  }

  release() {
    this.lastEstimateRawDeg = this.estimate(Date.now());
    this.startRawDeg = this.lastEstimateRawDeg;
    this.targetRawDeg = this.lastEstimateRawDeg;
    this.moveStartMs = Date.now();
    this.moveDurationMs = 1;
    this.released = true;
  }

  currentModelDeg(nowMs = Date.now()) {
    return removeServoCalibration(this.estimate(nowMs), this.centerDeg, this.sign);
  }
}

class RaspberryPiController {
  constructor() {
    this.status = createStatus();
    this.hardware = new Pca9685Driver({
      address: PCA9685_ADDRESS,
      frequencyHz: PCA9685_FREQUENCY,
      busNumber: PI_I2C_BUS,
      dryRun: DRY_RUN,
    });
    this.wss = null;
    this.loopHandle = null;
    this.lastTelemetryMs = 0;
    this.lastBuiltinStatusMs = 0;
    this.lastAnimationStatusMs = 0;
    this.modeStartMs = Date.now();
    this.animation = {
      clip: validateClip(DEFAULT_FULL_BODY_CLIP),
      playing: false,
      paused: false,
      pausedTimeSec: 0,
      startMs: Date.now(),
    };
    this.uploadedClips = new Map();
    this.legs = Object.fromEntries(
      LEG_IDS.map((legId) => [
        legId,
        {
          thigh: new ServoActuator(DEFAULT_SERVO_CHANNEL_MAP[legId].thigh, THIGH_CENTER_DEG, THIGH_SIGN),
          calf: new ServoActuator(DEFAULT_SERVO_CHANNEL_MAP[legId].calf, CALF_CENTER_DEG, CALF_SIGN),
        },
      ]),
    );
  }

  attachWebSocketServer(wss) {
    this.wss = wss;
  }

  broadcast(type, payload) {
    if (!this.wss) {
      return;
    }
    const message = JSON.stringify({ type, ...(payload ? { payload } : {}) });
    for (const client of this.wss.clients) {
      safeSendWs(client, message);
    }
  }

  broadcastEvent(event) {
    if (!this.wss) {
      return;
    }
    const message = JSON.stringify(event);
    for (const client of this.wss.clients) {
      safeSendWs(client, message);
    }
  }

  async connect() {
    if (!this.status.connected) {
      await this.hardware.connect();
    }

    this.status.connected = true;
    this.status.connectedPort = `i2c-${PI_I2C_BUS}@0x${PCA9685_ADDRESS.toString(16)}`;
    this.status.ports = [createPseudoPort()];
    this.startLoop();
    await this.flushOutputs();
    this.refreshCurrentState();
    this.status.firmwareMs = Date.now();
    this.broadcast("status", this.status);
    this.broadcastEvent({ type: "hello_ack", payload: this.status });
  }

  async disconnect() {
    await this.releaseAllServos();
    this.status.connected = false;
    this.status.connectedPort = null;
    this.broadcast("status", this.status);
  }

  startLoop() {
    if (this.loopHandle) {
      return;
    }

    this.loopHandle = setInterval(() => {
      void this.tick().catch((error) => {
        this.status.lastError = error.message;
        logControllerError("Controller tick failed", error);
      });
    }, CONTROL_INTERVAL_MS);
  }

  updateMode(mode) {
    if (!MODE_OPTIONS.has(mode)) {
      throw new Error(`Unknown mode: ${mode}`);
    }

    this.status.mode = mode;
    this.modeStartMs = Date.now();
    if (mode === "idle") {
      this.animation.playing = false;
      this.animation.paused = false;
    }
  }

  refreshCurrentState(nowMs = Date.now()) {
    for (const legId of LEG_IDS) {
      const legStatus = this.status.legs[legId];
      const actuators = this.legs[legId];
      const currentServoAngles = {
        thigh: actuators.thigh.currentModelDeg(nowMs),
        calf: actuators.calf.currentModelDeg(nowMs),
      };

      legStatus.current = buildLegPoseFromServoAngles(currentServoAngles, calibration, {
        jointLimits: legStatus.jointLimits,
      });
    }

    this.status.firmwareMs = nowMs;
  }

  async flushOutputs(nowMs = Date.now()) {
    if (!this.status.connected) {
      return;
    }

    for (const legId of LEG_IDS) {
      const actuators = this.legs[legId];
      for (const servo of [actuators.thigh, actuators.calf]) {
        if (servo.released) {
          if (!Number.isNaN(servo.lastWrittenRawDeg)) {
            await this.hardware.releaseChannel(servo.channel);
            servo.lastWrittenRawDeg = Number.NaN;
          }
          continue;
        }

        const estimateRawDeg = servo.estimate(nowMs);
        if (!Number.isFinite(estimateRawDeg)) {
          continue;
        }

        if (!Number.isFinite(servo.lastWrittenRawDeg) || Math.abs(estimateRawDeg - servo.lastWrittenRawDeg) >= 0.5) {
          await this.hardware.setServoAngle(servo.channel, estimateRawDeg);
          servo.lastWrittenRawDeg = estimateRawDeg;
        }
      }
    }
  }

  setAck(message) {
    this.status.lastAck = message;
    this.broadcastEvent({ type: "ack", message });
  }

  setError(message) {
    this.status.lastError = message;
    this.broadcastEvent({ type: "error", message });
  }

  setLegError(legId, message) {
    this.status.legs[legId].lastError = message;
    this.setError(message);
  }

  clearLegError(legId) {
    this.status.legs[legId].lastError = "";
  }

  commandLegPose(legId, pose, mode, statusLabel) {
    const legStatus = this.status.legs[legId];
    const actuators = this.legs[legId];
    this.updateMode(mode);
    this.status.servosReleased = false;
    legStatus.status = statusLabel;
    legStatus.desired = pose;
    this.clearLegError(legId);
    actuators.thigh.commandModelAngle(pose.servoAnglesDeg.thigh);
    actuators.calf.commandModelAngle(pose.servoAnglesDeg.calf);
  }

  setLegFootTarget(legId, foot, { internal = false } = {}) {
    const legStatus = this.status.legs[legId];
    const pose = buildLegPoseFromFoot(foot, calibration, {
      startThetaThigh: legStatus.desired?.geometry?.thetaThigh,
      startThetaServo: legStatus.desired?.geometry?.thetaServo,
      jointLimits: legStatus.jointLimits,
    });

    if (!pose.reachable || !pose.geometry.valid) {
      const message = `Leg ${legId} target is unreachable.`;
      this.setLegError(legId, message);
      if (!internal) {
        throw new Error(message);
      }
      return;
    }

    this.commandLegPose(legId, pose, "direct_foot_xy", "direct_foot_xy");
  }

  setLegJointAngles(legId, jointAngles) {
    const legStatus = this.status.legs[legId];
    const pose = buildLegPoseFromJointAngles(jointAngles, calibration, {
      startThetaServo: legStatus.desired?.geometry?.thetaServo,
      jointLimits: legStatus.jointLimits,
    });

    if (!pose.reachable || !pose.geometry.valid) {
      const message = `Leg ${legId} joint target is unreachable.`;
      this.setLegError(legId, message);
      throw new Error(message);
    }

    this.commandLegPose(legId, pose, "direct_joint_angles", "direct_joint_angles");
  }

  setLegServoAngles(legId, servoAngles) {
    const legStatus = this.status.legs[legId];
    const pose = buildLegPoseFromServoAngles(servoAngles, calibration, {
      jointLimits: legStatus.jointLimits,
    });
    this.commandLegPose(legId, pose, "direct_servo_angles", "direct_servo_angles");
  }

  setLegServoChannelMap(legId, thighChannel, calfChannel) {
    const legStatus = this.status.legs[legId];
    legStatus.servoChannelMap = {
      thigh: thighChannel,
      calf: calfChannel,
    };
    this.legs[legId].thigh.updateChannel(thighChannel);
    this.legs[legId].calf.updateChannel(calfChannel);
  }

  setLegJointLimits(legId, jointLimits) {
    const legStatus = this.status.legs[legId];
    legStatus.jointLimits = normalizeJointLimits(jointLimits);
    this.setLegFootTarget(legId, legStatus.desired?.foot ?? DEFAULT_LEG_COMMAND.foot, { internal: true });
  }

  runBuiltin(name) {
    if (name !== "walk" && name !== "crouch") {
      throw new Error(`Unknown builtin '${name}'.`);
    }

    this.updateMode(name === "walk" ? "builtin_walk" : "builtin_crouch");
    this.status.servosReleased = false;
  }

  uploadAnimation(clip) {
    const validated = validateClip(clip);
    this.uploadedClips.set(validated.name, validated);
    this.animation.clip = validated;
    this.status.activeAnimation = validated.name;
    return validated;
  }

  playAnimation(name) {
    const clip = this.uploadedClips.get(name);
    if (!clip) {
      throw new Error(`Animation '${name}' has not been uploaded.`);
    }

    this.animation.clip = clip;
    this.animation.playing = true;
    this.animation.paused = false;
    this.animation.pausedTimeSec = 0;
    this.animation.startMs = Date.now();
    this.status.activeAnimation = name;
    this.status.servosReleased = false;
    this.updateMode("animation_playback");
  }

  stopAnimation() {
    this.animation.playing = false;
    this.animation.paused = false;
    this.animation.pausedTimeSec = 0;
    this.updateMode("idle");
  }

  pauseAnimation() {
    if (!this.animation.playing) {
      return;
    }

    this.animation.playing = false;
    this.animation.paused = true;
    this.animation.pausedTimeSec = (Date.now() - this.animation.startMs) / 1000;
  }

  async releaseAllServos() {
    for (const legId of LEG_IDS) {
      this.legs[legId].thigh.release();
      this.legs[legId].calf.release();
    }

    this.animation.playing = false;
    this.animation.paused = false;
    this.status.mode = "idle";
    this.status.activeAnimation = null;
    this.status.servosReleased = true;
    await this.flushOutputs();
  }

  handleCommand(command) {
    validateCommand(command);
    switch (command.type) {
      case "set_mode":
        this.updateMode(command.mode);
        this.status.servosReleased = false;
        this.setAck("mode updated");
        return;
      case "release_servos":
        return this.releaseAllServos().then(() => {
          this.setAck("servos released");
        });
      case "run_builtin":
        this.runBuiltin(command.name);
        this.setAck(`builtin ${command.name}`);
        return;
      case "set_leg_foot_xy":
        this.setLegFootTarget(command.legId, { x: command.x, y: command.y });
        this.setAck("foot target accepted");
        return;
      case "set_leg_joint_angles":
        this.setLegJointAngles(command.legId, { thigh: command.thighDeg, calf: command.calfDeg });
        this.setAck("joint target accepted");
        return;
      case "set_leg_servo_angles":
        this.setLegServoAngles(command.legId, { thigh: command.thighServoDeg, calf: command.calfServoDeg });
        this.setAck("servo target accepted");
        return;
      case "set_leg_servo_channel_map":
        this.setLegServoChannelMap(command.legId, command.thighChannel, command.calfChannel);
        this.setAck("servo channel map updated");
        return;
      case "set_leg_joint_limits":
        this.setLegJointLimits(command.legId, {
          thighDeg: { min: command.thighMinDeg, max: command.thighMaxDeg },
          calfDeg: { min: command.calfMinDeg, max: command.calfMaxDeg },
        });
        this.setAck("joint limits updated");
        return;
      case "play_animation":
        this.playAnimation(command.name);
        this.setAck("animation playing");
        return;
      case "pause_animation":
        this.pauseAnimation();
        this.setAck("animation paused");
        return;
      case "stop_animation":
        this.stopAnimation();
        this.setAck("animation stopped");
        return;
      case "hello":
      case "get_state":
        this.setAck(command.type === "hello" ? "hello" : "state");
        return;
      default:
        throw new Error(`Unsupported command type '${command.type}'.`);
    }
  }

  async tick() {
    const nowMs = Date.now();
    const elapsedSec = (nowMs - this.modeStartMs) / 1000;

    if (this.status.mode === "builtin_walk") {
      for (let index = 0; index < LEG_IDS.length; index += 1) {
        this.setLegFootTarget(LEG_IDS[index], builtinWalkTarget(index, elapsedSec), { internal: true });
      }
    }

    if (this.status.mode === "builtin_crouch") {
      const foot = builtinCrouchTarget(elapsedSec);
      for (const legId of LEG_IDS) {
        this.setLegFootTarget(legId, foot, { internal: true });
      }
    }

    if (this.status.mode === "animation_playback" && this.animation.clip && this.animation.playing && !this.animation.paused) {
      const timeSec = ((nowMs - this.animation.startMs) / 1000) % this.animation.clip.duration;
      const frame = interpolateFullBodyClip(this.animation.clip, timeSec);
      for (const legId of LEG_IDS) {
        this.setLegFootTarget(legId, frame[legId], { internal: true });
      }
    }

    await this.flushOutputs(nowMs);
    this.refreshCurrentState(nowMs);

    if ((nowMs - this.lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
      this.lastTelemetryMs = nowMs;
      this.broadcastEvent({ type: "state", payload: this.status });
    }

    if ((this.status.mode === "builtin_walk" || this.status.mode === "builtin_crouch") && (nowMs - this.lastBuiltinStatusMs) >= BUILTIN_STATUS_INTERVAL_MS) {
      this.lastBuiltinStatusMs = nowMs;
      this.broadcastEvent({
        type: "builtin_status",
        name: this.status.mode === "builtin_walk" ? "walk" : "crouch",
        mode: this.status.mode,
      });
    }

    if (this.status.mode === "animation_playback" && (nowMs - this.lastAnimationStatusMs) >= ANIMATION_STATUS_INTERVAL_MS) {
      this.lastAnimationStatusMs = nowMs;
      this.broadcastEvent({
        type: "animation_progress",
        name: this.status.activeAnimation,
        time: this.animation.playing && this.animation.clip
          ? ((nowMs - this.animation.startMs) / 1000) % this.animation.clip.duration
          : this.animation.pausedTimeSec,
      });
    }
  }
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

const controller = new RaspberryPiController();
const app = express();
const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: "/telemetry" });

controller.attachWebSocketServer(wss);

app.use(express.json({ limit: "1mb" }));

app.get("/api/status", async (_request, response) => {
  safeJson(response, 200, controller.status);
});

app.post("/api/connect", async (_request, response) => {
  try {
    await controller.connect();
    safeJson(response, 200, { ok: true, status: controller.status });
  } catch (error) {
    logControllerError("Connect failed", error);
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/disconnect", async (_request, response) => {
  try {
    await controller.disconnect();
    safeJson(response, 200, { ok: true, status: controller.status });
  } catch (error) {
    logControllerError("Disconnect failed", error);
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/command", async (request, response) => {
  try {
    await controller.handleCommand(request.body.command);
    safeJson(response, 200, { ok: true });
  } catch (error) {
    controller.setError(error.message);
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations", async (request, response) => {
  try {
    const clip = controller.uploadAnimation(request.body.clip);
    controller.setAck("animation uploaded");
    safeJson(response, 200, { ok: true, name: clip.name });
  } catch (error) {
    controller.setError(error.message);
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations/:id/play", async (request, response) => {
  try {
    const name = decodeURIComponent(request.params.id);
    controller.playAnimation(name);
    controller.setAck("animation playing");
    safeJson(response, 200, { ok: true });
  } catch (error) {
    controller.setError(error.message);
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations/:id/stop", async (_request, response) => {
  try {
    controller.stopAnimation();
    controller.setAck("animation stopped");
    safeJson(response, 200, { ok: true });
  } catch (error) {
    controller.setError(error.message);
    safeJson(response, 500, { error: error.message });
  }
});

wss.on("connection", (socket) => {
  socket.on("error", (error) => {
    logControllerError("WebSocket connection error", error);
  });
  safeSendWs(socket, JSON.stringify({ type: "status", payload: controller.status }));
});

server.on("clientError", (error, socket) => {
  logControllerError("HTTP client error", error);
  if (socket.writable) {
    socket.end("HTTP/1.1 400 Bad Request\r\n\r\n");
  }
});

server.on("error", (error) => {
  if (error?.code === "EADDRINUSE") {
    console.error(`Port ${PORT} is already in use. Stop the existing controller or use a different PORT.`);
  } else {
    console.error("Raspberry Pi controller failed to start:", error);
  }
  process.exit(1);
});

server.listen(PORT, async () => {
  console.log(`Robot dog Raspberry Pi controller listening on http://localhost:${PORT}`);
  try {
    await controller.connect();
  } catch (error) {
    logControllerError("Initial controller connect failed", error);
  }
});

process.on("unhandledRejection", (error) => {
  logControllerError("Unhandled rejection in Raspberry Pi controller", error);
});

process.on("uncaughtException", (error) => {
  logControllerError("Uncaught exception in Raspberry Pi controller", error);
});
