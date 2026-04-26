import express from "express";
import http from "http";
import { WebSocketServer } from "ws";
import { ReadlineParser, SerialPort } from "serialport";
import {
  buildLegPoseFromFoot,
  buildLegPoseFromJointAngles,
  buildLegPoseFromServoAngles,
  createNeutralCalibration,
  normalizeJointLimits,
} from "../shared/kinematics.js";
import { createUploadFrames, validateClip } from "../shared/animation.js";
import { createFullBodyPoseCommand, createMotionStatePatch } from "../shared/locomotion.js";
import {
  DEFAULT_DRIVE_COMMAND,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_LEG_COMMAND,
  DEFAULT_SERVO_CHANNEL_MAP,
  DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
  DEFAULT_SERVO_TRIM_DEG,
  DEFAULT_STANCE,
  LEG_IDS,
  ROBOT_CONFIG,
} from "../shared/robot-config.js";
import { normalizeDriveCommand, normalizeStance, parseWireMessage, toWireMessage, validateCommand } from "../shared/protocol.js";
import { extractJsonMessageCandidate } from "../shared/serial-wire.js";

const PORT = Number(process.env.PORT || 8787);
const calibration = createNeutralCalibration();

function logBridgeError(context, error) {
  if (!error) {
    return;
  }

  const code = error.code ? ` (${error.code})` : "";
  console.error(`${context}${code}: ${error.message}`);
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function safeSendWs(client, message) {
  try {
    if (client.readyState === 1) {
      client.send(message, (error) => {
        if (error) {
          logBridgeError("WebSocket send failed", error);
          try {
            client.close();
          } catch {
            // ignore close failures on a broken socket
          }
        }
      });
    }
  } catch (error) {
    logBridgeError("WebSocket send threw", error);
  }
}

function safeJson(response, statusCode, payload) {
  if (response.destroyed || response.writableEnded || response.headersSent) {
    return;
  }

  try {
    response.status(statusCode).json(payload);
  } catch (error) {
    logBridgeError("HTTP response write failed", error);
  }
}

function buildPoseFromPayload(pose, fallbackPose, jointLimits) {
  if (!pose) {
    return fallbackPose;
  }

  if (pose.foot) {
    return buildLegPoseFromFoot(pose.foot, calibration, {
      startThetaThigh: fallbackPose?.geometry?.thetaThigh,
      startThetaServo: fallbackPose?.geometry?.thetaServo,
      jointLimits,
      hipYawDeg: pose.jointAnglesDeg?.hipYaw ?? fallbackPose?.jointAnglesDeg?.hipYaw ?? 0,
    });
  }

  if (pose.jointAnglesDeg) {
    return buildLegPoseFromJointAngles(pose.jointAnglesDeg, calibration, {
      startThetaServo: fallbackPose?.geometry?.thetaServo,
      jointLimits,
    });
  }

  if (pose.servoAnglesDeg) {
    return buildLegPoseFromServoAngles(pose.servoAnglesDeg, calibration, {
      jointLimits,
    });
  }

  return fallbackPose;
}

async function handleRealtimeCommand(rawCommand, wss) {
  const command = validateCommand(rawCommand);
  bridge.applyDerivedLocalState(command);
  await bridge.forwardCommand(command);
  bridge.broadcast(wss, "status", bridge.status);
}

function createStatus() {
  const legs = {};
  for (const legId of LEG_IDS) {
    const desired = buildLegPoseFromFoot(DEFAULT_LEG_COMMAND.foot, calibration, {
      jointLimits: DEFAULT_JOINT_LIMITS,
      hipYawDeg: DEFAULT_LEG_COMMAND.jointAnglesDeg.hipYaw,
    });

    legs[legId] = {
      desired,
      current: clone(desired),
      status: "idle",
      lastError: null,
      servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
      jointLimits: clone(DEFAULT_JOINT_LIMITS),
      servoSpeedLimitDegPerSec: { ...DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC },
      servoTrimDeg: { ...DEFAULT_SERVO_TRIM_DEG },
    };
  }

  return {
    connected: false,
    esp32Connected: false,
    connectedPort: null,
    mode: "idle",
    motionMode: "idle",
    driveCommand: { ...DEFAULT_DRIVE_COMMAND },
    stance: { ...DEFAULT_STANCE },
    servosReleased: false,
    activeAnimation: null,
    lastAck: null,
    lastError: null,
    faults: [],
    firmwareMs: 0,
    uptimeMs: 0,
    ports: [],
    robotConfig: ROBOT_CONFIG,
    legs,
  };
}

class BridgeState {
  constructor() {
    this.status = createStatus();
    this.serial = null;
    this.parser = null;
    this.seq = 1;
    this.uploadedClips = new Map();
  }

  async refreshPorts() {
    const ports = await SerialPort.list();
    this.status.ports = ports.map((port) => ({
      path: port.path,
      manufacturer: port.manufacturer ?? "",
      serialNumber: port.serialNumber ?? "",
    }));
    return this.status.ports;
  }

  nextSeq() {
    const value = this.seq;
    this.seq += 1;
    return value;
  }

  broadcast(wss, type, payload) {
    const message = JSON.stringify({ type, ...(payload ? { payload } : {}) });
    for (const client of wss.clients) {
      safeSendWs(client, message);
    }
  }

  broadcastEvent(wss, event) {
    const message = JSON.stringify(event);
    for (const client of wss.clients) {
      safeSendWs(client, message);
    }
  }

  async connect({ path, baudRate }, wss) {
    await this.disconnect();
    this.serial = new SerialPort({ path, baudRate, autoOpen: false });

    await new Promise((resolve, reject) => {
      this.serial.open((error) => (error ? reject(error) : resolve()));
    });

    this.parser = this.serial.pipe(new ReadlineParser({ delimiter: "\n" }));
    this.parser.on("error", (error) => {
      this.status.lastError = error.message;
      logBridgeError("Serial parser error", error);
      this.broadcastEvent(wss, { type: "error", message: error.message });
    });
    this.parser.on("data", (line) => {
      const candidate = extractJsonMessageCandidate(line);
      if (!candidate) {
        return;
      }

      try {
        const message = parseWireMessage(candidate);
        this.handleIncomingMessage(message, wss);
      } catch (error) {
        this.status.lastError = error.message;
        this.broadcastEvent(wss, { type: "error", message: error.message });
      }
    });

    this.serial.on("error", (error) => {
      this.status.lastError = error.message;
      logBridgeError("Serial port error", error);
      this.broadcastEvent(wss, { type: "error", message: error.message });
    });

    this.serial.on("close", () => {
      this.status.connected = false;
      this.status.esp32Connected = false;
      this.status.connectedPort = null;
      this.broadcast(wss, "status", this.status);
    });

    this.status.connected = true;
    this.status.esp32Connected = true;
    this.status.connectedPort = path;
    await this.refreshPorts();
    this.broadcast(wss, "status", this.status);
    await this.send({ type: "hello" });
    await this.send({ type: "get_state" });
  }

  async disconnect() {
    if (!this.serial) {
      this.status.connected = false;
      this.status.esp32Connected = false;
      this.status.connectedPort = null;
      return;
    }

    const serial = this.serial;
    this.serial = null;
    this.parser = null;

    await new Promise((resolve) => serial.close(() => resolve()));
    this.status.connected = false;
    this.status.esp32Connected = false;
    this.status.connectedPort = null;
  }

  async send(command) {
    validateCommand(command);
    if (!this.serial || !this.status.connected) {
      throw new Error("Serial bridge is not connected.");
    }

    const line = `${toWireMessage(command, this.nextSeq())}\n`;
    await new Promise((resolve, reject) => {
      this.serial.write(line, (error) => (error ? reject(error) : resolve()));
    });
  }

  async sendIfConnected(command) {
    if (!this.serial || !this.status.connected) {
      return;
    }

    await this.send(command);
  }

  applyMotionPatch({ motionMode, driveCommand, stance, timeMs = Date.now() }) {
    const nextStance = normalizeStance(stance ?? this.status.stance);
    const nextDrive = normalizeDriveCommand(driveCommand ?? this.status.driveCommand);
    const jointLimitsByLeg = Object.fromEntries(
      LEG_IDS.map((legId) => [legId, this.status.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS]),
    );
    const patch = createMotionStatePatch({
      driveCommand: nextDrive,
      motionMode,
      stance: nextStance,
      timeMs,
      jointLimitsByLeg,
    });

    this.status.stance = nextStance;
    this.status.driveCommand = patch.driveCommand;
    this.status.motionMode = patch.motionMode;
    this.status.mode = motionMode === "drive" ? "drive" : motionMode === "stand" ? "stand" : this.status.mode;
    this.status.servosReleased = false;

    for (const legId of LEG_IDS) {
      this.status.legs[legId] = {
        ...this.status.legs[legId],
        ...patch.legs[legId],
      };
    }
  }

  buildFullBodyPoseCommand() {
    return createFullBodyPoseCommand(this.status.legs);
  }

  async forwardCommand(command) {
    validateCommand(command);
    if (command.type === "set_stance") {
      await this.sendIfConnected(this.buildFullBodyPoseCommand());
      return;
    }

    if (command.type === "set_motion_mode") {
      if (command.mode === "stand" || command.mode === "drive") {
        await this.sendIfConnected(this.buildFullBodyPoseCommand());
        return;
      }

      if (command.mode === "idle") {
        await this.sendIfConnected({ type: "set_mode", mode: "idle" });
      }
      return;
    }

    if (command.type === "set_drive_command") {
      await this.sendIfConnected(this.buildFullBodyPoseCommand());
      return;
    }

    if (command.type === "stop_motion") {
      await this.sendIfConnected({ type: "set_mode", mode: "idle" });
      return;
    }

    if (command.type === "set_leg_servo_trim") {
      if (!this.serial || !this.status.connected) {
        throw new Error("Serial bridge is not connected.");
      }
      await this.send(this.buildFullBodyPoseCommand());
      return;
    }

    await this.send(command);
  }

  async uploadAnimation(clip) {
    const validated = validateClip(clip);
    this.uploadedClips.set(validated.name, validated);
    for (const frame of createUploadFrames(validated)) {
      await this.send(frame);
    }
    this.status.activeAnimation = validated.name;
  }

  applyDerivedLocalState(command) {
    if (command.type === "release_servos" || command.type === "panic_release") {
      this.status.mode = "idle";
      this.status.motionMode = "idle";
      this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND };
      this.status.servosReleased = true;
      this.status.activeAnimation = null;
      return;
    }

    if (command.type === "run_builtin") {
      this.status.mode = command.name === "walk" ? "builtin_walk" : "builtin_crouch";
      this.status.servosReleased = false;
      return;
    }

    if (command.type === "set_motion_mode") {
      if (command.stance) {
        this.status.stance = command.stance;
      }

      if (command.mode === "stand" || command.mode === "drive") {
        this.applyMotionPatch({
          motionMode: command.mode,
          driveCommand: command.mode === "drive" ? this.status.driveCommand : DEFAULT_DRIVE_COMMAND,
          stance: this.status.stance,
        });
        return;
      }

      this.status.motionMode = command.mode;
      if (command.mode === "idle") {
        this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND };
        this.status.mode = "idle";
      }
      this.status.servosReleased = false;
      return;
    }

    if (command.type === "set_drive_command") {
      this.applyMotionPatch({
        motionMode: "drive",
        driveCommand: command.drive ?? command,
        stance: command.stance ?? this.status.stance,
      });
      return;
    }

    if (command.type === "set_stance") {
      const nextMotionMode = this.status.motionMode === "drive" ? "drive" : "stand";
      this.applyMotionPatch({
        motionMode: nextMotionMode,
        driveCommand: nextMotionMode === "drive" ? this.status.driveCommand : DEFAULT_DRIVE_COMMAND,
        stance: command.stance,
      });
      return;
    }

    if (command.type === "stop_motion") {
      this.status.motionMode = "idle";
      this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND };
      if (this.status.mode === "drive") {
        this.status.mode = "idle";
      }
      return;
    }

    if (command.type === "set_mode") {
      this.status.mode = command.mode;
      if (command.mode === "idle") {
        this.status.motionMode = "idle";
      } else {
        this.status.servosReleased = false;
      }
      return;
    }

    if (!command.legId || !this.status.legs[command.legId]) {
      return;
    }

    const currentDesired = this.status.legs[command.legId].desired;
    const jointLimits = this.status.legs[command.legId].jointLimits ?? DEFAULT_JOINT_LIMITS;

    if (command.type === "set_leg_foot_xy") {
      this.status.mode = "direct_foot_xy";
      this.status.servosReleased = false;
      this.status.legs[command.legId].desired = buildLegPoseFromFoot({ x: command.x, y: command.y }, calibration, {
        startThetaThigh: currentDesired?.geometry?.thetaThigh,
        startThetaServo: currentDesired?.geometry?.thetaServo,
        jointLimits,
        hipYawDeg: currentDesired?.jointAnglesDeg?.hipYaw ?? 0,
      });
    }

    if (command.type === "set_leg_joint_angles") {
      this.status.mode = "direct_joint_angles";
      this.status.servosReleased = false;
      this.status.legs[command.legId].desired = buildLegPoseFromJointAngles(
        {
          hipYaw: command.hipYawDeg ?? currentDesired?.jointAnglesDeg?.hipYaw ?? 0,
          thigh: command.thighDeg,
          calf: command.calfDeg,
        },
        calibration,
        {
          startThetaServo: currentDesired?.geometry?.thetaServo,
          jointLimits,
        },
      );
    }

    if (command.type === "set_leg_servo_angles") {
      this.status.mode = "direct_servo_angles";
      this.status.servosReleased = false;
      this.status.legs[command.legId].desired = buildLegPoseFromServoAngles(
        {
          hipYaw: command.hipYawServoDeg ?? currentDesired?.servoAnglesDeg?.hipYaw ?? 90,
          thigh: command.thighServoDeg,
          calf: command.calfServoDeg,
        },
        calibration,
        {
          jointLimits,
        },
      );
    }

    if (command.type === "set_leg_servo_channel_map") {
      this.status.legs[command.legId].servoChannelMap = {
        hipYaw: command.hipYawChannel,
        thigh: command.thighChannel,
        calf: command.calfChannel,
      };
    }

    if (command.type === "set_leg_joint_limits") {
      const normalizedLimits = normalizeJointLimits({
        hipYawDeg: {
          min: command.hipYawMinDeg,
          max: command.hipYawMaxDeg,
        },
        thighDeg: {
          min: command.thighMinDeg,
          max: command.thighMaxDeg,
        },
        calfDeg: {
          min: command.calfMinDeg,
          max: command.calfMaxDeg,
        },
      });

      this.status.legs[command.legId].jointLimits = normalizedLimits;
      this.status.legs[command.legId].desired = buildLegPoseFromFoot(currentDesired?.foot ?? DEFAULT_LEG_COMMAND.foot, calibration, {
        startThetaThigh: currentDesired?.geometry?.thetaThigh,
        startThetaServo: currentDesired?.geometry?.thetaServo,
        jointLimits: normalizedLimits,
        hipYawDeg: currentDesired?.jointAnglesDeg?.hipYaw ?? 0,
      });
    }

    if (command.type === "set_leg_servo_speed_limit") {
      this.status.legs[command.legId].servoSpeedLimitDegPerSec = {
        hipYaw: command.hipYawDegPerSec,
        thigh: command.thighDegPerSec,
        calf: command.calfDegPerSec,
      };
    }

    if (command.type === "set_leg_servo_trim") {
      this.status.servosReleased = false;
      this.status.legs[command.legId].servoTrimDeg = {
        hipYaw: command.hipYawOffsetDeg,
        thigh: command.thighOffsetDeg,
        calf: command.calfOffsetDeg,
      };
    }
  }

  normalizeStatePayload(payload) {
    const next = clone(this.status);
    next.connected = payload.connected ?? this.status.connected;
    next.esp32Connected = payload.esp32Connected ?? payload.connected ?? next.connected;
    next.connectedPort = payload.connectedPort ?? this.status.connectedPort;
    next.ports = payload.ports ?? this.status.ports;
    next.mode = payload.mode ?? next.mode;
    next.motionMode = payload.motionMode ?? next.motionMode;
    next.driveCommand = payload.driveCommand ? normalizeDriveCommand(payload.driveCommand) : next.driveCommand;
    next.stance = payload.stance ? normalizeStance(payload.stance) : next.stance;
    next.servosReleased = payload.servosReleased ?? next.servosReleased;
    next.activeAnimation = payload.activeAnimation ?? next.activeAnimation;
    next.lastAck = payload.lastAck ?? next.lastAck;
    next.lastError = payload.lastError ?? next.lastError;
    next.faults = Array.isArray(payload.faults) ? payload.faults : next.faults;
    next.firmwareMs = payload.firmwareMs ?? next.firmwareMs;
    next.uptimeMs = payload.uptimeMs ?? next.uptimeMs;
    next.robotConfig = payload.robotConfig ?? next.robotConfig;

    if (payload.legs) {
      for (const legId of LEG_IDS) {
        const leg = payload.legs[legId];
        if (!leg) {
          continue;
        }

        const normalizedLimits = normalizeJointLimits(leg.jointLimits ?? next.legs[legId].jointLimits);
        next.legs[legId] = {
          ...next.legs[legId],
          ...leg,
          servoChannelMap: {
            ...next.legs[legId].servoChannelMap,
            ...(leg.servoChannelMap ?? {}),
          },
          jointLimits: normalizedLimits,
          servoSpeedLimitDegPerSec: {
            ...next.legs[legId].servoSpeedLimitDegPerSec,
            ...(leg.servoSpeedLimitDegPerSec ?? {}),
          },
          servoTrimDeg: {
            ...next.legs[legId].servoTrimDeg,
            ...(leg.servoTrimDeg ?? {}),
          },
          desired: buildPoseFromPayload(leg.desired, next.legs[legId].desired, normalizedLimits),
          current: buildPoseFromPayload(leg.current, next.legs[legId].current, normalizedLimits),
        };
      }
    }

    return next;
  }

  handleIncomingMessage(message, wss) {
    if (message.type === "state" || message.type === "hello_ack") {
      this.status = this.normalizeStatePayload(message.payload ?? {});
      this.broadcastEvent(wss, { type: message.type, payload: this.status });
      return;
    }

    if (message.type === "ack") {
      this.status.lastAck = message.message ?? String(message.seq ?? "ack");
      this.broadcastEvent(wss, message);
      return;
    }

    if (message.type === "error") {
      this.status.lastError = message.message ?? "Unknown firmware error";
      this.broadcastEvent(wss, message);
      return;
    }

    if (message.type === "animation_progress") {
      this.status.activeAnimation = message.name ?? this.status.activeAnimation;
      this.broadcastEvent(wss, message);
      return;
    }

    this.broadcastEvent(wss, message);
  }
}

const bridge = new BridgeState();
const app = express();
const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: "/telemetry" });

app.use((request, response, next) => {
  const origin = request.headers.origin ?? "*";
  response.header("Access-Control-Allow-Origin", origin);
  response.header("Vary", "Origin");
  response.header("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  response.header("Access-Control-Allow-Headers", "Content-Type");
  if (request.method === "OPTIONS") {
    response.sendStatus(204);
    return;
  }
  next();
});

app.use(express.json({ limit: "1mb" }));

app.get("/api/status", async (_request, response) => {
  try {
    await bridge.refreshPorts();
    safeJson(response, 200, bridge.status);
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/connect", async (request, response) => {
  try {
    const path = String(request.body.path || "");
    const baudRate = Number(request.body.baudRate || 460800);
    await bridge.connect({ path, baudRate }, wss);
    safeJson(response, 200, { ok: true, status: bridge.status });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/disconnect", async (_request, response) => {
  try {
    await bridge.disconnect();
    bridge.broadcast(wss, "status", bridge.status);
    safeJson(response, 200, { ok: true, status: bridge.status });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/command", async (request, response) => {
  try {
    const command = validateCommand(request.body.command);
    bridge.applyDerivedLocalState(command);
    await bridge.forwardCommand(command);
    bridge.broadcast(wss, "status", bridge.status);
    safeJson(response, 200, { ok: true });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations", async (request, response) => {
  try {
    const clip = validateClip(request.body.clip);
    await bridge.uploadAnimation(clip);
    bridge.broadcast(wss, "status", bridge.status);
    safeJson(response, 200, { ok: true, name: clip.name });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations/:id/play", async (request, response) => {
  try {
    const name = decodeURIComponent(request.params.id);
    if (!bridge.uploadedClips.has(name)) {
      throw new Error(`Animation '${name}' has not been uploaded.`);
    }
    await bridge.send({ type: "play_animation", name });
    bridge.status.activeAnimation = name;
    bridge.status.mode = "animation_playback";
    bridge.status.servosReleased = false;
    bridge.broadcast(wss, "status", bridge.status);
    safeJson(response, 200, { ok: true });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations/:id/stop", async (request, response) => {
  try {
    const name = decodeURIComponent(request.params.id);
    await bridge.send({ type: "stop_animation", name });
    bridge.status.mode = "idle";
    bridge.broadcast(wss, "status", bridge.status);
    safeJson(response, 200, { ok: true });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

wss.on("connection", (socket) => {
  socket.on("error", (error) => {
    logBridgeError("WebSocket connection error", error);
  });
  socket.on("message", async (data) => {
    try {
      const message = JSON.parse(String(data));
      if (message?.type !== "command") {
        return;
      }
      await handleRealtimeCommand(message.command, wss);
    } catch (error) {
      bridge.status.lastError = error.message;
      logBridgeError("WebSocket command failed", error);
      safeSendWs(socket, JSON.stringify({ type: "error", message: error.message }));
    }
  });
  safeSendWs(socket, JSON.stringify({ type: "status", payload: bridge.status }));
});

server.on("clientError", (error, socket) => {
  logBridgeError("HTTP client error", error);
  if (socket.writable) {
    socket.end("HTTP/1.1 400 Bad Request\r\n\r\n");
  }
});

server.on("error", (error) => {
  if (error?.code === "EADDRINUSE") {
    console.error(`Port ${PORT} is already in use. Reuse the existing bridge or stop the old process before starting another one.`);
  } else {
    console.error("Bridge failed to start:", error);
  }
  process.exit(1);
});

server.listen(PORT, async () => {
  await bridge.refreshPorts();
  console.log(`Robot dog bridge listening on http://localhost:${PORT}`);
});

process.on("unhandledRejection", (error) => {
  logBridgeError("Unhandled rejection in bridge", error);
});

process.on("uncaughtException", (error) => {
  logBridgeError("Uncaught exception in bridge", error);
});
