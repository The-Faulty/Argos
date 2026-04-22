import express from "express";
import http from "http";
import path from "path";
import { fileURLToPath } from "url";
import { WebSocketServer } from "ws";
import { ReadlineParser, SerialPort } from "serialport";
import { buildLegPoseFromFoot, buildLegPoseFromJointAngles, buildLegPoseFromServoAngles, createNeutralCalibration, normalizeJointLimits } from "../shared/kinematics.js";
import { createMotionStatePatch, flattenServoPose } from "../shared/locomotion.js";
import { parseWireMessage, toWireMessage, validateCommand } from "../shared/protocol.js";
import {
  DEFAULT_DRIVE_COMMAND,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_LEG_COMMAND,
  DEFAULT_SERVO_CHANNEL_MAP,
  DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
  LEG_IDS,
  ROBOT_CONFIG,
} from "../shared/robot-config.js";

const PORT = Number(process.env.PORT || 8787);
const BAUD_RATE = Number(process.env.PI_ESP32_BAUD || 460800);
const CONTROL_INTERVAL_MS = 50;
const TELEMETRY_INTERVAL_MS = 100;
const UI_DRIVE_TIMEOUT_MS = Number(process.env.PI_UI_DRIVE_TIMEOUT_MS || 250);
const AUTO_CONNECT_PORT = process.env.PI_ESP32_PORT?.trim() || "";
const calibration = createNeutralCalibration();
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const distDir = path.resolve(__dirname, "..", "dist");

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function safeJson(response, statusCode, payload) {
  if (response.destroyed || response.writableEnded || response.headersSent) {
    return;
  }
  response.status(statusCode).json(payload);
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
      client.send(message);
    }
  } catch (error) {
    logControllerError("WebSocket send failed", error);
  }
}

function createLegStatus() {
  const desired = buildLegPoseFromFoot(DEFAULT_LEG_COMMAND.foot, calibration, {
    jointLimits: DEFAULT_JOINT_LIMITS,
    hipYawDeg: DEFAULT_LEG_COMMAND.jointAnglesDeg.hipYaw,
  });

  return {
    desired,
    current: desired,
    status: "idle",
    lastError: "",
    servoChannelMap: {},
    jointLimits: clone(DEFAULT_JOINT_LIMITS),
    servoSpeedLimitDegPerSec: {},
  };
}

function createStatus() {
  const legs = {};
  for (const legId of LEG_IDS) {
    legs[legId] = {
      ...createLegStatus(),
      servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
      servoSpeedLimitDegPerSec: { ...DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC },
    };
  }

  return {
    connected: false,
    esp32Connected: false,
    connectedPort: null,
    mode: "idle",
    motionMode: "idle",
    driveCommand: { ...DEFAULT_DRIVE_COMMAND },
    servosReleased: false,
    activeAnimation: null,
    lastAck: null,
    lastError: null,
    faults: [],
    firmwareMs: 0,
    uptimeMs: 0,
    ports: [],
    robotConfig: clone(ROBOT_CONFIG),
    legs,
  };
}

class PiDogController {
  constructor() {
    this.status = createStatus();
    this.serial = null;
    this.parser = null;
    this.wss = null;
    this.seq = 1;
    this.bootMs = Date.now();
    this.lastTelemetryMs = 0;
    this.lastUiDriveAtMs = 0;
    this.controlHandle = null;
  }

  attachWebSocketServer(wss) {
    this.wss = wss;
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

  broadcastEvent(event) {
    if (!this.wss) {
      return;
    }
    const message = JSON.stringify(event);
    for (const client of this.wss.clients) {
      safeSendWs(client, message);
    }
  }

  setFault(message) {
    this.status.lastError = message;
    if (message && !this.status.faults.includes(message)) {
      this.status.faults = [...this.status.faults, message].slice(-8);
    }
    this.broadcastEvent({ type: "error", message });
  }

  clearFault(message) {
    this.status.faults = this.status.faults.filter((fault) => fault !== message);
    if (this.status.lastError === message) {
      this.status.lastError = this.status.faults.at(-1) ?? null;
    }
  }

  setAck(message) {
    this.status.lastAck = message;
    this.broadcastEvent({ type: "ack", message });
  }

  nextSeq() {
    const value = this.seq;
    this.seq += 1;
    return value;
  }

  async sendWire(command) {
    if (!this.serial || !this.status.esp32Connected) {
      throw new Error("ESP32-C6 is not connected.");
    }

    const line = `${toWireMessage(command, this.nextSeq())}\n`;
    await new Promise((resolve, reject) => {
      this.serial.write(line, (error) => (error ? reject(error) : resolve()));
    });
  }

  async connect({ path: serialPath, baudRate = BAUD_RATE }) {
    await this.disconnect();
    await this.refreshPorts();

    this.serial = new SerialPort({ path: serialPath, baudRate, autoOpen: false });
    await new Promise((resolve, reject) => {
      this.serial.open((error) => (error ? reject(error) : resolve()));
    });

    this.parser = this.serial.pipe(new ReadlineParser({ delimiter: "\n" }));
    this.parser.on("data", (line) => this.handleIncomingLine(String(line).trim()));
    this.parser.on("error", (error) => this.setFault(error.message));
    this.serial.on("error", (error) => this.setFault(error.message));
    this.serial.on("close", () => this.handleSerialClosed());

    this.status.connected = true;
    this.status.esp32Connected = true;
    this.status.connectedPort = serialPath;
    this.clearFault("ESP32 link disconnected");
    this.broadcastStatus();
    await this.sendWire({ type: "hello" });
    await this.sendWire({ type: "get_state" });
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
    this.handleSerialClosed();
  }

  handleSerialClosed() {
    this.status.connected = false;
    this.status.esp32Connected = false;
    this.status.connectedPort = null;
    this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND, updatedAt: Date.now() };
    this.status.motionMode = "idle";
    this.status.mode = "idle";
    this.setFault("ESP32 link disconnected");
    this.broadcastStatus();
  }

  broadcastStatus() {
    this.broadcastEvent({ type: "status", payload: this.status });
  }

  handleIncomingLine(line) {
    if (!line) {
      return;
    }

    try {
      const message = parseWireMessage(line);
      if (message.type === "state" || message.type === "hello_ack") {
        this.applyIncomingState(message.payload ?? {});
        this.broadcastEvent({ type: message.type, payload: this.status });
        return;
      }

      if (message.type === "ack") {
        this.setAck(message.message ?? "ack");
        return;
      }

      if (message.type === "error") {
        this.setFault(message.message ?? "Unknown ESP32 error");
      }
    } catch (error) {
      this.setFault(error.message);
    }
  }

  applyIncomingState(payload) {
    this.status.firmwareMs = payload.firmwareMs ?? Date.now();
    this.status.servosReleased = payload.servosReleased ?? this.status.servosReleased;
    this.status.activeAnimation = payload.activeAnimation ?? this.status.activeAnimation;
    this.status.lastAck = payload.lastAck ?? this.status.lastAck;
    this.status.lastError = payload.lastError ?? this.status.lastError;
    if (payload.legs) {
      for (const legId of LEG_IDS) {
        const incomingLeg = payload.legs[legId];
        if (!incomingLeg) {
          continue;
        }

        const jointLimits = normalizeJointLimits(incomingLeg.jointLimits ?? this.status.legs[legId].jointLimits);
        this.status.legs[legId] = {
          ...this.status.legs[legId],
          ...incomingLeg,
          jointLimits,
          servoChannelMap: incomingLeg.servoChannelMap ?? this.status.legs[legId].servoChannelMap,
          servoSpeedLimitDegPerSec: incomingLeg.servoSpeedLimitDegPerSec ?? this.status.legs[legId].servoSpeedLimitDegPerSec,
          desired: incomingLeg.desired?.servoAnglesDeg
            ? buildLegPoseFromServoAngles(incomingLeg.desired.servoAnglesDeg, calibration, { jointLimits })
            : incomingLeg.desired?.jointAnglesDeg
              ? buildLegPoseFromJointAngles(incomingLeg.desired.jointAnglesDeg, calibration, { jointLimits })
              : this.status.legs[legId].desired,
          current: incomingLeg.current?.servoAnglesDeg
            ? buildLegPoseFromServoAngles(incomingLeg.current.servoAnglesDeg, calibration, { jointLimits })
            : this.status.legs[legId].current,
        };
      }
    }
  }

  applyDrivePatch(timeMs) {
    const jointLimitsByLeg = Object.fromEntries(
      LEG_IDS.map((legId) => [legId, this.status.legs[legId].jointLimits ?? DEFAULT_JOINT_LIMITS]),
    );
    const patch = createMotionStatePatch({
      driveCommand: this.status.driveCommand,
      motionMode: this.status.motionMode,
      timeMs,
      jointLimitsByLeg,
    });

    this.status.driveCommand = patch.driveCommand;
    for (const legId of LEG_IDS) {
      this.status.legs[legId] = {
        ...this.status.legs[legId],
        desired: patch.legs[legId].desired,
        current: patch.legs[legId].desired,
        status: patch.legs[legId].status,
      };
    }
  }

  async sendCurrentPoseToEsp32() {
    const legs = Object.fromEntries(
      LEG_IDS.map((legId) => [legId, { servoAnglesDeg: this.status.legs[legId].desired.servoAnglesDeg }]),
    );
    await this.sendWire({
      type: "apply_full_body_pose",
      legs,
      ...flattenServoPose(this.status.legs),
    });
  }

  async handleCommand(rawCommand) {
    const command = validateCommand(rawCommand);
    const nowMs = Date.now();

    switch (command.type) {
      case "set_drive_command":
        this.status.driveCommand = {
          ...command.drive,
          updatedAt: nowMs,
        };
        this.status.motionMode = "drive";
        this.status.mode = "drive";
        this.status.servosReleased = false;
        this.lastUiDriveAtMs = nowMs;
        this.setAck("drive command updated");
        break;
      case "set_motion_mode":
        this.status.motionMode = command.mode;
        this.status.mode = command.mode;
        this.status.servosReleased = false;
        if (command.mode !== "drive") {
          this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND, updatedAt: nowMs };
        }
        this.setAck(`motion mode ${command.mode}`);
        break;
      case "stop_motion":
        this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND, updatedAt: nowMs, source: "stop" };
        this.status.motionMode = "stand";
        this.status.mode = "stand";
        this.setAck("motion stopped");
        break;
      case "panic_release":
      case "release_servos":
        this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND, updatedAt: nowMs, source: "panic" };
        this.status.motionMode = "idle";
        this.status.mode = "idle";
        this.status.servosReleased = true;
        if (this.status.esp32Connected) {
          await this.sendWire({ type: "release_servos" });
        }
        this.setAck("servos released");
        return;
      case "set_leg_foot_xy": {
        const legStatus = this.status.legs[command.legId];
        legStatus.desired = buildLegPoseFromFoot({ x: command.x, y: command.y }, calibration, {
          jointLimits: legStatus.jointLimits,
          hipYawDeg: legStatus.desired?.jointAnglesDeg?.hipYaw ?? 0,
        });
        legStatus.current = legStatus.desired;
        this.status.motionMode = "calibration";
        this.status.mode = "direct_foot_xy";
        this.status.servosReleased = false;
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck("foot target accepted");
        return;
      }
      case "set_leg_joint_angles": {
        const legStatus = this.status.legs[command.legId];
        legStatus.desired = buildLegPoseFromJointAngles(
          { hipYaw: command.hipYawDeg ?? 0, thigh: command.thighDeg, calf: command.calfDeg },
          calibration,
          { jointLimits: legStatus.jointLimits },
        );
        legStatus.current = legStatus.desired;
        this.status.motionMode = "calibration";
        this.status.mode = "direct_joint_angles";
        this.status.servosReleased = false;
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck("joint target accepted");
        return;
      }
      case "set_leg_servo_angles": {
        const legStatus = this.status.legs[command.legId];
        legStatus.desired = buildLegPoseFromServoAngles(
          { hipYaw: command.hipYawServoDeg ?? 90, thigh: command.thighServoDeg, calf: command.calfServoDeg },
          calibration,
          { jointLimits: legStatus.jointLimits },
        );
        legStatus.current = legStatus.desired;
        this.status.motionMode = "calibration";
        this.status.mode = "direct_servo_angles";
        this.status.servosReleased = false;
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck("servo target accepted");
        return;
      }
      case "set_leg_servo_channel_map":
        this.status.legs[command.legId].servoChannelMap = {
          hipYaw: command.hipYawChannel,
          thigh: command.thighChannel,
          calf: command.calfChannel,
        };
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck("servo channel map updated");
        return;
      case "set_leg_joint_limits":
        this.status.legs[command.legId].jointLimits = normalizeJointLimits({
          hipYawDeg: { min: command.hipYawMinDeg, max: command.hipYawMaxDeg },
          thighDeg: { min: command.thighMinDeg, max: command.thighMaxDeg },
          calfDeg: { min: command.calfMinDeg, max: command.calfMaxDeg },
        });
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck("joint limits updated");
        return;
      case "set_leg_servo_speed_limit":
        this.status.legs[command.legId].servoSpeedLimitDegPerSec = {
          hipYaw: command.hipYawDegPerSec,
          thigh: command.thighDegPerSec,
          calf: command.calfDegPerSec,
        };
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck("servo speed limit updated");
        return;
      default:
        if (this.status.esp32Connected) {
          await this.sendWire(command);
        }
        this.setAck(command.type);
        return;
    }

    this.applyDrivePatch(nowMs);
    if (this.status.esp32Connected && !this.status.servosReleased) {
      await this.sendCurrentPoseToEsp32();
    }
  }

  startLoop() {
    if (this.controlHandle) {
      return;
    }

    this.controlHandle = setInterval(() => {
      void this.tick().catch((error) => {
        this.setFault(error.message);
      });
    }, CONTROL_INTERVAL_MS);
  }

  async tick() {
    const nowMs = Date.now();
    this.status.uptimeMs = nowMs - this.bootMs;
    this.status.firmwareMs = nowMs;

    if (this.status.motionMode === "drive" && (nowMs - this.lastUiDriveAtMs) > UI_DRIVE_TIMEOUT_MS) {
      this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND, source: "timeout", updatedAt: nowMs };
    }

    if ((this.status.motionMode === "drive" || this.status.motionMode === "stand") && !this.status.servosReleased) {
      this.applyDrivePatch(nowMs);
      if (this.status.esp32Connected) {
        await this.sendCurrentPoseToEsp32();
      }
    }

    if ((nowMs - this.lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
      this.lastTelemetryMs = nowMs;
      this.broadcastEvent({ type: "state", payload: this.status });
    }
  }
}

const controller = new PiDogController();
const app = express();
const server = http.createServer(app);
const wss = new WebSocketServer({ server, path: "/telemetry" });

controller.attachWebSocketServer(wss);
controller.startLoop();

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
    await controller.refreshPorts();
    safeJson(response, 200, controller.status);
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/connect", async (request, response) => {
  try {
    await controller.connect({
      path: String(request.body.path || ""),
      baudRate: Number(request.body.baudRate || BAUD_RATE),
    });
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
    safeJson(response, 200, { ok: true, status: controller.status });
  } catch (error) {
    controller.setFault(error.message);
    safeJson(response, 500, { error: error.message });
  }
});

if (express.static && path.isAbsolute(distDir)) {
  app.use(express.static(distDir));
  app.get("*", (request, response, next) => {
    if (request.path.startsWith("/api") || request.path === "/telemetry") {
      next();
      return;
    }
    response.sendFile(path.join(distDir, "index.html"));
  });
}

wss.on("connection", (socket) => {
  safeSendWs(socket, JSON.stringify({ type: "status", payload: controller.status }));
  socket.on("message", async (data) => {
    try {
      const message = JSON.parse(String(data));
      if (message?.type !== "command") {
        return;
      }
      await controller.handleCommand(message.command);
      controller.broadcastStatus();
    } catch (error) {
      controller.setFault(error.message);
      safeSendWs(socket, JSON.stringify({ type: "error", message: error.message }));
    }
  });
});

server.listen(PORT, async () => {
  console.log(`Robot dog Pi controller listening on http://0.0.0.0:${PORT}`);
  try {
    await controller.refreshPorts();
    if (AUTO_CONNECT_PORT) {
      await controller.connect({ path: AUTO_CONNECT_PORT, baudRate: BAUD_RATE });
    }
  } catch (error) {
    logControllerError("Initial ESP32 connect failed", error);
  }
});
