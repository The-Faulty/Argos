import express from "express";
import http from "http";
import { WebSocketServer } from "ws";
import { ReadlineParser, SerialPort } from "serialport";
import { buildLegPoseFromFoot, buildLegPoseFromJointAngles, buildLegPoseFromServoAngles, createNeutralCalibration, normalizeJointLimits } from "../shared/kinematics.js";
import { createUploadFrames, validateClip } from "../shared/animation.js";
import { DEFAULT_JOINT_LIMITS, DEFAULT_LEG_COMMAND, DEFAULT_SERVO_CHANNEL_MAP, LEG_IDS } from "../shared/robot-config.js";
import { parseWireMessage, toWireMessage, validateCommand } from "../shared/protocol.js";

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

function createStatus() {
  const legs = {};
  for (const legId of LEG_IDS) {
    const desired = buildLegPoseFromFoot(DEFAULT_LEG_COMMAND.foot, calibration);
    legs[legId] = {
      desired,
      current: desired,
      status: "idle",
      lastError: null,
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
    ports: [],
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
      try {
        const message = parseWireMessage(String(line).trim());
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
      this.status.connectedPort = null;
      this.broadcast(wss, "status", this.status);
    });

    this.status.connected = true;
    this.status.connectedPort = path;
    await this.refreshPorts();
    this.broadcast(wss, "status", this.status);
    await this.send({ type: "hello" });
    await this.send({ type: "get_state" });
  }

  async disconnect() {
    if (!this.serial) {
      this.status.connected = false;
      this.status.connectedPort = null;
      return;
    }

    const serial = this.serial;
    this.serial = null;
    this.parser = null;

    await new Promise((resolve) => serial.close(() => resolve()));
    this.status.connected = false;
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

  async uploadAnimation(clip) {
    const validated = validateClip(clip);
    this.uploadedClips.set(validated.name, validated);
    for (const frame of createUploadFrames(validated)) {
      await this.send(frame);
    }
    this.status.activeAnimation = validated.name;
  }

  applyDerivedLocalState(command) {
    if (command.type === "release_servos") {
      this.status.mode = "idle";
      this.status.servosReleased = true;
      this.status.activeAnimation = null;
      return;
    }

    if (command.type === "run_builtin") {
      this.status.mode = command.name === "walk" ? "builtin_walk" : "builtin_crouch";
      this.status.servosReleased = false;
      return;
    }

    if (command.type === "set_mode") {
      this.status.mode = command.mode;
      if (command.mode !== "idle") {
        this.status.servosReleased = false;
      }
      return;
    }

    if (!command.legId || !this.status.legs[command.legId]) {
      return;
    }

    if (command.type === "set_leg_foot_xy") {
      this.status.mode = "direct_foot_xy";
      this.status.servosReleased = false;
      const currentDesired = this.status.legs[command.legId].desired;
      const jointLimits = this.status.legs[command.legId].jointLimits ?? DEFAULT_JOINT_LIMITS;
      this.status.legs[command.legId].desired = buildLegPoseFromFoot({ x: command.x, y: command.y }, calibration, {
        startThetaThigh: currentDesired?.geometry?.thetaThigh,
        startThetaServo: currentDesired?.geometry?.thetaServo,
        jointLimits,
      });
    }

    if (command.type === "set_leg_joint_angles") {
      this.status.mode = "direct_joint_angles";
      this.status.servosReleased = false;
      const currentDesired = this.status.legs[command.legId].desired;
      const jointLimits = this.status.legs[command.legId].jointLimits ?? DEFAULT_JOINT_LIMITS;
      this.status.legs[command.legId].desired = buildLegPoseFromJointAngles({ thigh: command.thighDeg, calf: command.calfDeg }, calibration, {
        startThetaServo: currentDesired?.geometry?.thetaServo,
        jointLimits,
      });
    }

    if (command.type === "set_leg_servo_angles") {
      this.status.mode = "direct_servo_angles";
      this.status.servosReleased = false;
      const jointLimits = this.status.legs[command.legId].jointLimits ?? DEFAULT_JOINT_LIMITS;
      this.status.legs[command.legId].desired = buildLegPoseFromServoAngles({ thigh: command.thighServoDeg, calf: command.calfServoDeg }, calibration, {
        jointLimits,
      });
    }

    if (command.type === "set_leg_servo_channel_map") {
      this.status.legs[command.legId].servoChannelMap = {
        thigh: command.thighChannel,
        calf: command.calfChannel,
      };
    }

    if (command.type === "set_leg_joint_limits") {
      const jointLimits = normalizeJointLimits({
        thighDeg: {
          min: command.thighMinDeg,
          max: command.thighMaxDeg,
        },
        calfDeg: {
          min: command.calfMinDeg,
          max: command.calfMaxDeg,
        },
      });
      const currentDesired = this.status.legs[command.legId].desired;
      this.status.legs[command.legId].jointLimits = jointLimits;
      this.status.legs[command.legId].desired = buildLegPoseFromFoot(currentDesired?.foot ?? DEFAULT_LEG_COMMAND.foot, calibration, {
        startThetaThigh: currentDesired?.geometry?.thetaThigh,
        startThetaServo: currentDesired?.geometry?.thetaServo,
        jointLimits,
      });
    }

  }

  normalizeStatePayload(payload) {
    const next = clone(this.status);
    next.connected = this.status.connected;
    next.connectedPort = this.status.connectedPort;
    next.ports = this.status.ports;
    next.mode = payload.mode ?? next.mode;
    next.servosReleased = payload.servosReleased ?? next.servosReleased;
    next.activeAnimation = payload.activeAnimation ?? next.activeAnimation;
    next.lastAck = payload.lastAck ?? next.lastAck;
    next.lastError = payload.lastError ?? next.lastError;
    next.firmwareMs = payload.firmwareMs ?? next.firmwareMs;

    if (payload.legs) {
      for (const legId of LEG_IDS) {
        const leg = payload.legs[legId];
        if (!leg) continue;

        next.legs[legId] = {
          ...next.legs[legId],
          ...leg,
          servoChannelMap: leg.servoChannelMap ?? next.legs[legId].servoChannelMap,
          jointLimits: normalizeJointLimits(leg.jointLimits ?? next.legs[legId].jointLimits),
          desired: leg.desired?.foot
            ? buildLegPoseFromFoot(leg.desired.foot, calibration, {
                startThetaThigh: next.legs[legId].desired?.geometry?.thetaThigh,
                startThetaServo: next.legs[legId].desired?.geometry?.thetaServo,
                jointLimits: leg.jointLimits ?? next.legs[legId].jointLimits,
              })
            : leg.desired?.jointAnglesDeg
              ? buildLegPoseFromJointAngles(leg.desired.jointAnglesDeg, calibration, {
                  startThetaServo: next.legs[legId].desired?.geometry?.thetaServo,
                  jointLimits: leg.jointLimits ?? next.legs[legId].jointLimits,
                })
              : leg.desired?.servoAnglesDeg
                ? buildLegPoseFromServoAngles(leg.desired.servoAnglesDeg, calibration, {
                    jointLimits: leg.jointLimits ?? next.legs[legId].jointLimits,
                  })
                : next.legs[legId].desired,
          current: leg.current?.servoAnglesDeg
            ? buildLegPoseFromServoAngles(leg.current.servoAnglesDeg, calibration, {
                jointLimits: leg.jointLimits ?? next.legs[legId].jointLimits,
              })
            : leg.current?.foot
              ? buildLegPoseFromFoot(leg.current.foot, calibration, {
                  startThetaThigh: next.legs[legId].current?.geometry?.thetaThigh,
                  startThetaServo: next.legs[legId].current?.geometry?.thetaServo,
                  jointLimits: leg.jointLimits ?? next.legs[legId].jointLimits,
                })
              : next.legs[legId].current,
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
    const baudRate = Number(request.body.baudRate || 115200);
    await bridge.connect({ path, baudRate }, wss);
    safeJson(response, 200, { ok: true, status: bridge.status });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/disconnect", async (_request, response) => {
  try {
    await bridge.disconnect();
    safeJson(response, 200, { ok: true, status: bridge.status });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/command", async (request, response) => {
  try {
    const command = validateCommand(request.body.command);
    bridge.applyDerivedLocalState(command);
    await bridge.send(command);
    safeJson(response, 200, { ok: true });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

app.post("/api/animations", async (request, response) => {
  try {
    const clip = validateClip(request.body.clip);
    await bridge.uploadAnimation(clip);
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
    safeJson(response, 200, { ok: true });
  } catch (error) {
    safeJson(response, 500, { error: error.message });
  }
});

wss.on("connection", (socket) => {
  socket.on("error", (error) => {
    logBridgeError("WebSocket connection error", error);
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
