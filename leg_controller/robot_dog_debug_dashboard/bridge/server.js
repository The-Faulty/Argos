import express from "express";
import http from "http";
import { pathToFileURL } from "node:url";
import { WebSocketServer } from "ws";
import { SerialPort } from "serialport";
import {
  buildLegPoseFromFoot,
  buildLegPoseFromJointAngles,
  buildLegPoseFromServoAngles,
  createNeutralCalibration,
  normalizeJointLimits,
} from "../shared/kinematics.js";
import { createUploadFrames, validateClip } from "../shared/animation.js";
import {
  createFullBodyPoseCommand,
  createMotionStatePatch,
  mapHardwareServoAnglesToLeg,
  mapLegServoAnglesToHardware,
} from "../shared/locomotion.js";
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
import { normalizeDriveCommand, normalizeStance, validateCommand } from "../shared/protocol.js";
import { SerialBridge as RefactorSerialBridge } from "./refactor-serial-bridge.js";

const PORT = Number(process.env.PORT || 8787);
export const DEFAULT_SERIAL_BAUD = Number(process.env.ARGOS_SERIAL_BAUD || 921600);
export const DEFAULT_SERIAL_PACING_MS = Number(process.env.ARGOS_SERIAL_PACING_MS ?? 12);
export const DEFAULT_SERIAL_ACK_TIMEOUT_MS = Number(process.env.ARGOS_SERIAL_ACK_TIMEOUT_MS ?? 0);

const calibration = createNeutralCalibration();

const FULL_BODY_PREFIXES = {
  front_left: "FL",
  front_right: "FR",
  rear_left: "RL",
  rear_right: "RR",
};

const LEGACY_TO_REFACTOR_LEG = {
  front_left: "FL",
  front_right: "FR",
  rear_left: "RL",
  rear_right: "RR",
};

const REFACTOR_TO_LEGACY_LEG = Object.fromEntries(
  Object.entries(LEGACY_TO_REFACTOR_LEG).map(([legacyLegId, refactorLegId]) => [refactorLegId, legacyLegId]),
);

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

function numeric(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
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

function normalizeDashboardServoAngles(source, fallback = DEFAULT_LEG_COMMAND.servoAnglesDeg) {
  return {
    hipYaw: numeric(source?.hipYaw ?? source?.hip ?? source?.hipYawServoDeg ?? source?.hipServoDeg, fallback.hipYaw),
    thigh: numeric(source?.thigh ?? source?.thighServoDeg, fallback.thigh),
    calf: numeric(source?.calf ?? source?.calfServoDeg, fallback.calf),
  };
}

function mapDashboardServoAnglesToFirmware(
  legId,
  source,
  servoTrimDeg = DEFAULT_SERVO_TRIM_DEG,
  fallback = DEFAULT_LEG_COMMAND.servoAnglesDeg,
) {
  const angles = normalizeDashboardServoAngles(source, fallback);
  const hardwareAngles = mapLegServoAnglesToHardware(legId, angles, servoTrimDeg);
  return {
    hipServoDeg: hardwareAngles.hipYaw,
    thighServoDeg: hardwareAngles.thigh,
    calfServoDeg: hardwareAngles.calf,
  };
}

function mapFirmwareServoAnglesToDashboard(
  legId,
  source,
  servoTrimDeg = DEFAULT_SERVO_TRIM_DEG,
  fallback = DEFAULT_LEG_COMMAND.servoAnglesDeg,
) {
  const angles = normalizeDashboardServoAngles(source, fallback);
  return mapHardwareServoAnglesToLeg(legId, angles, servoTrimDeg);
}

function translateFirmwareServoChannelMap(source, fallback = DEFAULT_SERVO_CHANNEL_MAP.front_left) {
  return {
    hipYaw: Math.round(numeric(source?.hipYaw ?? source?.hipChannel ?? source?.hip, fallback.hipYaw)),
    thigh: Math.round(numeric(source?.thighChannel ?? source?.thigh, fallback.thigh)),
    calf: Math.round(numeric(source?.calfChannel ?? source?.calf, fallback.calf)),
  };
}

function translateFirmwareServoSpeedLimits(source, fallback = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC) {
  return {
    hipYaw: numeric(source?.hipYaw ?? source?.hipDegPerSec ?? source?.hip, fallback.hipYaw),
    thigh: numeric(source?.thighDegPerSec ?? source?.thigh, fallback.thigh),
    calf: numeric(source?.calfDegPerSec ?? source?.calf, fallback.calf),
  };
}

function translateFirmwareJointLimits(source, fallback = DEFAULT_JOINT_LIMITS) {
  return normalizeJointLimits({
    hipYawDeg: clone(fallback.hipYawDeg ?? DEFAULT_JOINT_LIMITS.hipYawDeg),
    thighDeg: clone(source?.thighDeg ?? source?.femurDeg ?? fallback.thighDeg ?? DEFAULT_JOINT_LIMITS.thighDeg),
    calfDeg: clone(source?.calfDeg ?? source?.tibiaDeg ?? fallback.calfDeg ?? DEFAULT_JOINT_LIMITS.calfDeg),
  });
}

function normalizeFirmwareStatePayload(payload, currentStatus = createStatus()) {
  if (!payload || typeof payload !== "object") {
    return payload;
  }

  const next = {
    ...payload,
  };

  if (payload.legs && typeof payload.legs === "object") {
    next.legs = {};
    for (const legId of LEG_IDS) {
      const leg = payload.legs[legId];
      if (!leg) {
        continue;
      }

      const fallbackLeg = currentStatus.legs?.[legId] ?? {};
      next.legs[legId] = {
        ...leg,
        servoChannelMap: translateFirmwareServoChannelMap(leg.servoChannelMap, fallbackLeg.servoChannelMap ?? DEFAULT_SERVO_CHANNEL_MAP[legId]),
        servoSpeedLimitDegPerSec: translateFirmwareServoSpeedLimits(
          leg.servoSpeedLimitDegPerSec,
          fallbackLeg.servoSpeedLimitDegPerSec ?? DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
        ),
        firmwareJointLimits: translateFirmwareJointLimits(leg.jointLimits, fallbackLeg.jointLimits ?? DEFAULT_JOINT_LIMITS),
        desired: leg.desired,
        current: leg.current,
      };
    }
  }

  return next;
}

function translateRefactorServoAngles(source, fallback = DEFAULT_LEG_COMMAND.servoAnglesDeg) {
  return {
    hipYaw: numeric(source?.hipYaw ?? source?.coxa ?? source?.hip ?? source?.hipServoDeg, fallback.hipYaw),
    thigh: numeric(source?.thigh ?? source?.femur ?? source?.thighServoDeg, fallback.thigh),
    calf: numeric(source?.calf ?? source?.tibia ?? source?.calfServoDeg, fallback.calf),
  };
}

function translateRefactorServoChannelMap(source, fallback = DEFAULT_SERVO_CHANNEL_MAP.front_left) {
  return {
    hipYaw: Math.round(numeric(source?.hipYaw ?? source?.coxa ?? source?.hip ?? source?.hipChannel, fallback.hipYaw)),
    thigh: Math.round(numeric(source?.thigh ?? source?.femur ?? source?.thighChannel, fallback.thigh)),
    calf: Math.round(numeric(source?.calf ?? source?.tibia ?? source?.calfChannel, fallback.calf)),
  };
}

function translateRefactorServoSpeedLimits(source, fallback = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC) {
  return {
    hipYaw: numeric(source?.hipYaw ?? source?.coxa ?? source?.hip ?? source?.hipDegPerSec, fallback.hipYaw),
    thigh: numeric(source?.thigh ?? source?.femur ?? source?.thighDegPerSec, fallback.thigh),
    calf: numeric(source?.calf ?? source?.tibia ?? source?.calfDegPerSec, fallback.calf),
  };
}

function translateRefactorJointLimits(source, fallback = DEFAULT_JOINT_LIMITS) {
  return normalizeJointLimits({
    hipYawDeg: clone(fallback.hipYawDeg ?? DEFAULT_JOINT_LIMITS.hipYawDeg),
    thighDeg: clone(source?.thighDeg ?? source?.femurDeg ?? fallback.thighDeg ?? DEFAULT_JOINT_LIMITS.thighDeg),
    calfDeg: clone(source?.calfDeg ?? source?.tibiaDeg ?? fallback.calfDeg ?? DEFAULT_JOINT_LIMITS.calfDeg),
  });
}

function normalizeRefactorStatePayload(payload, currentStatus = createStatus()) {
  if (!payload || typeof payload !== "object") {
    return payload;
  }

  const next = {
    ...payload,
  };

  if (payload.legs && typeof payload.legs === "object") {
    next.legs = {};
    for (const [refactorLegId, leg] of Object.entries(payload.legs)) {
      const legId = REFACTOR_TO_LEGACY_LEG[refactorLegId] ?? refactorLegId;
      if (!LEG_IDS.includes(legId)) {
        continue;
      }

      const fallbackLeg = currentStatus.legs?.[legId] ?? {};
      next.legs[legId] = {
        ...leg,
        servoChannelMap: translateRefactorServoChannelMap(
          leg.servoChannelMap,
          fallbackLeg.servoChannelMap ?? DEFAULT_SERVO_CHANNEL_MAP[legId],
        ),
        servoSpeedLimitDegPerSec: translateRefactorServoSpeedLimits(
          leg.servoSpeedLimitDegPerSec,
          fallbackLeg.servoSpeedLimitDegPerSec ?? DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
        ),
        firmwareJointLimits: translateRefactorJointLimits(leg.jointLimits, fallbackLeg.jointLimits ?? DEFAULT_JOINT_LIMITS),
        desired: leg.desired
          ? {
            ...leg.desired,
            servoAnglesDeg: translateRefactorServoAngles(
              leg.desired.servoAnglesDeg,
              fallbackLeg.desired?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg,
            ),
          }
          : leg.desired,
        current: leg.current
          ? {
            ...leg.current,
            servoAnglesDeg: translateRefactorServoAngles(
              leg.current.servoAnglesDeg,
              fallbackLeg.current?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg,
            ),
          }
          : leg.current,
      };
    }
  }

  return next;
}

function extractServoSignature(command) {
  return {
    hipServoDeg: numeric(command?.hipServoDeg),
    thighServoDeg: numeric(command?.thighServoDeg),
    calfServoDeg: numeric(command?.calfServoDeg),
  };
}

function servoCommandsEqual(previous, nextCommand) {
  const next = extractServoSignature(nextCommand);
  const eps = 0.05;
  return Math.abs(previous.hipServoDeg - next.hipServoDeg) < eps
    && Math.abs(previous.thighServoDeg - next.thighServoDeg) < eps
    && Math.abs(previous.calfServoDeg - next.calfServoDeg) < eps;
}

function buildPoseFromPayload(legId, pose, fallbackPose, jointLimits, servoTrimDeg = DEFAULT_SERVO_TRIM_DEG) {
  if (!pose) {
    return fallbackPose;
  }

  if (pose.servoAnglesDeg) {
    return buildLegPoseFromServoAngles(
      mapFirmwareServoAnglesToDashboard(
        legId,
        pose.servoAnglesDeg,
        servoTrimDeg,
        fallbackPose?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg,
      ),
      calibration,
      {
        jointLimits,
      },
    );
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

  return fallbackPose;
}

function createDefaultLegPose(jointLimits = DEFAULT_JOINT_LIMITS) {
  return buildLegPoseFromJointAngles(DEFAULT_LEG_COMMAND.jointAnglesDeg, calibration, { jointLimits });
}

function extractFullBodyServoTargets(command, fallbackLegs = {}) {
  const targets = {};
  const hasLegMap = command?.legs && typeof command.legs === "object";

  if (hasLegMap) {
    for (const legId of LEG_IDS) {
      const leg = command.legs[legId];
      if (!leg) {
        throw new Error(`Full-body pose is missing ${legId}.`);
      }
      targets[legId] = normalizeDashboardServoAngles(
        leg.servoAnglesDeg,
        fallbackLegs[legId]?.desired?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg,
      );
    }
    return targets;
  }

  for (const legId of LEG_IDS) {
    const prefix = FULL_BODY_PREFIXES[legId];
    const hipYaw = Number(command?.[`${prefix}HipYawDeg`]);
    const thigh = Number(command?.[`${prefix}ThighDeg`]);
    const calf = Number(command?.[`${prefix}CalfDeg`]);
    if (!Number.isFinite(hipYaw) || !Number.isFinite(thigh) || !Number.isFinite(calf)) {
      throw new Error(`Full-body pose is missing servo targets for ${legId}.`);
    }
    targets[legId] = { hipYaw, thigh, calf };
  }

  return targets;
}

function buildFirmwareServoCommand(
  legId,
  servoAnglesDeg,
  servoTrimDeg = DEFAULT_SERVO_TRIM_DEG,
) {
  const hardwareAngles = mapDashboardServoAnglesToFirmware(
    legId,
    servoAnglesDeg,
    servoTrimDeg,
    DEFAULT_LEG_COMMAND.servoAnglesDeg,
  );
  return {
    type: "set_leg_servo_angles",
    legId,
    hipServoDeg: hardwareAngles.hipYaw ?? hardwareAngles.hipServoDeg,
    thighServoDeg: hardwareAngles.thigh ?? hardwareAngles.thighServoDeg,
    calfServoDeg: hardwareAngles.calf ?? hardwareAngles.calfServoDeg,
  };
}

function buildFirmwareServoCommandsFromTargets(targets, legs = {}) {
  return LEG_IDS.map((legId) =>
    buildFirmwareServoCommand(
      legId,
      targets[legId],
      legs[legId]?.servoTrimDeg ?? DEFAULT_SERVO_TRIM_DEG,
    ));
}

function buildFirmwareServoCommandsFromStatus(status) {
  return buildFirmwareServoCommandsFromTargets(
    Object.fromEntries(
      LEG_IDS.map((legId) => [
        legId,
        status.legs[legId]?.desired?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg,
      ]),
    ),
    status.legs,
  );
}

function translateCommandForFirmware(command, status) {
  switch (command.type) {
    case "apply_full_body_pose":
      return buildFirmwareServoCommandsFromTargets(
        extractFullBodyServoTargets(command, status?.legs ?? {}),
        status?.legs,
      );
    case "set_stance":
    case "set_drive_command":
      return buildFirmwareServoCommandsFromStatus(status);
    case "set_motion_mode":
      if (command.mode === "stand" || command.mode === "drive") {
        return buildFirmwareServoCommandsFromStatus(status);
      }
      if (command.mode === "idle") {
        return [{ type: "set_mode", mode: "idle" }];
      }
      return [];
    case "set_leg_foot_xy":
    case "set_leg_joint_angles":
    case "set_leg_servo_trim":
      if (command.type === "set_leg_servo_trim") {
        return buildFirmwareServoCommandsFromStatus(status);
      }
      return [buildFirmwareServoCommand(command.legId, status.legs[command.legId]?.desired?.servoAnglesDeg)];
    case "set_leg_servo_angles":
      return [
        {
          type: "set_leg_servo_angles",
          legId: command.legId,
          ...mapDashboardServoAnglesToFirmware(
            command.legId,
            {
            hipYawServoDeg: command.hipYawServoDeg,
            hipServoDeg: command.hipServoDeg,
            thighServoDeg: command.thighServoDeg,
            calfServoDeg: command.calfServoDeg,
            },
            status?.legs?.[command.legId]?.servoTrimDeg ?? DEFAULT_SERVO_TRIM_DEG,
          ),
        },
      ];
    case "set_leg_servo_channel_map":
      return [
        {
          type: "set_leg_servo_channel_map",
          legId: command.legId,
          hipChannel: Math.round(numeric(command.hipYawChannel ?? command.hipChannel)),
          thighChannel: Math.round(numeric(command.thighChannel)),
          calfChannel: Math.round(numeric(command.calfChannel)),
        },
      ];
    case "set_leg_joint_limits":
      return [
        {
          type: "set_leg_joint_limits",
          legId: command.legId,
          thighMinDeg: numeric(command.thighMinDeg),
          thighMaxDeg: numeric(command.thighMaxDeg),
          calfMinDeg: numeric(command.calfMinDeg),
          calfMaxDeg: numeric(command.calfMaxDeg),
        },
      ];
    case "set_leg_servo_speed_limit":
      return [
        {
          type: "set_leg_servo_speed_limit",
          legId: command.legId,
          hipDegPerSec: numeric(command.hipYawDegPerSec ?? command.hipDegPerSec),
          thighDegPerSec: numeric(command.thighDegPerSec),
          calfDegPerSec: numeric(command.calfDegPerSec),
        },
      ];
    case "stop_motion":
      return [{ type: "set_mode", mode: "idle" }];
    case "panic_release":
      return [{ type: "release_servos" }];
    case "upload_animation":
      if (command.stage === "frame") {
        return [
          {
            ...command,
            ...mapDashboardServoAnglesToFirmware(
              command.legId,
              {
                hipYawServoDeg: command.hipServoDeg ?? command.hipYawServoDeg,
                thighServoDeg: command.thighServoDeg,
                calfServoDeg: command.calfServoDeg,
              },
              status?.legs?.[command.legId]?.servoTrimDeg ?? DEFAULT_SERVO_TRIM_DEG,
            ),
          },
        ];
      }
      return [clone(command)];
    default:
      return [clone(command)];
  }
}

function commandUsesOptionalConnection(command) {
  return command.type === "set_stance"
    || command.type === "set_motion_mode"
    || command.type === "set_drive_command"
    || command.type === "stop_motion";
}

async function handleRealtimeCommand(bridge, rawCommand, wss) {
  const command = validateCommand(rawCommand);
  bridge.applyDerivedLocalState(command);
  await bridge.forwardCommand(command);
  bridge.broadcast(wss, "status", bridge.status);
}

function createStatus() {
  const legs = {};
  for (const legId of LEG_IDS) {
    const desired = createDefaultLegPose(DEFAULT_JOINT_LIMITS);

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

export class BridgeState {
  constructor({
    pacingMs = DEFAULT_SERIAL_PACING_MS,
    ackTimeoutMs = DEFAULT_SERIAL_ACK_TIMEOUT_MS,
    serialBridgeFactory = (options) => new RefactorSerialBridge(options),
  } = {}) {
    this.status = createStatus();
    this.serial = null;
    this.transportCleanup = null;
    this.uploadedClips = new Map();
    this.pacingMs = Math.max(0, Number.isFinite(pacingMs) ? pacingMs : 0);
    this.ackTimeoutMs = Math.max(0, Number.isFinite(ackTimeoutMs) ? ackTimeoutMs : 0);
    this.serialBridgeFactory = serialBridgeFactory;
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

  attachTransport(transport, wss) {
    const onOpen = async () => {
      this.status.connected = true;
      this.status.esp32Connected = true;
      this.status.connectedPort = transport.path ?? this.status.connectedPort;
      this.status.lastError = null;
      try {
        await this.refreshPorts();
      } catch (error) {
        this.status.lastError = error.message;
      }
      this.broadcast(wss, "status", this.status);
    };

    const onClose = () => {
      this.status.connected = false;
      this.status.esp32Connected = false;
      this.status.connectedPort = null;
      this.broadcast(wss, "status", this.status);
    };

    const onError = (error) => {
      this.status.lastError = error.message;
      logBridgeError("Serial bridge error", error);
      this.broadcastEvent(wss, { type: "error", message: error.message });
    };

    const onState = (payload) => {
      this.status = this.normalizeStatePayload(normalizeRefactorStatePayload(payload, this.status));
      this.broadcastEvent(wss, { type: "state", payload: this.status });
    };

    const onHelloAck = (message) => {
      const payload = message.payload
        ? this.normalizeStatePayload(normalizeFirmwareStatePayload(message.payload, this.status))
        : this.status;
      if (payload !== this.status) {
        this.status = payload;
      }
      this.broadcastEvent(wss, { type: "hello_ack", payload: this.status });
    };

    const onAck = (message) => {
      this.status.lastAck = message.message ?? String(message.seq ?? "ack");
      this.broadcastEvent(wss, message);
    };

    const onMessage = (message) => {
      if (message.type === "animation_progress") {
        this.status.activeAnimation = message.name ?? this.status.activeAnimation;
      }
      this.broadcastEvent(wss, message);
    };

    transport.on("open", onOpen);
    transport.on("close", onClose);
    transport.on("error", onError);
    transport.on("state", onState);
    transport.on("hello_ack", onHelloAck);
    transport.on("ack", onAck);
    transport.on("message", onMessage);

    this.transportCleanup = () => {
      transport.off("open", onOpen);
      transport.off("close", onClose);
      transport.off("error", onError);
      transport.off("state", onState);
      transport.off("hello_ack", onHelloAck);
      transport.off("ack", onAck);
      transport.off("message", onMessage);
    };
  }

  async connect({ path, baudRate = DEFAULT_SERIAL_BAUD }, wss) {
    await this.disconnect();
    const transport = this.serialBridgeFactory({
      path,
      baudRate,
      pacingMs: this.pacingMs,
      ackTimeoutMs: this.ackTimeoutMs,
    });
    this.serial = transport;
    this.attachTransport(transport, wss);

    try {
      await new Promise((resolve, reject) => {
        const cleanup = () => {
          transport.off("open", onOpen);
          transport.off("error", onError);
        };
        const onOpen = () => {
          cleanup();
          resolve();
        };
        const onError = (error) => {
          cleanup();
          reject(error);
        };
        transport.on("open", onOpen);
        transport.on("error", onError);
        transport.connect();
      });
    } catch (error) {
      this.transportCleanup?.();
      this.transportCleanup = null;
      this.serial = null;
      transport.disconnect?.();
      this.status.connected = false;
      this.status.esp32Connected = false;
      this.status.connectedPort = null;
      throw error;
    }
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
    this.transportCleanup?.();
    this.transportCleanup = null;
    serial.disconnect();
    this.status.connected = false;
    this.status.esp32Connected = false;
    this.status.connectedPort = null;
  }

  async dispatchFirmwareCommand(command) {
    if (!this.serial || !this.status.connected) {
      throw new Error("Serial bridge is not connected.");
    }

    switch (command.type) {
      case "hello":
      case "upload_animation":
      case "play_animation":
      case "stop_animation":
      case "run_builtin":
        return this.serial.enqueueRawCommand(command);
      case "get_state":
        return this.serial.requestState();
      case "set_mode":
        return this.serial.setMode(command.mode);
      case "release_servos":
        return this.serial.releaseServos();
      case "set_leg_servo_angles":
        return this.serial.setLegServoAngles(LEGACY_TO_REFACTOR_LEG[command.legId], {
          coxa: numeric(command.hipServoDeg),
          femur: numeric(command.thighServoDeg),
          tibia: numeric(command.calfServoDeg),
        });
      case "set_leg_servo_speed_limit":
        return this.serial.setLegServoSpeedLimit(LEGACY_TO_REFACTOR_LEG[command.legId], {
          coxa: numeric(command.hipDegPerSec),
          femur: numeric(command.thighDegPerSec),
          tibia: numeric(command.calfDegPerSec),
        });
      case "set_leg_joint_limits":
        return this.serial.setLegJointLimits(LEGACY_TO_REFACTOR_LEG[command.legId], {
          femur: [numeric(command.thighMinDeg), numeric(command.thighMaxDeg)],
          tibia: [numeric(command.calfMinDeg), numeric(command.calfMaxDeg)],
        });
      case "set_leg_servo_channel_map":
        return this.serial.setLegServoChannelMap(LEGACY_TO_REFACTOR_LEG[command.legId], {
          coxa: Math.round(numeric(command.hipChannel)),
          femur: Math.round(numeric(command.thighChannel)),
          tibia: Math.round(numeric(command.calfChannel)),
        });
      default:
        return this.serial.enqueueRawCommand(command);
    }
  }

  async send(command) {
    if (!this.serial || !this.status.connected) {
      throw new Error("Serial bridge is not connected.");
    }
    return this.dispatchFirmwareCommand(command);
  }

  async sendIfConnected(command) {
    if (!this.serial || !this.status.connected) {
      return;
    }
    await this.dispatchFirmwareCommand(command);
  }

  async sendMany(commands) {
    if (!this.serial || !this.status.connected) {
      throw new Error("Serial bridge is not connected.");
    }
    await Promise.all(commands.map((command) => this.dispatchFirmwareCommand(command)));
  }

  async sendManyIfConnected(commands) {
    if (!this.serial || !this.status.connected) {
      return;
    }
    await Promise.all(commands.map((command) => this.dispatchFirmwareCommand(command)));
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

  buildFullBodyServoCommands() {
    return buildFirmwareServoCommandsFromStatus(this.status);
  }

  buildLegServoCommand(legId) {
    return buildFirmwareServoCommand(
      legId,
      this.status.legs[legId]?.desired?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg,
    );
  }

  async forwardCommand(command) {
    validateCommand(command);
    const translatedCommands = translateCommandForFirmware(command, this.status);
    if (!translatedCommands.length) {
      return;
    }

    if (commandUsesOptionalConnection(command)) {
      await this.sendManyIfConnected(translatedCommands);
      return;
    }

    await this.sendMany(translatedCommands);
  }

  async uploadAnimation(clip) {
    const validated = validateClip(clip);
    this.uploadedClips.set(validated.name, validated);
    const jointLimitsByLeg = Object.fromEntries(
      LEG_IDS.map((legId) => [legId, this.status.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS]),
    );
    for (const frame of createUploadFrames(validated, { jointLimitsByLeg })) {
      const translated = translateCommandForFirmware(frame, this.status);
      for (const command of translated) {
        await this.send(command);
      }
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
      this.status.activeAnimation = null;
      return;
    }

    if (command.type === "apply_full_body_pose") {
      const jointLimitsByLeg = Object.fromEntries(
        LEG_IDS.map((legId) => [legId, this.status.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS]),
      );
      const servoTargets = extractFullBodyServoTargets(command, this.status.legs);
      this.status.mode = "direct_servo_angles";
      this.status.motionMode = "idle";
      this.status.driveCommand = { ...DEFAULT_DRIVE_COMMAND };
      this.status.servosReleased = false;
      this.status.activeAnimation = null;
      for (const legId of LEG_IDS) {
        this.status.legs[legId].desired = buildLegPoseFromServoAngles(servoTargets[legId], calibration, {
          jointLimits: jointLimitsByLeg[legId],
        });
      }
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
        this.status.activeAnimation = null;
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
      this.status.activeAnimation = null;
      return;
    }

    if (command.type === "set_stance") {
      const nextMotionMode = this.status.motionMode === "drive" ? "drive" : "stand";
      this.applyMotionPatch({
        motionMode: nextMotionMode,
        driveCommand: nextMotionMode === "drive" ? this.status.driveCommand : DEFAULT_DRIVE_COMMAND,
        stance: command.stance,
      });
      this.status.activeAnimation = null;
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
      this.status.activeAnimation = null;
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
      this.status.activeAnimation = null;
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
      this.status.activeAnimation = null;
      this.status.legs[command.legId].desired = buildLegPoseFromServoAngles(
        normalizeDashboardServoAngles(command, currentDesired?.servoAnglesDeg ?? DEFAULT_LEG_COMMAND.servoAnglesDeg),
        calibration,
        {
          jointLimits,
        },
      );
    }

    if (command.type === "set_leg_servo_channel_map") {
      this.status.legs[command.legId].servoChannelMap = {
        hipYaw: Math.round(numeric(command.hipYawChannel ?? command.hipChannel)),
        thigh: Math.round(numeric(command.thighChannel)),
        calf: Math.round(numeric(command.calfChannel)),
      };
    }

    if (command.type === "set_leg_joint_limits") {
      const existingLimits = this.status.legs[command.legId].jointLimits ?? DEFAULT_JOINT_LIMITS;
      const normalizedLimits = normalizeJointLimits({
        hipYawDeg: {
          min: numeric(command.hipYawMinDeg, existingLimits.hipYawDeg.min),
          max: numeric(command.hipYawMaxDeg, existingLimits.hipYawDeg.max),
        },
        thighDeg: {
          min: numeric(command.thighMinDeg),
          max: numeric(command.thighMaxDeg),
        },
        calfDeg: {
          min: numeric(command.calfMinDeg),
          max: numeric(command.calfMaxDeg),
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
        hipYaw: numeric(command.hipYawDegPerSec ?? command.hipDegPerSec),
        thigh: numeric(command.thighDegPerSec),
        calf: numeric(command.calfDegPerSec),
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

      const normalizedLimits = normalizeJointLimits(next.legs[legId].jointLimits ?? DEFAULT_JOINT_LIMITS);
      const servoTrimDeg = {
        ...next.legs[legId].servoTrimDeg,
        ...(leg.servoTrimDeg ?? {}),
      };
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
        servoTrimDeg,
        firmwareJointLimits: leg.firmwareJointLimits ?? next.legs[legId].firmwareJointLimits ?? null,
        desired: buildPoseFromPayload(legId, leg.desired, next.legs[legId].desired, normalizedLimits, servoTrimDeg),
        current: buildPoseFromPayload(legId, leg.current, next.legs[legId].current, normalizedLimits, servoTrimDeg),
      };
      }
    }

    return next;
  }
}

export function createBridgeServer({ bridge = new BridgeState(), port = PORT } = {}) {
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
      const baudRate = Number(request.body.baudRate || DEFAULT_SERIAL_BAUD);
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
        await handleRealtimeCommand(bridge, message.command, wss);
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

  return {
    app,
    bridge,
    port,
    server,
    wss,
  };
}

export async function startBridgeServer({ bridge = new BridgeState(), port = PORT } = {}) {
  const context = createBridgeServer({ bridge, port });

  await new Promise((resolve, reject) => {
    context.server.on("error", (error) => {
      if (error?.code === "EADDRINUSE") {
        console.error(`Port ${port} is already in use. Reuse the existing bridge or stop the old process before starting another one.`);
      } else {
        console.error("Bridge failed to start:", error);
      }
      reject(error);
    });

    context.server.listen(port, async () => {
      try {
        await bridge.refreshPorts();
        console.log(`Robot dog bridge listening on http://localhost:${port}`);
        resolve();
      } catch (error) {
        reject(error);
      }
    });
  });

  return context;
}

function isMainModule() {
  return Boolean(process.argv[1]) && import.meta.url === pathToFileURL(process.argv[1]).href;
}

export const _internals = {
  buildFirmwareServoCommandsFromStatus,
  commandUsesOptionalConnection,
  createStatus,
  extractFullBodyServoTargets,
  normalizeRefactorStatePayload,
  normalizeDashboardServoAngles,
  normalizeFirmwareStatePayload,
  servoCommandsEqual,
  translateCommandForFirmware,
};

if (isMainModule()) {
  startBridgeServer().catch(() => {
    process.exit(1);
  });

  process.on("unhandledRejection", (error) => {
    logBridgeError("Unhandled rejection in bridge", error);
  });

  process.on("uncaughtException", (error) => {
    logBridgeError("Uncaught exception in bridge", error);
  });
}
