import { LEG_IDS, MODE_OPTIONS, MOTION_MODE_OPTIONS } from "./robot-config.js";

function clampAxis(value) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    throw new Error("Drive axes must be numeric.");
  }
  return Math.max(-1, Math.min(1, numeric));
}

function assertNumber(value, label) {
  if (!Number.isFinite(value)) {
    throw new Error(`${label} must be numeric.`);
  }
}

export function isKnownLegId(value) {
  return LEG_IDS.includes(value);
}

export function normalizeDriveCommand(command = {}) {
  return {
    vx: clampAxis(command.vx ?? 0),
    vy: clampAxis(command.vy ?? 0),
    yawRate: clampAxis(command.yawRate ?? 0),
    source: typeof command.source === "string" && command.source ? command.source : "unknown",
  };
}

export function validateFullBodyPose(command) {
  const hasLegMap = command?.legs && typeof command.legs === "object";
  const prefixes = {
    front_left: "FL",
    front_right: "FR",
    rear_left: "RL",
    rear_right: "RR",
  };

  if (!hasLegMap) {
    for (const legId of LEG_IDS) {
      const prefix = prefixes[legId];
      for (const key of ["HipYawDeg", "ThighDeg", "CalfDeg"]) {
        assertNumber(command?.[`${prefix}${key}`], `${prefix}${key}`);
      }
    }
    return command;
  }

  for (const legId of LEG_IDS) {
    const leg = command.legs[legId];
    if (!leg) {
      throw new Error(`Full-body pose is missing ${legId}.`);
    }
    for (const key of ["hipYaw", "thigh", "calf"]) {
      assertNumber(leg.servoAnglesDeg?.[key], `${legId}.${key}`);
    }
  }

  return command;
}

export function validateCommand(command) {
  if (!command || typeof command !== "object") {
    throw new Error("Command must be an object.");
  }

  if (typeof command.type !== "string" || !command.type) {
    throw new Error("Command is missing a type.");
  }

  if ("legId" in command && !isKnownLegId(command.legId)) {
    throw new Error(`Unknown leg id: ${command.legId}`);
  }

  if (command.type === "set_mode" && !MODE_OPTIONS.includes(command.mode)) {
    throw new Error(`Unknown mode: ${command.mode}`);
  }

  if (command.type === "set_motion_mode" && !MOTION_MODE_OPTIONS.includes(command.mode)) {
    throw new Error(`Unknown motion mode: ${command.mode}`);
  }

  if (command.type === "set_drive_command") {
    command.drive = normalizeDriveCommand(command.drive ?? command);
  }

  if (command.type === "apply_full_body_pose") {
    validateFullBodyPose(command);
  }

  if (command.type === "set_leg_servo_channel_map") {
    if (typeof command.legId !== "string" || !isKnownLegId(command.legId)) {
      throw new Error(`Unknown leg id: ${command.legId}`);
    }

    for (const key of ["hipYawChannel", "thighChannel", "calfChannel"]) {
      if (!Number.isInteger(command[key]) || command[key] < 0 || command[key] > 15) {
        throw new Error(`${key} must be an integer between 0 and 15.`);
      }
    }
  }

  if (command.type === "set_leg_joint_limits") {
    if (typeof command.legId !== "string" || !isKnownLegId(command.legId)) {
      throw new Error(`Unknown leg id: ${command.legId}`);
    }

    for (const key of ["hipYawMinDeg", "hipYawMaxDeg", "thighMinDeg", "thighMaxDeg", "calfMinDeg", "calfMaxDeg"]) {
      assertNumber(command[key], key);
    }
  }

  if (command.type === "set_leg_servo_speed_limit") {
    if (typeof command.legId !== "string" || !isKnownLegId(command.legId)) {
      throw new Error(`Unknown leg id: ${command.legId}`);
    }

    for (const key of ["hipYawDegPerSec", "thighDegPerSec", "calfDegPerSec"]) {
      if (!Number.isFinite(command[key]) || command[key] <= 0) {
        throw new Error(`${key} must be a positive number.`);
      }
    }
  }

  if (command.type === "set_leg_servo_trim") {
    if (typeof command.legId !== "string" || !isKnownLegId(command.legId)) {
      throw new Error(`Unknown leg id: ${command.legId}`);
    }

    for (const key of ["hipYawOffsetDeg", "thighOffsetDeg", "calfOffsetDeg"]) {
      assertNumber(command[key], key);
    }
  }

  if (command.type === "release_servos" && "legId" in command) {
    throw new Error("release_servos must not target a single leg.");
  }

  return command;
}

export function toWireMessage(message, seq = 0) {
  return JSON.stringify({
    ...validateCommand(message),
    seq,
  });
}

export function parseWireMessage(line) {
  const parsed = JSON.parse(line);
  if (!parsed?.type) {
    throw new Error("Protocol message missing type.");
  }
  return parsed;
}
