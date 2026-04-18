import { LEG_IDS, MODE_OPTIONS } from "./robot-config.js";

export function isKnownLegId(value) {
  return LEG_IDS.includes(value);
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

  if (command.type === "set_leg_servo_channel_map") {
    if (typeof command.legId !== "string" || !isKnownLegId(command.legId)) {
      throw new Error(`Unknown leg id: ${command.legId}`);
    }

    for (const key of ["thighChannel", "calfChannel"]) {
      if (!Number.isInteger(command[key]) || command[key] < 0 || command[key] > 15) {
        throw new Error(`${key} must be an integer between 0 and 15.`);
      }
    }
  }

  if (command.type === "set_leg_joint_limits") {
    if (typeof command.legId !== "string" || !isKnownLegId(command.legId)) {
      throw new Error(`Unknown leg id: ${command.legId}`);
    }

    for (const key of ["thighMinDeg", "thighMaxDeg", "calfMinDeg", "calfMaxDeg"]) {
      if (!Number.isFinite(command[key])) {
        throw new Error(`${key} must be numeric.`);
      }
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
    seq
  });
}

export function parseWireMessage(line) {
  const parsed = JSON.parse(line);
  if (!parsed?.type) {
    throw new Error("Protocol message missing type.");
  }
  return parsed;
}
