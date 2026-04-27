// Browser ↔ Pi command protocol. The dashboard POSTs JSON bodies matching
// these shapes to the Node server (pi_server/server.js), which routes them
// to the gait planner / mode controller / serial bridge. Validation lives
// here so both ends agree on the schema — the server rejects bad commands
// early and the UI can preview validation errors without a round-trip.

import { LEG_IDS, MODE_OPTIONS, JOINT_NAMES } from "./robot-config.js";

export function isKnownLegId(value) {
  return LEG_IDS.includes(value);
}

export function isKnownJointName(value) {
  return JOINT_NAMES.includes(value);
}

const COMMAND_VALIDATORS = {
  set_mode(cmd) {
    if (!MODE_OPTIONS.includes(cmd.mode)) throw new Error(`Unknown mode: ${cmd.mode}`);
  },

  twist(cmd) {
    for (const k of ["x", "y", "yaw"]) {
      if (!Number.isFinite(cmd[k])) throw new Error(`twist.${k} must be numeric`);
    }
  },

  foot_targets(cmd) {
    // 4x3 matrix in body frame, meters. Rows = FR/FL/RR/RL, columns = x/y/z.
    if (!Array.isArray(cmd.targets) || cmd.targets.length !== 4) {
      throw new Error("foot_targets.targets must be 4 entries (FR/FL/RR/RL)");
    }
    for (let i = 0; i < 4; i++) {
      const t = cmd.targets[i];
      if (!Array.isArray(t) || t.length !== 3 || !t.every(Number.isFinite)) {
        throw new Error(`foot_targets.targets[${i}] must be [x, y, z] numbers`);
      }
    }
  },

  joint_angles(cmd) {
    // Flat 12-element array in JOINT_NAMES order (radians).
    if (!Array.isArray(cmd.angles) || cmd.angles.length !== 12 || !cmd.angles.every(Number.isFinite)) {
      throw new Error("joint_angles.angles must be 12 numbers");
    }
  },

  servo_angles(cmd) {
    // Flat 12-element array in JOINT_NAMES order, servo-horn degrees 0..180.
    if (!Array.isArray(cmd.angles_deg) || cmd.angles_deg.length !== 12 || !cmd.angles_deg.every(Number.isFinite)) {
      throw new Error("servo_angles.angles_deg must be 12 numbers");
    }
  },

  // Same shape as servo_angles, but bypasses the dashboard-side servo_cal
  // clamp window. Intended for the limit-finder workflow where you need to
  // probe past the configured min/max to find the real hardware bound.
  raw_servo_angles(cmd) {
    if (!Array.isArray(cmd.angles_deg) || cmd.angles_deg.length !== 12 || !cmd.angles_deg.every(Number.isFinite)) {
      throw new Error("raw_servo_angles.angles_deg must be 12 numbers");
    }
  },

  disconnect() {
    /* no fields */
  },

  rotate_degrees(cmd) {
    if (cmd.direction !== "left" && cmd.direction !== "right") {
      throw new Error("rotate_degrees.direction must be 'left' or 'right'");
    }
    if (!Number.isFinite(cmd.degrees) || cmd.degrees <= 0) {
      throw new Error("rotate_degrees.degrees must be a positive number");
    }
  },

  stance_play(cmd) {
    if (!["stand", "crouch", "extend"].includes(cmd.name)) {
      throw new Error("stance_play.name must be stand, crouch, or extend");
    }
  },

  stance_save(cmd) {
    if (!["stand", "crouch", "extend"].includes(cmd.name)) {
      throw new Error("stance_save.name must be stand, crouch, or extend");
    }
  },

  stabilizer_set(cmd) {
    if (typeof cmd.params !== "object" || cmd.params === null) {
      throw new Error("stabilizer_set.params must be an object");
    }
  },

  animation_play(cmd) {
    if (typeof cmd.name !== "string" || !cmd.name) {
      throw new Error("animation_play.name must be a non-empty string");
    }
  },

  animation_stop() {
    /* no fields */
  },

  recording_start() {
    /* no fields */
  },

  recording_stop() {
    /* no fields */
  },

  set_servo_overrides(cmd) {
    if (typeof cmd.overrides !== "object" || cmd.overrides === null) {
      throw new Error("set_servo_overrides.overrides must be an object");
    }
    for (const key of Object.keys(cmd.overrides)) {
      if (!isKnownJointName(key)) throw new Error(`Unknown joint: ${key}`);
    }
  },
};

export function validateCommand(command) {
  if (!command || typeof command !== "object") throw new Error("Command must be an object.");
  if (typeof command.type !== "string" || !command.type) throw new Error("Command is missing a type.");

  const validator = COMMAND_VALIDATORS[command.type];
  if (!validator) throw new Error(`Unknown command type: ${command.type}`);
  validator(command);
  return command;
}

export function toWireMessage(message, seq = 0) {
  return JSON.stringify({ ...validateCommand(message), seq });
}

export function parseWireMessage(line) {
  const parsed = JSON.parse(line);
  if (!parsed?.type) throw new Error("Protocol message missing type.");
  return parsed;
}

export const COMMAND_TYPES = Object.keys(COMMAND_VALIDATORS);
