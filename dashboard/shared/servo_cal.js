// Per-joint servo calibration table. Row-for-row mirror of
// firmware/esp32c6/main/main.c::s_servo_cal AND
// ros2_ws/argos_control/Config.py::SERVO_CAL_PER_JOINT. A unit test
// (dashboard/tests/servo_cal.test.js) extracts the firmware + Python
// tables at test time and asserts these three stay in lock-step.
//
// Fields:
//   joint       — JOINT_NAMES string, e.g. "FR_coxa_joint"
//   channel     — PCA9685 output channel (0..11)
//   direction   — +1 or -1; inverts joint-rad → servo-deg mapping
//   offset_deg  — added after the sign flip, bench-calibrated
//   min_deg     — hardware-safe lower horn bound
//   max_deg     — hardware-safe upper horn bound

import {
  JOINT_NAMES,
  SERVO_CENTER_DEG,
} from "./robot-config.js";

export const SERVO_CAL_PER_JOINT = [
  { joint: "FR_coxa_joint",  channel:  0, direction:  1, offset_deg: 0.0, min_deg:  45.0, max_deg: 135.0 },
  { joint: "FR_femur_joint", channel:  1, direction:  1, offset_deg: 0.0, min_deg:  50.0, max_deg: 115.0 },
  { joint: "FR_tibia_joint", channel:  2, direction:  1, offset_deg: 0.0, min_deg:   5.0, max_deg: 180.0 },
  { joint: "FL_coxa_joint",  channel:  3, direction:  1, offset_deg: 0.0, min_deg:  45.0, max_deg: 135.0 },
  { joint: "FL_femur_joint", channel:  4, direction:  1, offset_deg: 0.0, min_deg:  50.0, max_deg: 115.0 },
  { joint: "FL_tibia_joint", channel:  5, direction:  1, offset_deg: 0.0, min_deg:   5.0, max_deg: 180.0 },
  { joint: "RR_coxa_joint",  channel:  6, direction: -1, offset_deg: 0.0, min_deg:  45.0, max_deg: 135.0 },
  { joint: "RR_femur_joint", channel:  7, direction:  1, offset_deg: 0.0, min_deg:  50.0, max_deg: 115.0 },
  { joint: "RR_tibia_joint", channel:  8, direction:  1, offset_deg: 0.0, min_deg:   5.0, max_deg: 180.0 },
  { joint: "RL_coxa_joint",  channel:  9, direction: -1, offset_deg: 0.0, min_deg:  45.0, max_deg: 135.0 },
  { joint: "RL_femur_joint", channel: 10, direction:  1, offset_deg: 0.0, min_deg:  50.0, max_deg: 115.0 },
  { joint: "RL_tibia_joint", channel: 11, direction:  1, offset_deg: 0.0, min_deg:   5.0, max_deg: 180.0 },
];

/** Look up a calibration entry by joint name. */
export function findCalByJoint(jointName) {
  return SERVO_CAL_PER_JOINT.find((e) => e.joint === jointName) ?? null;
}

/** Look up a calibration entry by PCA9685 channel (0..11). */
export function findCalByChannel(channel) {
  return SERVO_CAL_PER_JOINT.find((e) => e.channel === channel) ?? null;
}

const RAD2DEG = 180.0 / Math.PI;
const DEG2RAD = Math.PI / 180.0;

/**
 * Joint-space radians → servo-horn degrees, respecting direction + offset.
 * Matches firmware: servo_deg = CENTER + direction * joint_deg + offset.
 * Does NOT clamp to min/max — use `clampServoDeg` for that.
 */
export function applyServoCal(jointName, jointRad) {
  const cal = findCalByJoint(jointName);
  if (!cal) return null;
  return SERVO_CENTER_DEG + cal.direction * jointRad * RAD2DEG + cal.offset_deg;
}

/** Reverse of applyServoCal. */
export function inverseServoCal(jointName, servoDeg) {
  const cal = findCalByJoint(jointName);
  if (!cal) return null;
  return (servoDeg - SERVO_CENTER_DEG - cal.offset_deg) / cal.direction * DEG2RAD;
}

/** Clamp a servo horn command into the calibration's min_deg..max_deg window. */
export function clampServoDeg(jointName, servoDeg) {
  const cal = findCalByJoint(jointName);
  if (!cal) return servoDeg;
  return Math.min(cal.max_deg, Math.max(cal.min_deg, servoDeg));
}

/**
 * Apply a user-editable overrides table on top of the defaults. The dashboard
 * persists overrides in ~/.argos/servo_overrides.json and broadcasts them as
 * `/dashboard/servo_overrides` (JSON string). Overrides can flip `direction`
 * or shift `offset_deg` per-joint — the clamp window is handled via the
 * separate joint_limits table (see persistence.js / dashboard_bridge_node).
 */
export function applyOverrides(baseTable, overrides = {}) {
  return baseTable.map((entry) => {
    const override = overrides[entry.joint];
    if (!override) return entry;
    return {
      ...entry,
      direction:  override.invert ? -entry.direction : entry.direction,
      offset_deg: entry.offset_deg + (override.offset_deg ?? 0.0),
    };
  });
}

// Self-consistency check — JOINT_NAMES row order must match the cal table.
if (SERVO_CAL_PER_JOINT.map((e) => e.joint).join(",") !== JOINT_NAMES.join(",")) {
  throw new Error(
    "SERVO_CAL_PER_JOINT joint order must match JOINT_NAMES.\n" +
      `  cal: ${SERVO_CAL_PER_JOINT.map((e) => e.joint).join(", ")}\n` +
      `  ros: ${JOINT_NAMES.join(", ")}`
  );
}
