// Atomic JSON persistence for Pi-side settings.
//
// The dashboard stores user-editable state outside the repo in `~/.argos/` so
// fresh checkouts never stomp on bench calibration:
//
//   servo_overrides.json  — per-joint invert / offset_deg overrides
//   joint_limits.json     — per-leg manual joint-angle clamps
//   stances.json          — saved stand / crouch / extend snapshots
//   rotate_settings.json  — rotate-by-degrees tuneables
//
// Writes go temp-file → fsync → rename so a crash mid-save can't leave a
// truncated JSON file that bricks the next dashboard boot. Schema validation
// runs on BOTH load and save so a hand-edited file can't inject garbage.

import { promises as fs } from "node:fs";
import path from "node:path";
import os from "node:os";

import {
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  DEFAULT_ROTATE_SETTINGS,
  ROTATE_SETTINGS_BOUNDS,
  DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
  SERVO_SPEED_LIMIT_BOUNDS_DEG_PER_SEC,
  DEFAULT_SERVO_UPDATE_RATE_HZ,
  SERVO_UPDATE_RATE_BOUNDS_HZ,
} from "../shared/robot-config.js";

export const ARGOS_STATE_DIR = path.join(os.homedir(), ".argos");

export const PERSIST_PATHS = {
  servo_overrides:    path.join(ARGOS_STATE_DIR, "servo_overrides.json"),
  joint_limits:       path.join(ARGOS_STATE_DIR, "joint_limits.json"),
  stances:            path.join(ARGOS_STATE_DIR, "stances.json"),
  rotate_settings:    path.join(ARGOS_STATE_DIR, "rotate_settings.json"),
  servo_speed:        path.join(ARGOS_STATE_DIR, "servo_speed.json"),
  servo_update_rate:  path.join(ARGOS_STATE_DIR, "servo_update_rate.json"),
};

async function ensureDir() {
  await fs.mkdir(ARGOS_STATE_DIR, { recursive: true });
}

async function readJsonIfExists(filePath) {
  try {
    const raw = await fs.readFile(filePath, "utf8");
    return JSON.parse(raw);
  } catch (err) {
    if (err.code === "ENOENT") return null;
    throw err;
  }
}

async function writeJsonAtomic(filePath, value) {
  await ensureDir();
  const tmpPath = `${filePath}.${process.pid}.${Date.now()}.tmp`;
  const body = JSON.stringify(value, null, 2);
  const handle = await fs.open(tmpPath, "w");
  try {
    await handle.writeFile(body, "utf8");
    await handle.sync();
  } finally {
    await handle.close();
  }
  await fs.rename(tmpPath, filePath);
}

// ─── Schema validators ───────────────────────────────────────────────────

function validateServoOverrides(raw) {
  const out = {};
  if (!raw || typeof raw !== "object") return out;
  for (const [key, value] of Object.entries(raw)) {
    if (!JOINT_NAMES.includes(key)) continue;
    if (!value || typeof value !== "object") continue;
    const invert = Boolean(value.invert);
    const offset = Number.isFinite(value.offset_deg) ? value.offset_deg : 0.0;
    if (!invert && offset === 0.0) continue; // drop no-op entries
    out[key] = { invert, offset_deg: offset };
  }
  return out;
}

function validateJointLimits(raw) {
  const out = {};
  if (!raw || typeof raw !== "object") return out;
  for (const [key, value] of Object.entries(raw)) {
    if (!JOINT_NAMES.includes(key)) continue;
    if (!value || typeof value !== "object") continue;
    if (!Number.isFinite(value.min_deg) || !Number.isFinite(value.max_deg)) continue;
    if (value.min_deg >= value.max_deg) continue;
    out[key] = { min_deg: value.min_deg, max_deg: value.max_deg };
  }
  return out;
}

function validateStances(raw) {
  const allowed = ["stand", "crouch", "extend"];
  const out = {};
  if (!raw || typeof raw !== "object") return out;
  for (const name of allowed) {
    const entry = raw[name];
    if (!entry || typeof entry !== "object") continue;
    if (!Array.isArray(entry.angles_rad) || entry.angles_rad.length !== JOINT_NAMES.length) continue;
    if (!entry.angles_rad.every(Number.isFinite)) continue;
    out[name] = {
      angles_rad: entry.angles_rad.map(Number),
      saved_at: typeof entry.saved_at === "string" ? entry.saved_at : new Date().toISOString(),
    };
  }
  return out;
}

function clampBound(bounds, value, fallback) {
  if (!Number.isFinite(value)) return fallback;
  return Math.min(bounds.max, Math.max(bounds.min, value));
}

function validateRotateSettings(raw) {
  const b = ROTATE_SETTINGS_BOUNDS;
  const d = DEFAULT_ROTATE_SETTINGS;
  const src = raw && typeof raw === "object" ? raw : {};
  return {
    rotate_increment_deg:      clampBound(b.rotate_increment_deg,      src.rotate_increment_deg,      d.rotate_increment_deg),
    rotate_rate_rad_s:         clampBound(b.rotate_rate_rad_s,         src.rotate_rate_rad_s,         d.rotate_rate_rad_s),
    rotate_calibration_factor: clampBound(b.rotate_calibration_factor, src.rotate_calibration_factor, d.rotate_calibration_factor),
  };
}

// Per-joint-row speed limits in deg/s (coxa/femur/tibia). The bridge node
// turns these into per-servo slew rates before forwarding /joint_command/raw.
function validateServoSpeed(raw) {
  const d = DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;
  const b = SERVO_SPEED_LIMIT_BOUNDS_DEG_PER_SEC;
  const src = raw && typeof raw === "object" ? raw : {};
  const out = {};
  for (const row of JOINT_ROWS) {
    out[row] = clampBound(b, src[row], d[row]);
  }
  return out;
}

// Scalar PWM / command update rate (Hz). Covers both how often the Pi
// re-publishes joint commands and how the firmware paces its PCA9685 writes.
function validateServoUpdateRate(raw) {
  const b = SERVO_UPDATE_RATE_BOUNDS_HZ;
  const d = DEFAULT_SERVO_UPDATE_RATE_HZ;
  const val = raw && typeof raw === "object" ? raw.hz ?? raw.value : raw;
  return { hz: clampBound(b, Number(val), d) };
}

// ─── Public API ──────────────────────────────────────────────────────────

export async function loadServoOverrides() {
  return validateServoOverrides(await readJsonIfExists(PERSIST_PATHS.servo_overrides));
}
export async function saveServoOverrides(raw) {
  const clean = validateServoOverrides(raw);
  await writeJsonAtomic(PERSIST_PATHS.servo_overrides, clean);
  return clean;
}

export async function loadJointLimits() {
  return validateJointLimits(await readJsonIfExists(PERSIST_PATHS.joint_limits));
}
export async function saveJointLimits(raw) {
  const clean = validateJointLimits(raw);
  await writeJsonAtomic(PERSIST_PATHS.joint_limits, clean);
  return clean;
}

export async function loadStances() {
  return validateStances(await readJsonIfExists(PERSIST_PATHS.stances));
}
export async function saveStances(raw) {
  const clean = validateStances(raw);
  await writeJsonAtomic(PERSIST_PATHS.stances, clean);
  return clean;
}
export async function upsertStance(name, anglesRad) {
  if (!["stand", "crouch", "extend"].includes(name)) {
    throw new Error(`Unknown stance name: ${name}`);
  }
  if (!Array.isArray(anglesRad) || anglesRad.length !== JOINT_NAMES.length) {
    throw new Error(`anglesRad must be length ${JOINT_NAMES.length}`);
  }
  const current = await loadStances();
  current[name] = {
    angles_rad: anglesRad.map(Number),
    saved_at: new Date().toISOString(),
  };
  return saveStances(current);
}

export async function loadRotateSettings() {
  return validateRotateSettings(await readJsonIfExists(PERSIST_PATHS.rotate_settings));
}
export async function saveRotateSettings(raw) {
  const clean = validateRotateSettings(raw);
  await writeJsonAtomic(PERSIST_PATHS.rotate_settings, clean);
  return clean;
}

export async function loadServoSpeed() {
  return validateServoSpeed(await readJsonIfExists(PERSIST_PATHS.servo_speed));
}
export async function saveServoSpeed(raw) {
  const clean = validateServoSpeed(raw);
  await writeJsonAtomic(PERSIST_PATHS.servo_speed, clean);
  return clean;
}

export async function loadServoUpdateRate() {
  return validateServoUpdateRate(await readJsonIfExists(PERSIST_PATHS.servo_update_rate));
}
export async function saveServoUpdateRate(raw) {
  const clean = validateServoUpdateRate(raw);
  await writeJsonAtomic(PERSIST_PATHS.servo_update_rate, clean);
  return clean;
}

// Utility used by the bridge node to broadcast a per-leg × per-joint view
// of the joint limit table. Dashboard stores limits keyed by JOINT_NAMES
// string; the bridge wants them indexed by (leg, joint_row).
export function jointLimitsByLeg(limitsByName) {
  const out = {};
  for (const leg of LEG_IDS) out[leg] = {};
  for (const [jointName, range] of Object.entries(limitsByName)) {
    const [leg, jointRow] = jointName.split("_"); // e.g. "FR_coxa_joint" → ["FR","coxa"]
    if (!LEG_IDS.includes(leg) || !JOINT_ROWS.includes(jointRow)) continue;
    out[leg][jointRow] = range;
  }
  return out;
}
