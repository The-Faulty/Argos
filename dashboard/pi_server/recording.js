// CSV telemetry recorder.
//
// Writes one row per sample under `~/argos_recordings/<ISO-timestamp>.csv`.
// Columns are stable: timestamp + flattened JointState positions (12 cols,
// JOINT_NAMES order) + roll/pitch/yaw + gas + cmd_vel{x,y,z,wx,wy,wz}.
//
// Streaming strategy:
//   - Single owner (the server keeps at most one active recorder).
//   - Every append goes through a write queue so interleaved samples don't
//     mangle rows.
//   - On `stop()`, we drain the queue before resolving.
//
// Rotation strategy:
//   - Each call to `start()` opens a fresh file, so "rotation" = operator
//     clicking stop + start again. That matches the UX contract — we don't
//     want silent mid-session file splits.

import { promises as fs } from "node:fs";
import { createWriteStream } from "node:fs";
import path from "node:path";
import os from "node:os";

import { JOINT_NAMES } from "../shared/robot-config.js";

export const RECORDING_DIR = path.join(os.homedir(), "argos_recordings");

const HEADER_COLUMNS = [
  "timestamp_iso",
  "timestamp_epoch_ms",
  ...JOINT_NAMES.map((n) => `pos_${n}`),
  "imu_roll",
  "imu_pitch",
  "imu_yaw",
  "gas_ppm",
  "cmd_lin_x",
  "cmd_lin_y",
  "cmd_lin_z",
  "cmd_ang_x",
  "cmd_ang_y",
  "cmd_ang_z",
];

function csvEscape(value) {
  if (value === undefined || value === null) return "";
  const str = typeof value === "number" ? Number(value).toString() : String(value);
  if (/[",\n\r]/.test(str)) return `"${str.replace(/"/g, '""')}"`;
  return str;
}

export class TelemetryRecorder {
  constructor() {
    this.stream = null;
    this.filePath = null;
    this.startedAt = null;
    this.rowCount = 0;
    this._writeChain = Promise.resolve();
  }

  get isActive() {
    return this.stream !== null;
  }

  status() {
    return {
      active: this.isActive,
      filePath: this.filePath,
      startedAt: this.startedAt,
      rowCount: this.rowCount,
    };
  }

  async start() {
    if (this.isActive) {
      return this.status();
    }
    await fs.mkdir(RECORDING_DIR, { recursive: true });
    const iso = new Date().toISOString().replace(/[:.]/g, "-");
    this.filePath = path.join(RECORDING_DIR, `${iso}.csv`);
    this.stream = createWriteStream(this.filePath, { flags: "w" });
    this.startedAt = new Date().toISOString();
    this.rowCount = 0;
    await this._write(HEADER_COLUMNS.join(",") + "\n");
    return this.status();
  }

  async stop() {
    if (!this.isActive) return this.status();
    await this._writeChain;
    await new Promise((resolve, reject) => {
      this.stream.end((err) => (err ? reject(err) : resolve()));
    });
    const finalStatus = { ...this.status(), active: false };
    this.stream = null;
    return finalStatus;
  }

  // Accepts a "sample" snapshot the server assembles from the latest topic
  // cache. Missing fields are written as empty cells so the CSV columns stay
  // fixed-width.
  append(sample = {}) {
    if (!this.isActive) return;
    const now = new Date();
    const row = [
      now.toISOString(),
      now.getTime(),
      ...JOINT_NAMES.map((name) => {
        const positions = sample.joint_positions_by_name;
        if (positions && Number.isFinite(positions[name])) return positions[name];
        return "";
      }),
      sample.imu_roll ?? "",
      sample.imu_pitch ?? "",
      sample.imu_yaw ?? "",
      sample.gas_ppm ?? "",
      sample.cmd_lin_x ?? "",
      sample.cmd_lin_y ?? "",
      sample.cmd_lin_z ?? "",
      sample.cmd_ang_x ?? "",
      sample.cmd_ang_y ?? "",
      sample.cmd_ang_z ?? "",
    ];
    this.rowCount += 1;
    this._write(row.map(csvEscape).join(",") + "\n");
  }

  _write(chunk) {
    this._writeChain = this._writeChain.then(
      () =>
        new Promise((resolve, reject) => {
          if (!this.stream) return resolve();
          this.stream.write(chunk, (err) => (err ? reject(err) : resolve()));
        }),
    );
    return this._writeChain;
  }
}

export async function listRecordings() {
  try {
    const entries = await fs.readdir(RECORDING_DIR, { withFileTypes: true });
    return entries
      .filter((e) => e.isFile() && e.name.endsWith(".csv"))
      .map((e) => path.join(RECORDING_DIR, e.name))
      .sort();
  } catch (err) {
    if (err.code === "ENOENT") return [];
    throw err;
  }
}
