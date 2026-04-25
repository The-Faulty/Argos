// Extracts ros2_ws/argos_control/Config.py::SERVO_CAL_PER_JOINT at test time
// and diffs it against the JS mirror. If either the firmware or the Python
// Config drifts without updating this JS table, the dashboard's direct-servo
// slider + overrides get pointed at the wrong channel and the robot binds.
//
// The firmware table in firmware/esp32c6/main/main.c::s_servo_cal is the real
// source of truth. Keeping JS ↔ Python in sync here is the lightweight
// version of that guarantee — a third test could parse the C file too.

import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import {
  SERVO_CAL_PER_JOINT,
  findCalByJoint,
  findCalByChannel,
  applyServoCal,
  inverseServoCal,
  clampServoDeg,
} from "../shared/servo_cal.js";
import { JOINT_NAMES, SERVO_CENTER_DEG } from "../shared/robot-config.js";

const HERE = dirname(fileURLToPath(import.meta.url));
const REPO = resolve(HERE, "..", "..");

/**
 * Parse the Python SERVO_CAL_PER_JOINT list into `[{joint, channel, direction,
 * offset_deg, min_deg, max_deg}, ...]`. We intentionally use a simple field-
 * per-field regex rather than trying to run Python — the table is stable and
 * only this one shape is expected.
 */
function parsePyCalTable() {
  const src = readFileSync(
    resolve(REPO, "ros2_ws", "argos_control", "Config.py"),
    "utf8",
  );
  const block = src.match(/SERVO_CAL_PER_JOINT\s*=\s*\[([\s\S]*?)\]/);
  if (!block) throw new Error("Could not find SERVO_CAL_PER_JOINT block");
  const entries = [];
  const rowRe = /\{\s*"joint"\s*:\s*"([^"]+)"\s*,\s*"channel"\s*:\s*(-?\d+)\s*,\s*"direction"\s*:\s*(-?\d+)\s*,\s*"offset_deg"\s*:\s*(-?\d+(?:\.\d+)?)\s*,\s*"min_deg"\s*:\s*(-?\d+(?:\.\d+)?)\s*,\s*"max_deg"\s*:\s*(-?\d+(?:\.\d+)?)\s*\}/g;
  let match;
  while ((match = rowRe.exec(block[1])) !== null) {
    entries.push({
      joint: match[1],
      channel: Number(match[2]),
      direction: Number(match[3]),
      offset_deg: Number(match[4]),
      min_deg: Number(match[5]),
      max_deg: Number(match[6]),
    });
  }
  return entries;
}

test("SERVO_CAL_PER_JOINT matches Python Config.py row-for-row", () => {
  const py = parsePyCalTable();
  assert.equal(py.length, 12, "Python table should have 12 rows");
  assert.equal(SERVO_CAL_PER_JOINT.length, 12);

  for (let i = 0; i < 12; i++) {
    const p = py[i];
    const j = SERVO_CAL_PER_JOINT[i];
    assert.equal(j.joint, p.joint, `row ${i} joint`);
    assert.equal(j.channel, p.channel, `row ${i} channel`);
    assert.equal(j.direction, p.direction, `row ${i} direction`);
    assert.equal(j.offset_deg, p.offset_deg, `row ${i} offset_deg`);
    assert.equal(j.min_deg, p.min_deg, `row ${i} min_deg`);
    assert.equal(j.max_deg, p.max_deg, `row ${i} max_deg`);
  }
});

test("SERVO_CAL_PER_JOINT joint order matches JOINT_NAMES", () => {
  const calOrder = SERVO_CAL_PER_JOINT.map((e) => e.joint);
  assert.deepEqual(calOrder, JOINT_NAMES);
});

test("Channels are 0..11 unique", () => {
  const channels = SERVO_CAL_PER_JOINT.map((e) => e.channel).sort((a, b) => a - b);
  assert.deepEqual(channels, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
});

test("Rear coxa joints have direction=-1 (firmware polarity)", () => {
  assert.equal(findCalByJoint("RR_coxa_joint").direction, -1);
  assert.equal(findCalByJoint("RL_coxa_joint").direction, -1);
  // Sanity: front coxa stays +1
  assert.equal(findCalByJoint("FR_coxa_joint").direction, +1);
  assert.equal(findCalByJoint("FL_coxa_joint").direction, +1);
});

test("findCalByChannel round-trips through findCalByJoint", () => {
  for (const entry of SERVO_CAL_PER_JOINT) {
    assert.equal(findCalByChannel(entry.channel).joint, entry.joint);
    assert.equal(findCalByJoint(entry.joint).channel, entry.channel);
  }
});

test("applyServoCal at joint=0 rad returns SERVO_CENTER_DEG + offset", () => {
  for (const entry of SERVO_CAL_PER_JOINT) {
    const deg = applyServoCal(entry.joint, 0.0);
    assert.equal(deg, SERVO_CENTER_DEG + entry.offset_deg);
  }
});

test("applyServoCal / inverseServoCal are inverses", () => {
  const sample = [-0.4, -0.1, 0.0, 0.15, 0.55];
  for (const entry of SERVO_CAL_PER_JOINT) {
    for (const rad of sample) {
      const deg = applyServoCal(entry.joint, rad);
      const back = inverseServoCal(entry.joint, deg);
      assert.ok(Math.abs(back - rad) < 1e-9, `inverse drift ${entry.joint}: ${back} vs ${rad}`);
    }
  }
});

test("clampServoDeg keeps outputs inside [min_deg, max_deg]", () => {
  for (const entry of SERVO_CAL_PER_JOINT) {
    assert.equal(clampServoDeg(entry.joint, -999), entry.min_deg);
    assert.equal(clampServoDeg(entry.joint, +999), entry.max_deg);
    const mid = 0.5 * (entry.min_deg + entry.max_deg);
    assert.equal(clampServoDeg(entry.joint, mid), mid);
  }
});
