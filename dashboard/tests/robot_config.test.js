// Guardrail tests for shared/robot-config.js.
//
// The JS config has to stay in sync with two Python sources of truth:
//   - ros_support.JOINT_NAMES (authoritative joint-name ordering)
//   - gait_planner_node.py's allowed mode set + dashboard MODE_OPTIONS
//
// We don't import the Python files (no Python runtime in node --test).
// Instead we pin the expected values here and cross-check by reading the
// Python files as text — any future drift on either side raises an obvious
// diff. If you rename a joint, update BOTH files + this test together.

import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import {
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  MODE_OPTIONS,
} from "../shared/robot-config.js";

const HERE = dirname(fileURLToPath(import.meta.url));
const REPO = resolve(HERE, "..", "..");

function readRosSupport() {
  return readFileSync(
    resolve(REPO, "ros2_ws", "argos_control", "ros_support.py"),
    "utf8",
  );
}

function parsePyTuple(src, name) {
  // Match `NAME = ( "a", "b", ... )` or `NAME = [ ... ]`. Returns the string
  // entries in declaration order.
  const re = new RegExp(`^${name}\\s*=\\s*[\\(\\[]([^\\)\\]]+)[\\)\\]]`, "m");
  const match = src.match(re);
  if (!match) return null;
  return match[1]
    .split(",")
    .map((s) => s.trim().replace(/['"]/g, ""))
    .filter(Boolean);
}

test("LEG_IDS matches ros_support.LEG_ORDER", () => {
  const pyOrder = parsePyTuple(readRosSupport(), "LEG_ORDER");
  assert.ok(pyOrder, "Could not locate LEG_ORDER in ros_support.py");
  assert.deepEqual(LEG_IDS, pyOrder);
});

test("JOINT_NAMES matches the ros_support comprehension", () => {
  // JOINT_NAMES in ros_support.py is built via a comprehension:
  //   [f"{leg}_{joint}_joint" for leg in LEG_ORDER for joint in JOINT_ROWS]
  // We reconstruct the same ordering from the parsed tuples and compare.
  const src = readRosSupport();
  const pyLegs = parsePyTuple(src, "LEG_ORDER");
  const pyRows = parsePyTuple(src, "JOINT_ROWS");
  assert.ok(pyLegs && pyRows, "Could not locate LEG_ORDER / JOINT_ROWS");
  const expected = [];
  for (const leg of pyLegs) {
    for (const row of pyRows) {
      expected.push(`${leg}_${row}_joint`);
    }
  }
  assert.equal(JOINT_NAMES.length, 12);
  assert.deepEqual(JOINT_NAMES, expected);
});

test("JOINT_NAMES row suffixes map to JOINT_ROWS for all legs", () => {
  for (const leg of LEG_IDS) {
    for (const row of JOINT_ROWS) {
      assert.ok(
        JOINT_NAMES.includes(`${leg}_${row}_joint`),
        `Missing ${leg}_${row}_joint in JOINT_NAMES`,
      );
    }
  }
});

test("MODE_OPTIONS uses 'extend' not 'extended'", () => {
  // This matches the gait_planner_node's allowed string set — the Python
  // side only accepts 'extend'; a regression here would silently break mode
  // switching from the dashboard.
  assert.ok(MODE_OPTIONS.includes("extend"));
  assert.ok(!MODE_OPTIONS.includes("extended"));
});

test("MODE_OPTIONS covers gait planner's walking modes", () => {
  // The dashboard must expose everything the gait planner understands, plus
  // the bridge-only direct modes and animation_playback.
  for (const m of ["idle", "stand", "crouch", "extend", "crawl", "trot"]) {
    assert.ok(MODE_OPTIONS.includes(m), `missing mode "${m}"`);
  }
  for (const m of ["direct_foot_xyz", "direct_joint_angles", "direct_servo_angles", "animation_playback"]) {
    assert.ok(MODE_OPTIONS.includes(m), `missing mode "${m}"`);
  }
});
