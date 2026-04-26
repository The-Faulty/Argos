// Guardrail tests for shared/robot-config.js.
//
// JOINT_NAMES, JOINT_ROWS, LEG_IDS, and MODE_OPTIONS feed every panel and the
// gait planner. These tests pin the shape and the mode strings so a rename
// has to update the test file too — making drift impossible to land silently.

import test from "node:test";
import assert from "node:assert/strict";

import {
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  MODE_OPTIONS,
} from "../shared/robot-config.js";

test("LEG_IDS has the four expected legs in order", () => {
  assert.deepEqual(LEG_IDS, ["FR", "FL", "RR", "RL"]);
});

test("JOINT_NAMES is 12 entries built from LEG_IDS × JOINT_ROWS", () => {
  const expected = [];
  for (const leg of LEG_IDS) {
    for (const row of JOINT_ROWS) {
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
  // The gait planner only accepts 'extend'; a regression here would silently
  // break mode switching from the dashboard.
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
