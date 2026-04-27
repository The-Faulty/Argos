// Guardrail tests for shared/robot-config.js.
//
// JOINT_NAMES, JOINT_ROWS, LEG_IDS, and MODE_OPTIONS feed every panel and the
// gait planner. These tests pin the shape and the mode strings so a rename
// has to update the test file too — making drift impossible to land silently.

import test from "node:test";
import assert from "node:assert/strict";

import {
  COUPLED_BOT_MAX_SAMPLES_DEG,
  COUPLED_BOT_MIN_SAMPLES_DEG,
  COUPLED_TOP_SAMPLES_DEG,
  DEFAULT_JOINT_LIMITS_RAD,
  DEFAULT_ROTATE_SETTINGS,
  DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
  GAIT_TUNABLE_PARAMS,
  JOINT_NAMES,
  JOINT_ROWS,
  JOYSTICK_SCALE,
  LEG_IDS,
  MODE_OPTIONS,
  SERVO_LIMITS_DEG,
} from "../shared/robot-config.js";

const RAD2DEG = 180 / Math.PI;

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

test("default joint limits match measured leg travel", () => {
  assert.deepEqual(
    DEFAULT_JOINT_LIMITS_RAD.femur.map((rad) => Math.round(rad * RAD2DEG)),
    [-90, 0],
  );
  assert.deepEqual(
    DEFAULT_JOINT_LIMITS_RAD.tibia.map((rad) => Math.round(rad * RAD2DEG)),
    [-90, 90],
  );
});

test("servo horn limits match measured safe travel", () => {
  assert.deepEqual(SERVO_LIMITS_DEG[1], [0.0, 90.0]);
  assert.deepEqual(SERVO_LIMITS_DEG[2], [0.0, 180.0]);
});

test("coupled femur/tibia samples match measured safe envelope", () => {
  assert.deepEqual(COUPLED_TOP_SAMPLES_DEG, [0.0, 40.0, 52.0, 55.0, 65.0, 77.0, 90.0]);
  assert.deepEqual(COUPLED_BOT_MIN_SAMPLES_DEG, [0.0, 0.0, 0.0, 0.0, 20.0, 40.0, 65.0]);
  assert.deepEqual(COUPLED_BOT_MAX_SAMPLES_DEG, [0.0, 95.0, 120.0, 127.0, 150.0, 180.0, 180.0]);
});

test("walking and rotate defaults are tuned for controlled motion", () => {
  // MAX_LIN_VEL is the velocity the planner sees on a full-deflection arrow
  // hold. With MAX_STRIDE_X = 0.060 m and ~427 ms stance, 0.25 m/s yields
  // ~0.107 m peak-to-peak stride, exercising the new cap (0.120 m
  // peak-to-peak) without saturating off the bottom. Changing this also
  // requires re-checking that full-stick stride still fits inside
  // DEFAULT_FOOT_REACH_X without invoking the per-leg blend-back.
  assert.equal(JOYSTICK_SCALE.MAX_LIN_VEL, 0.25);
  assert.equal(JOYSTICK_SCALE.MAX_ANG_VEL, 1.4);
  assert.equal(GAIT_TUNABLE_PARAMS.swing_time_ms.default, 220);
  assert.deepEqual(DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC, {
    coxa: 120.0,
    femur: 180.0,
    tibia: 240.0,
  });
  assert.deepEqual(DEFAULT_ROTATE_SETTINGS, {
    rotate_increment_deg: 20.0,
    rotate_rate_rad_s: 1.4,
    rotate_calibration_factor: 1.12,
  });
});
