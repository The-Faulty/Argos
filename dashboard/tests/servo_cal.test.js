// Self-consistency tests for shared/servo_cal.js.
//
// The firmware's PCA9685 channel/direction table is the real source of
// truth (firmware/argos_servo/argos_servo.ino::LEG_CONFIGS). These tests
// pin the JS mirror's shape and round-trip behavior so calibration drift
// or a bad refactor on the JS side fails fast.

import test from "node:test";
import assert from "node:assert/strict";

import {
  SERVO_CAL_PER_JOINT,
  findCalByJoint,
  findCalByChannel,
  applyServoCal,
  inverseServoCal,
  clampServoDeg,
} from "../shared/servo_cal.js";
import { JOINT_NAMES, SERVO_CENTER_DEG } from "../shared/robot-config.js";

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
