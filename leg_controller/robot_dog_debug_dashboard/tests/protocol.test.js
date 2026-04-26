import test from "node:test";
import assert from "node:assert/strict";
import { normalizeDriveCommand, normalizeStance, validateCommand } from "../shared/protocol.js";

test("normalizeDriveCommand clamps each axis", () => {
  assert.deepEqual(normalizeDriveCommand({ vx: 2, vy: -2, yawRate: 0.5, source: "web" }), {
    vx: 1,
    vy: -1,
    yawRate: 0.5,
    source: "web",
  });
});

test("validateCommand accepts high-level drive commands", () => {
  const command = validateCommand({
    type: "set_drive_command",
    drive: { vx: 0.75, vy: -0.25, yawRate: 0.4, source: "keyboard" },
    stance: { height: 12 },
  });

  assert.equal(command.drive.vx, 0.75);
  assert.equal(command.drive.source, "keyboard");
  assert.equal(command.stance.height, 12);
});

test("normalizeStance clamps stance height to the configured range", () => {
  assert.equal(normalizeStance({ height: 200 }).height, 35);
  assert.equal(normalizeStance({ height: -200 }).height, -25);
});

test("validateCommand accepts stance height commands", () => {
  const command = validateCommand({
    type: "set_stance",
    stance: { height: 18 },
  });

  assert.equal(command.stance.height, 18);
});

test("validateCommand rejects invalid motion mode", () => {
  assert.throws(() => validateCommand({ type: "set_motion_mode", mode: "fly" }));
});

test("validateCommand accepts flattened full-body pose commands", () => {
  const command = validateCommand({
    type: "apply_full_body_pose",
    FLHipYawDeg: 90,
    FLThighDeg: 70,
    FLCalfDeg: 85,
    FRHipYawDeg: 90,
    FRThighDeg: 72,
    FRCalfDeg: 88,
    RLHipYawDeg: 90,
    RLThighDeg: 71,
    RLCalfDeg: 86,
    RRHipYawDeg: 90,
    RRThighDeg: 73,
    RRCalfDeg: 89,
  });

  assert.equal(command.type, "apply_full_body_pose");
});

test("validateCommand accepts per-leg servo trim commands", () => {
  const command = validateCommand({
    type: "set_leg_servo_trim",
    legId: "front_left",
    hipYawOffsetDeg: 2,
    thighOffsetDeg: -6,
    calfOffsetDeg: 4,
  });

  assert.equal(command.legId, "front_left");
});

test("validateCommand accepts three-channel servo map commands", () => {
  const command = validateCommand({
    type: "set_leg_servo_channel_map",
    legId: "front_left",
    hipYawChannel: 0,
    thighChannel: 1,
    calfChannel: 2,
  });

  assert.equal(command.hipYawChannel, 0);
  assert.equal(command.calfChannel, 2);
});

test("validateCommand accepts three-axis joint limit commands", () => {
  const command = validateCommand({
    type: "set_leg_joint_limits",
    legId: "front_left",
    hipYawMinDeg: -35,
    hipYawMaxDeg: 35,
    thighMinDeg: -145,
    thighMaxDeg: 15,
    calfMinDeg: -165,
    calfMaxDeg: -25,
  });

  assert.equal(command.hipYawMaxDeg, 35);
  assert.equal(command.thighMinDeg, -145);
});

test("validateCommand accepts three-axis servo speed limit commands", () => {
  const command = validateCommand({
    type: "set_leg_servo_speed_limit",
    legId: "front_left",
    hipYawDegPerSec: 120,
    thighDegPerSec: 180,
    calfDegPerSec: 160,
  });

  assert.equal(command.hipYawDegPerSec, 120);
  assert.equal(command.calfDegPerSec, 160);
});
