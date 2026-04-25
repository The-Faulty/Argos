import test from "node:test";
import assert from "node:assert/strict";
import { normalizeDriveCommand, validateCommand } from "../shared/protocol.js";

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
  });

  assert.equal(command.drive.vx, 0.75);
  assert.equal(command.drive.source, "keyboard");
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
