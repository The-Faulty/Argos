import test from "node:test";
import assert from "node:assert/strict";
import { createMotionStatePatch, flattenServoPose } from "../shared/locomotion.js";

test("createMotionStatePatch produces four desired leg poses", () => {
  const patch = createMotionStatePatch({
    motionMode: "drive",
    driveCommand: { vx: 1, vy: 0.25, yawRate: -0.25, source: "test" },
    timeMs: 500,
  });

  assert.equal(patch.motionMode, "drive");
  assert.equal(Object.keys(patch.legs).length, 4);
  assert.equal(typeof patch.legs.front_left.desired.servoAnglesDeg.hipYaw, "number");
});

test("flattenServoPose emits flattened servo fields for serial transport", () => {
  const patch = createMotionStatePatch({
    motionMode: "stand",
    driveCommand: { vx: 0, vy: 0, yawRate: 0, source: "test" },
    timeMs: 0,
  });
  const flattened = flattenServoPose(patch.legs);

  assert.ok("FLHipYawDeg" in flattened);
  assert.ok("RRCalfDeg" in flattened);
});

test("flattenServoPose mirrors right legs around 90 degrees", () => {
  const flattened = flattenServoPose({
    front_left: { desired: { servoAnglesDeg: { hipYaw: 95, thigh: 80, calf: 100 } }, servoTrimDeg: { hipYaw: 0, thigh: 0, calf: 0 } },
    front_right: { desired: { servoAnglesDeg: { hipYaw: 95, thigh: 80, calf: 100 } }, servoTrimDeg: { hipYaw: 0, thigh: 0, calf: 0 } },
    rear_left: { desired: { servoAnglesDeg: { hipYaw: 95, thigh: 80, calf: 100 } }, servoTrimDeg: { hipYaw: 0, thigh: 0, calf: 0 } },
    rear_right: { desired: { servoAnglesDeg: { hipYaw: 95, thigh: 80, calf: 100 } }, servoTrimDeg: { hipYaw: 0, thigh: 0, calf: 0 } },
  });

  assert.equal(flattened.FLThighDeg, 80);
  assert.equal(flattened.FRThighDeg, 100);
  assert.equal(flattened.RRHipYawDeg, 95);
  assert.equal(flattened.RRCalfDeg, 80);
});

test("flattenServoPose applies servo neutral trims after mirroring", () => {
  const flattened = flattenServoPose({
    front_left: { desired: { servoAnglesDeg: { hipYaw: 90, thigh: 90, calf: 90 } }, servoTrimDeg: { hipYaw: 2, thigh: -3, calf: 4 } },
    front_right: { desired: { servoAnglesDeg: { hipYaw: 90, thigh: 90, calf: 90 } }, servoTrimDeg: { hipYaw: -2, thigh: 3, calf: -4 } },
    rear_left: { desired: { servoAnglesDeg: { hipYaw: 90, thigh: 90, calf: 90 } }, servoTrimDeg: { hipYaw: 0, thigh: 0, calf: 0 } },
    rear_right: { desired: { servoAnglesDeg: { hipYaw: 90, thigh: 90, calf: 90 } }, servoTrimDeg: { hipYaw: 0, thigh: 0, calf: 0 } },
  });

  assert.equal(flattened.FLHipYawDeg, 92);
  assert.equal(flattened.FLThighDeg, 87);
  assert.equal(flattened.FRThighDeg, 93);
  assert.equal(flattened.FRCalfDeg, 86);
});

test("createMotionStatePatch exaggerates movement while staying periodic", () => {
  const early = createMotionStatePatch({
    motionMode: "drive",
    driveCommand: { vx: 1, vy: 0, yawRate: 0, source: "test" },
    timeMs: 0,
  });
  const later = createMotionStatePatch({
    motionMode: "drive",
    driveCommand: { vx: 1, vy: 0, yawRate: 0, source: "test" },
    timeMs: 600,
  });

  assert.notEqual(early.legs.front_left.desired.foot.x, later.legs.front_left.desired.foot.x);
  assert.ok(Math.abs(early.legs.front_left.desired.foot.y - later.legs.front_left.desired.foot.y) > 5);
});
