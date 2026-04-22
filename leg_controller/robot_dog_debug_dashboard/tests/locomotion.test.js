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
