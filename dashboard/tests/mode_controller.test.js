import assert from "node:assert/strict";
import test from "node:test";

import { ModeController } from "../pi_server/mode_controller.js";

const DEG2RAD = Math.PI / 180.0;
const RAD2DEG = 180.0 / Math.PI;

test("direct joint commands are clamped to the coupled femur/tibia envelope", async () => {
  const serial = {
    setAllServoAnglesDeg(angles) {
      this.lastAngles = angles;
    },
    releaseServos() {},
  };
  const mode = new ModeController({ serial, getStances: async () => ({}) });

  const bad = [];
  for (let leg = 0; leg < 4; leg++) {
    bad.push(0, -50 * DEG2RAD, 90 * DEG2RAD);
  }

  await mode.setJointAngles(bad);

  const snapshot = mode.getSnapshot();
  for (let leg = 0; leg < 4; leg++) {
    const femurDeg = snapshot.joint_states.position[leg * 3 + 1] * RAD2DEG;
    const tibiaDeg = snapshot.joint_states.position[leg * 3 + 2] * RAD2DEG;
    assert.equal(Math.round(femurDeg), -50);
    assert.ok(tibiaDeg <= 5.001, `tibia should clamp to +5 deg at femur=-50, got ${tibiaDeg}`);
  }
  assert.equal(serial.lastAngles.length, 12);
});

test("auto poses apply small directional backlash compensation", async () => {
  const serial = {
    setAllServoAnglesDeg(angles) {
      this.lastAngles = angles;
    },
    releaseServos() {},
  };
  const mode = new ModeController({ serial, getStances: async () => ({}) });

  await mode.setMode("stand");

  // STAND_JOINTS_RAD is [0, -46.75°, -30.25°] → femur servo 43.25, tibia
  // servo 59.75. Both move downward from the 90° rest baseline, so the
  // direction-tracking backlash comp pushes the commanded servos a few
  // degrees PAST those targets (i.e., even smaller than the desired
  // servo), regardless of whether tibia is on the "up" or "down" branch.
  assert.ok(serial.lastAngles[1] < 43.25, `femur command should overshoot past stand target, got ${serial.lastAngles[1]}`);
  assert.ok(serial.lastAngles[2] < 59.75, `tibia command should overshoot past stand target, got ${serial.lastAngles[2]}`);
});

test("backlash compensation is suppressed for both swing and stance legs during trot/crawl", async () => {
  const serial = {
    setAllServoAnglesDeg(angles) {
      this.lastAngles = angles.slice();
    },
    releaseServos() {},
  };
  const mode = new ModeController({ serial, getStances: async () => ({}) });

  await mode.setMode("trot");
  mode.setTwist({ x: 0.5, y: 0, yaw: 0 });

  // The 1.8°/2.4° femur/tibia kick was visibly jittery on hardware during
  // gait — comp is now globally suppressed for crawl/trot regardless of
  // per-leg phase. Static modes (stand/extend/crouch/direct_foot_xyz) still
  // get the comp; that's covered by the "auto poses apply small directional
  // backlash compensation" test above.
  let sawSwingSample = false;
  let sawStanceSample = false;

  for (let t = 0; t < 30; t++) {
    const out = mode.planner.step();
    if (!out || out.error) continue;
    mode._publishServoDeg(out.servoAnglesDeg, out.jointAnglesRad, out.phaseTag);

    for (let leg = 0; leg < 4; leg++) {
      const femurIdx = leg * 3 + 1;
      const desired = out.servoAnglesDeg[femurIdx];
      const commanded = serial.lastAngles[femurIdx];
      const diff = Math.abs(commanded - desired);

      assert.ok(
        diff < 0.5,
        `tick ${t} leg ${leg} (${out.phaseTag[leg]}): trot femur should NOT get backlash overshoot, got diff=${diff.toFixed(3)}°`,
      );

      if (out.phaseTag[leg] === "swing") sawSwingSample = true;
      else sawStanceSample = true;
    }
  }

  assert.ok(sawSwingSample, "expected at least one swing-phase femur sample");
  assert.ok(sawStanceSample, "expected at least one stance-phase femur sample");
});
