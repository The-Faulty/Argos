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

  assert.ok(serial.lastAngles[1] < 90 - 28.84, "femur command should overshoot in the downward direction");
  assert.ok(serial.lastAngles[2] > 90 + 15.80, "tibia command should overshoot in the lifting direction");
});
