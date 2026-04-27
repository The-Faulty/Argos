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

  assert.ok(serial.lastAngles[1] < 90 - 25.99, "femur command should overshoot in the downward direction");
  assert.ok(serial.lastAngles[2] > 90 + 19.61, "tibia command should overshoot in the lifting direction");
});

test("backlash compensation is suppressed for swing-phase legs in trot", async () => {
  const serial = {
    setAllServoAnglesDeg(angles) {
      this.lastAngles = angles.slice();
    },
    releaseServos() {},
  };
  const mode = new ModeController({ serial, getStances: async () => ({}) });

  await mode.setMode("trot");
  mode.setTwist({ x: 0.5, y: 0, yaw: 0 });

  let sawSwingFemurMatch = false;
  let sawStanceFemurOvershoot = false;

  // ~30 ticks covers a full trot cycle (220 ms swing / 0.34 fraction ≈ 647 ms,
  // ≈16 ticks at 25 Hz) with margin for direction tracking to settle.
  for (let t = 0; t < 30; t++) {
    const out = mode.planner.step();
    if (!out || out.error) continue;
    mode._publishServoDeg(out.servoAnglesDeg, out.jointAnglesRad, out.phaseTag);

    for (let leg = 0; leg < 4; leg++) {
      const femurIdx = leg * 3 + 1;
      const desired = out.servoAnglesDeg[femurIdx];
      const commanded = serial.lastAngles[femurIdx];
      const diff = Math.abs(commanded - desired);

      if (out.phaseTag[leg] === "swing") {
        // No overshoot during swing — the 1.8° femur kick that destabilizes
        // the lift trajectory must not be added.
        assert.ok(
          diff < 0.5,
          `tick ${t} leg ${leg}: swing femur should not get backlash overshoot, got diff=${diff.toFixed(3)}°`,
        );
        sawSwingFemurMatch = true;
      } else if (t > 2 && diff > 0.5) {
        // Stance still gets the full overshoot once direction has stabilized.
        sawStanceFemurOvershoot = true;
      }
    }
  }

  assert.ok(sawSwingFemurMatch, "expected at least one swing-phase femur sample");
  assert.ok(sawStanceFemurOvershoot, "expected at least one stance-phase femur tick with overshoot still applied");
});
