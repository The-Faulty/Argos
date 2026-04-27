import test from "node:test";
import assert from "node:assert/strict";

import { GaitPlanner } from "../pi_server/gait_planner.js";

test("held forward trot keeps producing reachable gait poses", () => {
  const planner = new GaitPlanner();
  planner.setMode("trot");
  planner.setTwist({ x: 0.4, y: 0, yaw: 0 });

  const ranges = Array.from({ length: 4 }, () => ({ min: Infinity, max: -Infinity }));

  for (let tick = 0; tick < 120; tick++) {
    const out = planner.step();
    assert.ok(out, `tick ${tick}: expected gait output`);
    assert.equal(out.error, undefined, `tick ${tick}: ${out.error}`);
    for (let leg = 0; leg < out.feet.length; leg++) {
      ranges[leg].min = Math.min(ranges[leg].min, out.feet[leg][0]);
      ranges[leg].max = Math.max(ranges[leg].max, out.feet[leg][0]);
    }
  }

  for (const range of ranges) {
    const span = range.max - range.min;
    assert.ok(span > 0.055, `expected exaggerated per-leg x travel, got ${span}`);
  }
});
