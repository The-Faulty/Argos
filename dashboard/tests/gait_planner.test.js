import test from "node:test";
import assert from "node:assert/strict";

import { GaitPlanner } from "../pi_server/gait_planner.js";

test("held forward trot keeps producing reachable gait poses", () => {
  const planner = new GaitPlanner();
  planner.setMode("trot");
  planner.setTwist({ x: 0.5, y: 0, yaw: 0 });

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
    // Rear legs keep a 3 mm clamp budget at full backward sweep.
    assert.ok(span > 0.054, `expected exaggerated per-leg x travel, got ${span}`);
  }
});

test("held forward crawl keeps producing exaggerated reachable gait poses", () => {
  const planner = new GaitPlanner();
  planner.setMode("crawl");
  planner.setTwist({ x: 0.5, y: 0, yaw: 0 });

  const ranges = Array.from({ length: 4 }, () => ({ min: Infinity, max: -Infinity }));

  for (let tick = 0; tick < 180; tick++) {
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
    // Rear legs keep a 3 mm clamp budget at full backward sweep.
    assert.ok(span > 0.054, `expected exaggerated per-leg x travel, got ${span}`);
  }
});

test("held forward plus yaw trot stays reachable with higher swing lift", () => {
  const planner = new GaitPlanner();
  planner.setMode("trot");
  planner.setTwist({ x: 0.5, y: 0, yaw: 2.0 });

  for (let tick = 0; tick < 240; tick++) {
    const out = planner.step();
    assert.ok(out, `tick ${tick}: expected gait output`);
    assert.equal(out.error, undefined, `tick ${tick}: ${out.error}`);
  }
});

test("held trot rotation produces visible reachable yaw stride", () => {
  const planner = new GaitPlanner();
  planner.setMode("trot");
  planner.setTwist({ x: 0, y: 0, yaw: 2.0 });

  const ranges = Array.from({ length: 4 }, () => ({ minX: Infinity, maxX: -Infinity, minY: Infinity, maxY: -Infinity }));

  for (let tick = 0; tick < 180; tick++) {
    const out = planner.step();
    assert.ok(out, `tick ${tick}: expected gait output`);
    assert.equal(out.error, undefined, `tick ${tick}: ${out.error}`);
    for (let leg = 0; leg < out.feet.length; leg++) {
      ranges[leg].minX = Math.min(ranges[leg].minX, out.feet[leg][0]);
      ranges[leg].maxX = Math.max(ranges[leg].maxX, out.feet[leg][0]);
      ranges[leg].minY = Math.min(ranges[leg].minY, out.feet[leg][1]);
      ranges[leg].maxY = Math.max(ranges[leg].maxY, out.feet[leg][1]);
    }
  }

  for (const range of ranges) {
    const xSpan = range.maxX - range.minX;
    const ySpan = range.maxY - range.minY;
    assert.ok(xSpan > 0.030, `expected visible yaw x travel, got ${xSpan}`);
    assert.ok(ySpan > 0.020, `expected visible yaw y travel, got ${ySpan}`);
  }
});
