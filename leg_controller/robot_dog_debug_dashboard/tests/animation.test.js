import test from "node:test";
import assert from "node:assert/strict";
import { convertLegacyLegClip, interpolateFullBodyClip, validateClip } from "../shared/animation.js";

test("legacy clip conversion maps to selected leg", () => {
  const converted = convertLegacyLegClip(
    {
      name: "legacy-step",
      duration: 2,
      keyframes: [
        { time: 0, foot: { x: 0, y: 0 } },
        { time: 2, foot: { x: 20, y: -10 } }
      ]
    },
    { mode: "selected_leg", legId: "rear_right" }
  );

  assert.equal(converted.tracks.rear_right.length, 2);
  assert.equal(converted.tracks.front_left.length, 0);
});

test("full body interpolation returns per-leg foot targets", () => {
  const clip = validateClip({
    name: "full-body",
    duration: 2,
    tracks: {
      front_left: [
        { time: 0, foot: { x: 0, y: 0 } },
        { time: 2, foot: { x: 20, y: 10 } }
      ],
      front_right: [{ time: 0, foot: { x: 5, y: -5 } }],
      rear_left: [{ time: 0, foot: { x: -5, y: 5 } }],
      rear_right: [{ time: 0, foot: { x: 10, y: 0 } }]
    }
  });

  const frame = interpolateFullBodyClip(clip, 1);
  assert.equal(frame.front_left.x, 10);
  assert.equal(frame.front_left.y, 5);
  assert.equal(frame.front_right.x, 5);
});
