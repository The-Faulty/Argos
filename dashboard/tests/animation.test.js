// Clip-library round-trip tests. animation.js has two public surfaces:
//   - validateClip: schema + leg-ID migration (andy-servo "front_left" etc.
//     → Argos FR/FL/RR/RL) for anything imported from the old dashboard.
//   - interpolateFullBodyClip: read back a {leg: {x,y,z}} sample at time t.
// Legacy 2-D clips (before the 3-DOF port) are converted via
// convertLegacyLegClip and land on a single leg, the rest start at stance.

import test from "node:test";
import assert from "node:assert/strict";
import {
  convertLegacyLegClip,
  interpolateFullBodyClip,
  validateClip,
} from "../shared/animation.js";
import { LEG_IDS } from "../shared/robot-config.js";

test("legacy clip conversion maps onto the selected Argos leg", () => {
  // andy-servo's clips had foot positions in millimeters and no `z`; the
  // converter promotes them into the 3-D track for one chosen Argos leg.
  const converted = convertLegacyLegClip(
    {
      name: "legacy-step",
      duration: 2,
      keyframes: [
        { time: 0, foot: { x: 0, y: 0 } },
        { time: 2, foot: { x: 20, y: -10 } },
      ],
    },
    { mode: "selected_leg", legId: "RR" },
  );

  // All four Argos legs must have a track, but only the chosen leg carries
  // the keyframes — the rest stay at stance (0 keyframes is the sentinel).
  for (const legId of LEG_IDS) {
    assert.ok(Array.isArray(converted.tracks[legId]));
  }
  assert.equal(converted.tracks.RR.length, 2);
  assert.equal(converted.tracks.FL.length, 0);
});

test("validateClip migrates andy-servo leg names to Argos IDs", () => {
  const clip = validateClip({
    name: "migrated",
    duration: 1,
    tracks: {
      front_left: [{ time: 0, foot: { x: 1, y: 2, z: -0.18 } }],
      rear_right: [{ time: 0, foot: { x: 3, y: 4, z: -0.18 } }],
    },
  });
  // After validation the keys are FR/FL/RR/RL.
  assert.equal(clip.tracks.FL.length, 1);
  assert.equal(clip.tracks.RR.length, 1);
  assert.ok(clip.tracks.FR !== undefined);
  assert.ok(clip.tracks.RL !== undefined);
});

test("interpolateFullBodyClip returns per-leg {x,y,z} at time t", () => {
  const clip = validateClip({
    name: "full-body",
    duration: 2,
    tracks: {
      FL: [
        { time: 0, foot: { x: 0,  y: 0,  z: -0.18 } },
        { time: 2, foot: { x: 20, y: 10, z: -0.16 } },
      ],
      FR: [{ time: 0, foot: { x: 5, y: -5, z: -0.18 } }],
      RL: [{ time: 0, foot: { x: -5, y: 5, z: -0.18 } }],
      RR: [{ time: 0, foot: { x: 10, y: 0, z: -0.18 } }],
    },
  });

  const frame = interpolateFullBodyClip(clip, 1.0);
  // FL interpolates halfway between the two keyframes.
  assert.equal(frame.FL.x, 10);
  assert.equal(frame.FL.y, 5);
  assert.ok(Math.abs(frame.FL.z - -0.17) < 1e-9);
  // Single-keyframe legs hold their pose.
  assert.equal(frame.FR.x, 5);
  assert.equal(frame.FR.y, -5);
});
