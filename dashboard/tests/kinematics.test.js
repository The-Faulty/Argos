// Argos 3-DOF IK parity test.
//
// Regenerate the fixture with:
//   python dashboard/tests/fixtures/gen_ik_golden.py
// which calls the authoritative Kinematics.py IK and writes a JSON grid of
// (feet → angles) samples. The JS port in shared/argos_kinematics.js must
// match each sample within one sweep-step of Python (0.1°, ≈ 1.75e-3 rad).
// Drift means the JS IK has desynced from the Python source — the dashboard's
// direct-foot-XYZ mode would then preview the wrong stance and animation
// scrub would disagree with what the real robot executes.
//
// The 3e-3 rad (≈ 0.17°) tolerance accounts for the two-level sweep in
// _sagittal_ik (coarse 2°, refine 0.1°) landing on different floating-point
// samples between interpreters — per-joint algorithmic drift would be far
// larger than this.
const TOL_RAD = 3e-3;

import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import {
  fourLegsInverseKinematics,
  clamp_joint_matrix,
} from "../shared/kinematics.js";
import { LEG_IDS, DEFAULT_JOINT_LIMITS_RAD } from "../shared/robot-config.js";

const HERE = dirname(fileURLToPath(import.meta.url));
const FIXTURE = resolve(HERE, "fixtures", "ik_golden.json");

function loadFixture() {
  const raw = readFileSync(FIXTURE, "utf8");
  return JSON.parse(raw);
}

test("golden fixture exists and has samples", () => {
  const g = loadFixture();
  assert.deepEqual(g.leg_order, ["FR", "FL", "RR", "RL"]);
  assert.ok(g.cases.length >= 5, `expected ≥5 cases, got ${g.cases.length}`);
});

test("fourLegsInverseKinematics matches Python within one sweep step (~0.17°)", () => {
  const g = loadFixture();
  let worst = 0;
  let worstAt = null;

  for (let i = 0; i < g.cases.length; i++) {
    const { feet_body_m, angles_rad } = g.cases[i];
    // Python feet are a (3, 4) matrix (rows = x/y/z, cols = leg). Convert to
    // the JS contract: feet = [[x,y,z] per leg].
    const feet = LEG_IDS.map((_, leg) => [
      feet_body_m[0][leg],
      feet_body_m[1][leg],
      feet_body_m[2][leg],
    ]);
    const res = fourLegsInverseKinematics(feet);
    assert.ok(res.ok, `Case ${i}: JS IK failed: ${res.reason}`);

    // Python angles_rad is (3, 4). JS res.angles is [4][3] (leg outer).
    for (let leg = 0; leg < 4; leg++) {
      for (let row = 0; row < 3; row++) {
        const expected = angles_rad[row][leg];
        const got = res.angles[leg][row];
        const err = Math.abs(expected - got);
        if (err > worst) {
          worst = err;
          worstAt = { case: i, leg: LEG_IDS[leg], row };
        }
        assert.ok(
          err < TOL_RAD,
          `Case ${i} ${LEG_IDS[leg]} row ${row}: got ${got}, expected ${expected} (err ${err.toExponential(3)})`,
        );
      }
    }
  }
  // Emit the worst error so small drifts show up as a diagnostic in logs.
  console.log(
    `  max per-joint error across ${g.cases.length} cases: ${worst.toExponential(3)} rad` +
      (worstAt ? ` at case ${worstAt.case} ${worstAt.leg}/row${worstAt.row}` : ""),
  );
});

test("fourLegsInverseKinematics returns angles[4][3] on success", () => {
  const g = loadFixture();
  const { feet_body_m } = g.cases[0];
  const feet = LEG_IDS.map((_, leg) => [
    feet_body_m[0][leg],
    feet_body_m[1][leg],
    feet_body_m[2][leg],
  ]);
  const res = fourLegsInverseKinematics(feet);
  assert.ok(res.ok);
  assert.equal(res.angles.length, 4, "must have 4 legs");
  for (const legAngles of res.angles) {
    assert.equal(legAngles.length, 3, "each leg has 3 joints");
  }
});

test("fourLegsInverseKinematics reports leg id on failure", () => {
  // Put one foot far outside the workspace; IK should fail with an id.
  const feet = [
    [0.10, -0.11, -0.189],
    [0.10, 0.11, -0.189],
    [-0.14, -0.11, -0.189],
    [5.0, 5.0, 5.0], // unreachable
  ];
  const res = fourLegsInverseKinematics(feet);
  assert.equal(res.ok, false);
  assert.equal(res.leg, "RL");
  assert.ok(typeof res.reason === "string" && res.reason.length > 0);
});

test("clamp_joint_matrix tightens the coupled envelope", () => {
  // A wildly-out-of-envelope joint matrix should be pulled back inside.
  // Shape is [4 legs][3 joints] = [coxa, femur, tibia]. Use extreme femur
  // + extreme tibia — clamp must produce values that respect both global
  // joint limits AND the coupled bot formula.
  const bad = [
    [0.0, 2.0, -2.0],    // FR: femur +115°, tibia -115° (outside)
    [0.0, -2.0, 2.0],    // FL
    [0.0, 2.0, -2.0],    // RR
    [0.0, -2.0, 2.0],    // RL
  ];
  const out = clamp_joint_matrix(bad);
  const femurLimits = DEFAULT_JOINT_LIMITS_RAD.femur;
  const tibiaLimits = DEFAULT_JOINT_LIMITS_RAD.tibia;
  for (const leg of out) {
    assert.ok(leg[1] >= femurLimits[0] - 1e-9 && leg[1] <= femurLimits[1] + 1e-9,
      `femur ${leg[1]} outside ${femurLimits}`);
    assert.ok(leg[2] >= tibiaLimits[0] - 1e-9 && leg[2] <= tibiaLimits[1] + 1e-9,
      `tibia ${leg[2]} outside ${tibiaLimits}`);
  }
});
