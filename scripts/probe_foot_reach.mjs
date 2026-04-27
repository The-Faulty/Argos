// Probe the per-leg foot-x reach window at the stance height AND at the
// lifted (swing-peak) height, using the same fourLegsInverseKinematics the
// gait planner runs. Output goes into robot-config.js as
// DEFAULT_FOOT_REACH_X (stance) and DEFAULT_FOOT_REACH_X_LIFTED (swing).
//
// Why two windows? The legacy single window was probed at the stance Z
// (-0.189 m) only, but the gait planner uses it to clamp BOTH stance and
// swing-phase feet. During swing the foot lifts ~25 mm, which changes the
// femur/tibia angles needed to hit a given (x, y) — the reachable x range
// narrows asymmetrically. Without a separate lifted window, swing-phase
// targets at the edges of the stance window would silently fail IK and the
// gait_planner would blend the foot back to neutral, shrinking the visible
// step size. Pair this with phase-aware clampFootInReach.
//
// Usage:   node scripts/probe_foot_reach.mjs
// Output is a JS-ready object you can paste into robot-config.js.

import {
  fourLegsInverseKinematics,
} from "../dashboard/shared/argos_kinematics.js";
import {
  LEG_IDS,
} from "../dashboard/shared/robot-config.js";

// The gait planner uses its own stance constants (gait_planner.js
// STAND_FRONT_FOOT_X / STAND_REAR_FOOT_X / STAND_FOOT_Y / STAND_FOOT_Z),
// NOT robot-config.js DEFAULT_STANCE — that was a separate mode's stance.
// Probe at the gait's actual neutral so the resulting reach window matches
// where clampFootInReach actually needs to clamp.
const GAIT_STANCE = {
  FR: [ 0.1000, -0.0733, -0.1950],
  FL: [ 0.1000,  0.0733, -0.1950],
  RR: [-0.1233, -0.0733, -0.1950],
  RL: [-0.1233,  0.0733, -0.1950],
};

const STANCE_Z = -0.1950;                // matches gait_planner STAND_FOOT_Z
const DEFAULT_LIFT_M = 0.018;            // matches gait_planner z_clearance

const args = parseArgs(process.argv.slice(2));
const liftM = args.liftMm != null ? args.liftMm / 1000 : DEFAULT_LIFT_M;
const stepM = args.stepMm != null ? args.stepMm / 1000 : 0.0005;  // 0.5 mm
const safetyMm = args.safetyMm != null ? args.safetyMm : 2;       // pull in 2 mm
const stanceZ = args.zMm != null ? args.zMm / 1000 : STANCE_Z;
if (args.yMm != null) {
  // Override every leg's lateral y so we can probe how stance-y affects the
  // reachable foot.x window. Keep the sign per-leg.
  const newY = args.yMm / 1000;
  for (const leg of LEG_IDS) {
    const stance = GAIT_STANCE[leg];
    GAIT_STANCE[leg] = [stance[0], stance[1] >= 0 ? newY : -newY, stance[2]];
  }
}

console.log(`# Probe params: stanceZ=${(stanceZ * 1000).toFixed(1)} mm, lift=${(liftM * 1000).toFixed(1)} mm, step=${(stepM * 1000).toFixed(2)} mm, safety=${safetyMm} mm pulled-in`);

const stanceWindows = probeAtZ(stanceZ, stepM, safetyMm);
const liftedWindows = probeAtZ(stanceZ + liftM, stepM, safetyMm);

console.log("");
console.log("export const DEFAULT_FOOT_REACH_X = {");
for (const leg of LEG_IDS) {
  const [lo, hi] = stanceWindows[leg];
  console.log(`  ${leg}: [${lo.toFixed(3)}, ${hi.toFixed(3)}],`);
}
console.log("};");

console.log("");
console.log(`export const DEFAULT_FOOT_REACH_X_LIFTED = {  // swing-peak z = ${(STANCE_Z + liftM).toFixed(3)} m`);
for (const leg of LEG_IDS) {
  const [lo, hi] = liftedWindows[leg];
  console.log(`  ${leg}: [${lo.toFixed(3)}, ${hi.toFixed(3)}],`);
}
console.log("};");

// ─── Helpers ──────────────────────────────────────────────────────────────

function probeAtZ(z, stepM, safetyMm) {
  // For each leg in turn, sweep foot.x from far behind to far in front and
  // record the boundaries where IK succeeds. The other three legs are pinned
  // to their gait-stance feet so fourLegsInverseKinematics doesn't reject
  // the whole solve over an unrelated leg.
  const out = {};
  const stanceFeet = LEG_IDS.map((id) => GAIT_STANCE[id]);
  for (let probedIdx = 0; probedIdx < LEG_IDS.length; probedIdx++) {
    const probedLeg = LEG_IDS[probedIdx];
    const stance = GAIT_STANCE[probedLeg];
    const baseX = stance[0];
    // Sweep ±100 mm around the leg's neutral x — well beyond the geometric
    // workspace so the boundaries definitely fall inside the sweep.
    const xMin = baseX - 0.10;
    const xMax = baseX + 0.10;
    let lo = null;
    let hi = null;
    for (let x = xMin; x <= xMax + 1e-9; x += stepM) {
      const candidate = stanceFeet.map((f, i) =>
        i === probedIdx ? [x, stance[1], z] : f.slice(),
      );
      const res = fourLegsInverseKinematics(candidate);
      if (res.ok) {
        if (lo === null) lo = x;
        hi = x;
      }
    }
    if (lo === null || hi === null) {
      console.error(`  WARN: no reachable x for ${probedLeg} at z=${z.toFixed(3)}`);
      out[probedLeg] = [baseX, baseX];
      continue;
    }
    // Pull bounds in by safety margin so IK never sits on the cusp.
    const safetyM = safetyMm / 1000;
    out[probedLeg] = [lo + safetyM, hi - safetyM];
  }
  return out;
}

function parseArgs(argv) {
  const out = {};
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = argv[i + 1];
    if (a === "--lift-mm") { out.liftMm = Number(next); i++; }
    else if (a === "--step-mm") { out.stepMm = Number(next); i++; }
    else if (a === "--safety-mm") { out.safetyMm = Number(next); i++; }
    else if (a === "--z-mm") { out.zMm = Number(next); i++; }  // override stance z
    else if (a === "--y-mm") { out.yMm = Number(next); i++; }  // override stance |y|
  }
  return out;
}
