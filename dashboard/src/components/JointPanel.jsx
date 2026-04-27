// Always-on joint-angle sliders for the active leg (coxa / femur / tibia).
//
// Rendered directly under LegDetail so the three servos for the focal leg
// are always reachable at a glance. Emits a full 12-element array — the
// three active-leg values get updated on drag, everything else is pulled
// from the latest /joint_states so untouched legs hold their pose. The
// App shell auto-switches to `direct_joint_angles` on first interaction
// (see `sendJointAnglesForActiveLeg` in App.jsx) so the bridge actually
// consumes the command without requiring a manual mode flip.
//
// Bell-crank envelope clamp happens on the Pi, and the sliders mirror that
// coupled envelope live: tibia bounds follow the current femur angle, while
// femur bounds follow the current tibia angle.

import { useMemo } from "react";
import {
  DEFAULT_JOINT_LIMITS_RAD,
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  LEG_LABELS,
} from "../../shared/robot-config.js";
import {
  coupled_bot_limits_joint_rad,
  coupled_top_limits_joint_rad,
} from "../../shared/kinematics.js";
import { leg_angle_map } from "./LegDetail.jsx";

const RAD2DEG = 180.0 / Math.PI;
const DEG2RAD = Math.PI / 180.0;
const NEAR_LIMIT_DEG = 5.0;

export default function JointPanel({
  activeLeg,
  jointState,
  onSendAngles,
  previewAngles,
  onPreviewAngles,
}) {
  // We work in the full 12-vector so we can send one command covering the
  // untouched legs at their current pose. Active-leg rows prefer the shared
  // optimistic preview (set by either this panel or LegDetail's foot drag)
  // so slider positions track drag in real time.
  const full = useMemo(
    () => readFull(jointState, previewAngles, activeLeg),
    [jointState, previewAngles, activeLeg],
  );

  function onChange(row, valueRad) {
    // Compose the updated 3-element active-leg vector (preserve untouched
    // rows at their current effective value). Push it to the shared
    // preview first so LegDetail's SVG + stats move in lock-step, then
    // fire the 12-vector command so the Pi catches up on the next tick.
    const activeAngles = JOINT_ROWS.map((r) =>
      r === row ? valueRad : (full[`${activeLeg}_${r}_joint`] ?? 0),
    );
    onPreviewAngles?.({ leg: activeLeg, angles: activeAngles });

    const next = { ...full };
    JOINT_ROWS.forEach((r, i) => {
      next[`${activeLeg}_${r}_joint`] = activeAngles[i];
    });
    const ordered = JOINT_NAMES.map((n) => next[n] ?? 0);
    onSendAngles?.(ordered);
  }

  return (
    <section className="joint-panel">
      <header>
        <h3>Joint sliders — {LEG_LABELS[activeLeg]}</h3>
        <span className="muted">coxa / femur / tibia</span>
      </header>

      {JOINT_ROWS.map((row) => {
        const jointName = `${activeLeg}_${row}_joint`;
        const limits = dynamicLimitsFor(row, activeLeg, full);
        const absLimits = DEFAULT_JOINT_LIMITS_RAD[row];
        const rad = full[jointName] ?? 0;
        const deg = rad * RAD2DEG;
        const minDeg = limits[0] * RAD2DEG;
        const maxDeg = limits[1] * RAD2DEG;
        const absMinDeg = absLimits[0] * RAD2DEG;
        const absMaxDeg = absLimits[1] * RAD2DEG;
        const sliderDeg = clampNum(deg, minDeg, maxDeg);
        const nearLimit = deg - minDeg < NEAR_LIMIT_DEG || maxDeg - deg < NEAR_LIMIT_DEG;
        // True when the coupled envelope is currently tighter than the
        // hardware-absolute envelope — i.e., the other joint's position is
        // restricting this joint's reach. Shown explicitly in the label so
        // the operator can see the coupling is actively constraining them.
        const coupledTighter =
          Math.abs(minDeg - absMinDeg) > 0.05 || Math.abs(maxDeg - absMaxDeg) > 0.05;
        // Place the coupled-allowed band as a colored region on the track.
        // The slider's min/max are kept at the coupled values (so the thumb
        // can't visit the disallowed zone) but the absolute envelope is
        // overlaid as a faint outer band so the operator can see HOW MUCH
        // of the joint's full range is currently locked out.
        const absSpan = Math.max(1e-6, absMaxDeg - absMinDeg);
        const coupledLoPct = Math.max(0, (minDeg - absMinDeg) / absSpan) * 100;
        const coupledHiPct = Math.min(100, (maxDeg - absMinDeg) / absSpan) * 100;
        const trackBg = `linear-gradient(to right,
          rgba(220,38,38,0.18) 0%,
          rgba(220,38,38,0.18) ${coupledLoPct.toFixed(2)}%,
          rgba(34,197,94,0.28) ${coupledLoPct.toFixed(2)}%,
          rgba(34,197,94,0.28) ${coupledHiPct.toFixed(2)}%,
          rgba(220,38,38,0.18) ${coupledHiPct.toFixed(2)}%,
          rgba(220,38,38,0.18) 100%)`;
        return (
          <label key={row} className={`slider${nearLimit ? " is-near-limit" : ""}`}>
            <span>
              {row} (
              <strong>{minDeg.toFixed(1)}° … {maxDeg.toFixed(1)}°</strong>
              {coupledTighter && (
                <span className="muted">
                  {" "}/ abs {absMinDeg.toFixed(0)}° … {absMaxDeg.toFixed(0)}°
                </span>
              )}
              )
            </span>
            <input
              type="range"
              min={absMinDeg}
              max={absMaxDeg}
              step="0.1"
              value={sliderDeg}
              onChange={(e) => {
                // Clamp to coupled limits at the input boundary so a value
                // that violates the coupled envelope can't ever be sent.
                const next = clampNum(Number(e.target.value), minDeg, maxDeg);
                onChange(row, next * DEG2RAD);
              }}
              style={{ background: trackBg }}
            />
            <span className="slider__value">{deg.toFixed(1)}°</span>
          </label>
        );
      })}
    </section>
  );
}

function readFull(jointState, previewAngles, activeLeg) {
  const out = {};
  const bag = leg_angle_map(jointState);
  for (const leg of LEG_IDS) {
    for (const row of JOINT_ROWS) {
      const name = `${leg}_${row}_joint`;
      out[name] = Number.isFinite(bag[name]) ? bag[name] : 0;
    }
  }
  // Override active-leg rows from the shared preview if present. Keeps the
  // sliders in sync with LegDetail drags without waiting for /joint_states.
  if (previewAngles && previewAngles.leg === activeLeg && Array.isArray(previewAngles.angles)) {
    JOINT_ROWS.forEach((row, i) => {
      const v = previewAngles.angles[i];
      if (Number.isFinite(v)) out[`${activeLeg}_${row}_joint`] = v;
    });
  }
  return out;
}

function dynamicLimitsFor(row, activeLeg, full) {
  const global = DEFAULT_JOINT_LIMITS_RAD[row];
  if (row === "tibia") {
    const femur = full[`${activeLeg}_femur_joint`] ?? 0;
    return intersectLimits(global, coupled_bot_limits_joint_rad(femur));
  }
  if (row === "femur") {
    const tibia = full[`${activeLeg}_tibia_joint`] ?? 0;
    return intersectLimits(global, coupled_top_limits_joint_rad(tibia));
  }
  return global;
}

function intersectLimits(a, b) {
  const lo = Math.max(a[0], b[0]);
  const hi = Math.min(a[1], b[1]);
  if (lo <= hi) return [lo, hi];
  const mid = 0.5 * (lo + hi);
  return [mid, mid];
}

function clampNum(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}
