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
// Bell-crank envelope clamp happens on the Pi (dashboard_bridge_node calls
// `clamp_joint_matrix` before publishing). Visual "near limit" hinting
// still happens here so the user sees red when approaching the boundary.

import { useMemo } from "react";
import {
  DEFAULT_JOINT_LIMITS_RAD,
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  LEG_LABELS,
} from "../../shared/robot-config.js";
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
        const limits = DEFAULT_JOINT_LIMITS_RAD[row];
        const rad = full[jointName] ?? 0;
        const deg = rad * RAD2DEG;
        const minDeg = limits[0] * RAD2DEG;
        const maxDeg = limits[1] * RAD2DEG;
        const nearLimit = deg - minDeg < NEAR_LIMIT_DEG || maxDeg - deg < NEAR_LIMIT_DEG;
        return (
          <label key={row} className={`slider${nearLimit ? " is-near-limit" : ""}`}>
            <span>{row} ({minDeg.toFixed(0)}° … {maxDeg.toFixed(0)}°)</span>
            <input
              type="range"
              min={minDeg}
              max={maxDeg}
              step="0.5"
              value={deg}
              onChange={(e) => onChange(row, Number(e.target.value) * DEG2RAD)}
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
