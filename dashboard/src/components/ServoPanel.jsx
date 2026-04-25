// Direct servo-horn sliders (0..180°) for the active leg.
//
// Operates in servo-space (horn degrees), not joint-space radians. Servo
// cal direction is applied server-side when the bridge re-maps back to
// joint angles. We DO clamp to the bench-measured SERVO_LIMITS_DEG per
// row here so a user can't drag into a hard stop.
//
// Coupled bell-crank clamps live on the Pi — dragging the bottom slider
// toward an invalid angle for the current top-slider position will snap
// to the envelope when the bridge publishes.

import { useMemo } from "react";
import {
  JOINT_ROWS,
  LEG_LABELS,
  SERVO_CENTER_DEG,
  SERVO_LIMITS_DEG,
  JOINT_NAMES,
  LEG_IDS,
} from "../../shared/robot-config.js";
import { applyServoCal, clampServoDeg } from "../../shared/servo_cal.js";
import { leg_angle_map } from "./LegDetail.jsx";

const ROW_INDEX = { coxa: 0, femur: 1, tibia: 2 };

export default function ServoPanel({ activeLeg, jointState, onSendServoAngles, enabled }) {
  const cur = useMemo(() => readFullServoDeg(jointState), [jointState]);

  function onChange(row, deg) {
    const jointName = `${activeLeg}_${row}_joint`;
    const clamped = clampServoDeg(jointName, deg);
    const next = { ...cur, [jointName]: clamped };
    const ordered = JOINT_NAMES.map((n) => next[n] ?? SERVO_CENTER_DEG);
    onSendServoAngles?.(ordered);
  }

  return (
    <section className="servo-panel">
      <header>
        <h3>Servo sliders — {LEG_LABELS[activeLeg]}</h3>
        <span className="muted">horn ° (0..180)</span>
      </header>

      {JOINT_ROWS.map((row) => {
        const jointName = `${activeLeg}_${row}_joint`;
        const [lo, hi] = SERVO_LIMITS_DEG[ROW_INDEX[row]];
        const deg = cur[jointName] ?? SERVO_CENTER_DEG;
        return (
          <label key={row} className="slider">
            <span>{row} ({lo}° … {hi}°)</span>
            <input
              type="range"
              min={lo}
              max={hi}
              step="0.5"
              value={deg}
              disabled={!enabled}
              onChange={(e) => onChange(row, Number(e.target.value))}
            />
            <span className="slider__value">{deg.toFixed(1)}°</span>
          </label>
        );
      })}
    </section>
  );
}

function readFullServoDeg(jointState) {
  const out = {};
  const bag = leg_angle_map(jointState);
  for (const leg of LEG_IDS) {
    for (const row of JOINT_ROWS) {
      const name = `${leg}_${row}_joint`;
      const rad = Number.isFinite(bag[name]) ? bag[name] : 0;
      const deg = applyServoCal(name, rad);
      out[name] = Number.isFinite(deg) ? deg : SERVO_CENTER_DEG;
    }
  }
  return out;
}
