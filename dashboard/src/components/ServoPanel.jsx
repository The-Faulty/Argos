// Direct servo-horn sliders for the active leg.
//
// Commands go out in servo-space and use the calibrated measured horn window.
// The coupled femur/tibia envelope is enforced in joint/IK paths; servo mode
// stays a direct calibration tool.

import { useMemo } from "react";
import {
  JOINT_ROWS,
  LEG_LABELS,
  SERVO_CENTER_DEG,
  JOINT_NAMES,
  LEG_IDS,
} from "../../shared/robot-config.js";
import {
  applyServoCal,
  clampServoDeg,
  inverseServoCal,
  servoCommandLimitsDeg,
} from "../../shared/servo_cal.js";
import { leg_angle_map } from "./LegDetail.jsx";

const RAD2DEG = 180.0 / Math.PI;

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
        const [lo, hi] = servoCommandLimitsDeg(jointName);
        const deg = cur[jointName] ?? SERVO_CENTER_DEG;
        const sliderDeg = clampNum(deg, lo, hi);
        const jointDeg = safeJointDeg(jointName, deg);
        const minJointDeg = safeJointDeg(jointName, lo);
        const maxJointDeg = safeJointDeg(jointName, hi);
        return (
          <label key={row} className="slider">
            <span>{row} joint ({minJointDeg.toFixed(0)}° … {maxJointDeg.toFixed(0)}°)</span>
            <input
              type="range"
              min={lo}
              max={hi}
              step="0.5"
              value={sliderDeg}
              disabled={!enabled}
              onChange={(e) => onChange(row, Number(e.target.value))}
            />
            <span className="slider__value">{jointDeg.toFixed(1)}° joint / {deg.toFixed(1)}° servo</span>
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

function safeJointDeg(jointName, servoDeg) {
  const rad = inverseServoCal(jointName, servoDeg);
  return (Number.isFinite(rad) ? rad : 0) * RAD2DEG;
}

function clampNum(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}
