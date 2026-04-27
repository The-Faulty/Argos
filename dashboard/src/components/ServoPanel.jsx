// Direct servo-horn sliders for the active leg.
//
// Commands still go out in servo-space, but the labels and slider window are
// shown in joint-space and mirror the coupled femur/tibia envelope live.

import { useMemo } from "react";
import {
  DEFAULT_JOINT_LIMITS_RAD,
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
import {
  coupled_bot_limits_joint_rad,
  coupled_top_limits_joint_rad,
} from "../../shared/kinematics.js";
import { leg_angle_map } from "./LegDetail.jsx";

const RAD2DEG = 180.0 / Math.PI;

export default function ServoPanel({ activeLeg, jointState, onSendServoAngles, enabled }) {
  const cur = useMemo(() => readFullServoDeg(jointState), [jointState]);

  function onChange(row, deg) {
    const jointName = `${activeLeg}_${row}_joint`;
    const { servoDeg } = dynamicServoLimitsFor(row, jointName, activeLeg, cur);
    const clamped = clampNum(clampServoDeg(jointName, deg), servoDeg[0], servoDeg[1]);
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
        const { jointRad, servoDeg } = dynamicServoLimitsFor(row, jointName, activeLeg, cur);
        const [lo, hi] = servoDeg;
        const deg = cur[jointName] ?? SERVO_CENTER_DEG;
        const sliderDeg = clampNum(deg, lo, hi);
        const jointDeg = safeJointDeg(jointName, deg);
        const minJointDeg = jointRad[0] * RAD2DEG;
        const maxJointDeg = jointRad[1] * RAD2DEG;
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

function dynamicServoLimitsFor(row, jointName, activeLeg, servoDegByJoint) {
  const jointRad = dynamicJointLimitsFor(row, activeLeg, servoDegByJoint);
  const servoDeg = servoRangeForJointLimits(jointName, jointRad);
  return { jointRad, servoDeg };
}

function dynamicJointLimitsFor(row, activeLeg, servoDegByJoint) {
  const global = DEFAULT_JOINT_LIMITS_RAD[row];
  if (row === "tibia") {
    const femur = currentJointRad(`${activeLeg}_femur_joint`, servoDegByJoint);
    return intersectLimits(global, coupled_bot_limits_joint_rad(femur));
  }
  if (row === "femur") {
    const tibia = currentJointRad(`${activeLeg}_tibia_joint`, servoDegByJoint);
    return intersectLimits(global, coupled_top_limits_joint_rad(tibia));
  }
  return global;
}

function servoRangeForJointLimits(jointName, jointRad) {
  const a = applyServoCal(jointName, jointRad[0]);
  const b = applyServoCal(jointName, jointRad[1]);
  const fromJoint = sortPair([
    Number.isFinite(a) ? a : SERVO_CENTER_DEG,
    Number.isFinite(b) ? b : SERVO_CENTER_DEG,
  ]);
  return intersectLimits(fromJoint, servoCommandLimitsDeg(jointName));
}

function currentJointRad(jointName, servoDegByJoint) {
  const rad = inverseServoCal(jointName, servoDegByJoint[jointName] ?? SERVO_CENTER_DEG);
  return Number.isFinite(rad) ? rad : 0;
}

function safeJointDeg(jointName, servoDeg) {
  const rad = inverseServoCal(jointName, servoDeg);
  return (Number.isFinite(rad) ? rad : 0) * RAD2DEG;
}

function intersectLimits(a, b) {
  const lo = Math.max(a[0], b[0]);
  const hi = Math.min(a[1], b[1]);
  if (lo <= hi) return [lo, hi];
  const mid = 0.5 * (lo + hi);
  return [mid, mid];
}

function sortPair(pair) {
  return pair[0] <= pair[1] ? pair : [pair[1], pair[0]];
}

function clampNum(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}
