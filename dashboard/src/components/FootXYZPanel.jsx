// Direct foot XYZ panel for the active leg.
//
// Two-axis pad for X/Y (body frame, meters) + a vertical slider for Z. Sends
// a foot_targets command with the 4×3 matrix — untouched legs reuse the
// default stance so the robot doesn't jump when a single leg drags.
//
// Kinematic reachability isn't checked here; the bridge node applies
// `clamp_joint_matrix` which will bounce unreachable commands back. Visual
// hint: if the user drags outside the typical reachable box we shade the
// target red.

import { useRef, useState } from "react";
import {
  DEFAULT_STANCE,
  DEFAULT_Z_REF,
  LEG_IDS,
  LEG_LABELS,
} from "../../shared/robot-config.js";

const PAD_SIZE = 240;
const XY_RANGE_M = 0.18; // pad covers ±18 cm in x, ±18 cm in y
const Z_MIN = -0.23;
const Z_MAX = -0.14;

export default function FootXYZPanel({ activeLeg, onSendFootTargets, enabled }) {
  const padRef = useRef(null);
  const [target, setTarget] = useState(() => ({ ...DEFAULT_STANCE[activeLeg] && defaultForLeg(activeLeg) }));

  // Re-seed target when the active leg changes so we don't "remember" the
  // last leg's XY on a different hip.
  const lastLegRef = useRef(activeLeg);
  if (lastLegRef.current !== activeLeg) {
    lastLegRef.current = activeLeg;
    const d = defaultForLeg(activeLeg);
    if (Math.abs(target.x - d.x) > 1e-6 || Math.abs(target.y - d.y) > 1e-6) {
      // Async via setTimeout to avoid setState-in-render.
      setTimeout(() => setTarget(d), 0);
    }
  }

  function dispatch(next) {
    setTarget(next);
    const matrix = LEG_IDS.map((leg) => {
      if (leg === activeLeg) return [next.x, next.y, next.z];
      return [...DEFAULT_STANCE[leg]];
    });
    onSendFootTargets?.(matrix);
  }

  function onPadMove(event) {
    if (!enabled) return;
    const rect = padRef.current.getBoundingClientRect();
    const px = (event.clientX - rect.left) / rect.width;
    const py = (event.clientY - rect.top) / rect.height;
    const x = (py - 0.5) * -2 * XY_RANGE_M + DEFAULT_STANCE[activeLeg][0];
    const y = (px - 0.5) *  2 * XY_RANGE_M + DEFAULT_STANCE[activeLeg][1];
    dispatch({ ...target, x, y });
  }

  return (
    <section className="foot-xyz">
      <header>
        <h3>Foot XYZ — {LEG_LABELS[activeLeg]}</h3>
        <span className="muted">body frame, meters</span>
      </header>

      <div className="foot-xyz__grid">
        <div
          ref={padRef}
          className="foot-xyz__pad"
          style={{ width: PAD_SIZE, height: PAD_SIZE, touchAction: "none" }}
          onPointerDown={(e) => {
            if (!enabled) return;
            padRef.current.setPointerCapture(e.pointerId);
            onPadMove(e);
          }}
          onPointerMove={(e) => e.buttons === 1 && onPadMove(e)}
        >
          <PadDot target={target} leg={activeLeg} />
          <Crosshair />
        </div>

        <div className="foot-xyz__z">
          <label>
            <span>z (height)</span>
            <input
              type="range"
              min={Z_MIN} max={Z_MAX} step={0.002}
              value={target.z}
              disabled={!enabled}
              onChange={(e) => dispatch({ ...target, z: Number(e.target.value) })}
              style={{ writingMode: "bt-lr", appearance: "slider-vertical", height: PAD_SIZE }}
            />
            <span className="slider__value">{target.z.toFixed(3)} m</span>
          </label>
        </div>
      </div>

      <dl className="foot-xyz__readout">
        <div><dt>x</dt><dd>{target.x.toFixed(3)} m</dd></div>
        <div><dt>y</dt><dd>{target.y.toFixed(3)} m</dd></div>
        <div><dt>z</dt><dd>{target.z.toFixed(3)} m</dd></div>
      </dl>
    </section>
  );
}

function PadDot({ target, leg }) {
  const [dx, dy] = [
    target.y - DEFAULT_STANCE[leg][1],
    -(target.x - DEFAULT_STANCE[leg][0]),
  ];
  const cx = PAD_SIZE / 2 + (dx / XY_RANGE_M) * (PAD_SIZE / 2);
  const cy = PAD_SIZE / 2 + (dy / XY_RANGE_M) * (PAD_SIZE / 2);
  return (
    <svg width={PAD_SIZE} height={PAD_SIZE} viewBox={`0 0 ${PAD_SIZE} ${PAD_SIZE}`}
         style={{ pointerEvents: "none", position: "absolute", inset: 0 }}>
      <circle cx={cx} cy={cy} r={10} fill="#2563eb" />
    </svg>
  );
}

function Crosshair() {
  const mid = PAD_SIZE / 2;
  return (
    <svg width={PAD_SIZE} height={PAD_SIZE} viewBox={`0 0 ${PAD_SIZE} ${PAD_SIZE}`}
         style={{ pointerEvents: "none", position: "absolute", inset: 0 }}>
      <line x1={0} x2={PAD_SIZE} y1={mid} y2={mid} stroke="rgba(0,0,0,0.12)" />
      <line x1={mid} x2={mid} y1={0} y2={PAD_SIZE} stroke="rgba(0,0,0,0.12)" />
    </svg>
  );
}

function defaultForLeg(leg) {
  const [x, y, z = DEFAULT_Z_REF] = DEFAULT_STANCE[leg];
  return { x, y, z };
}
