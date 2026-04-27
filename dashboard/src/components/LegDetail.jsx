// Big SVG leg view — one leg, rendered with andy-servo-control's sagittal
// bell-crank drawing. The real IK still drives commands, but this panel uses
// the legacy millimeter-space linkage so the large leg view matches the
// reference full-body pose dashboard visually.

import { useMemo, useRef } from "react";
import {
  clamp_joint_matrix,
} from "../../shared/argos_kinematics.js";
import {
  LEGACY_LEG_GEOMETRY,
  buildLegPoseFromJointAngles,
  fromCanvasPoint,
  toCanvasPoint,
} from "../../shared/kinematics.js";
import {
  DEFAULT_JOINT_LIMITS_RAD,
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_DRAWING,
  LEG_IDS,
  LEG_LABELS,
  SERVO_CENTER_DEG,
} from "../../shared/robot-config.js";
import { applyServoCal } from "../../shared/servo_cal.js";

const RAD2DEG = 180.0 / Math.PI;
const DEG2RAD = Math.PI / 180.0;
const LEGACY_CALF_FROM_TIBIA_OFFSET_DEG = -150.0;

// Seven segments, drawn in the same colors/widths the legacy view used so
// the visual identity is preserved. Keys reference the geometry names
// returned by kinematics_legacy.solveGeometry.
const SEGMENTS = [
  { key: "upper",     from: "hip",          to: "knee",         stroke: "#141414", width: 5 },
  { key: "lower",     from: "knee",         to: "foot",         stroke: "#1d8cff", width: 5 },
  { key: "bell-a",    from: "hip",          to: "bellArmA",     stroke: "#21a264", width: 4 },
  { key: "bell-b",    from: "hip",          to: "bellArmB",     stroke: "#21a264", width: 4 },
  { key: "horn",      from: "servoPivot",   to: "servoHornEnd", stroke: "#ff5232", width: 4 },
  { key: "linkShort", from: "servoHornEnd", to: "bellArmA",     stroke: "#ef8a17", width: 4 },
  { key: "linkLong",  from: "bellArmB",     to: "calfAttach",   stroke: "#6f4cff", width: 4 },
];

const DOT_KEYS = ["hip", "knee", "servoPivot", "servoHornEnd", "bellArmA", "bellArmB", "calfAttach"];

export default function LegDetail({
  activeLeg,
  jointState,
  onFootDrag,
  mode,
  previewAngles,
  onPreviewAngles,
}) {
  const svgRef = useRef(null);

  const angles = useMemo(() => readLegAnglesRad(jointState, activeLeg), [jointState, activeLeg]);

  // Effective angles: prefer the shared optimistic preview when it matches
  // the active leg, so the SVG + stats reflect drag/slider input even when
  // /joint_states isn't flowing back. Falls through to raw jointState when
  // the preview is null or points at a different leg.
  const effectiveAngles = useMemo(() => {
    if (previewAngles && previewAngles.leg === activeLeg && Array.isArray(previewAngles.angles)) {
      return previewAngles.angles;
    }
    return angles;
  }, [previewAngles, activeLeg, angles]);

  const pose = useMemo(() => {
    try {
      const result = buildLegPoseFromJointAngles(toLegacyJointPose(effectiveAngles));
      const geometry = result.geometry;
      return {
        geometry,
        foot: {
          x: geometry.foot.x - LEGACY_LEG_GEOMETRY.footOriginOffset.x,
          y: geometry.foot.y - LEGACY_LEG_GEOMETRY.footOriginOffset.y,
        },
        reachable: result.reachable,
      };
    } catch {
      return null;
    }
  }, [effectiveAngles, activeLeg]);

  // Pre-map every legacy millimeter-space geometry point once per render.
  const points = useMemo(() => {
    if (!pose?.geometry) return null;
    const p = (pt) => toCanvasPoint(pt, LEG_DRAWING);
    return {
      hip:          p(pose.geometry.hip),
      knee:         p(pose.geometry.knee),
      foot:         p(pose.geometry.foot),
      servoPivot:   p(pose.geometry.servoPivot),
      servoHornEnd: p(pose.geometry.servoHornEnd),
      bellArmA:     p(pose.geometry.bellArmA),
      bellArmB:     p(pose.geometry.bellArmB),
      calfAttach:   p(pose.geometry.calfAttach),
    };
  }, [pose]);

  const isDragging = useRef(false);
  // Andy-servo lets you drag the foot any time a handler is wired. If the
  // app is not currently in direct_foot_xyz, the drag handler upstream
  // auto-switches the mode so the command is actually consumed by the
  // bridge. Don't gate on `mode` here — gating broke foot drag for users
  // who expected it to "just work" like on andy-servo.
  const dragModeOk = typeof onFootDrag === "function";

  // Right-side legs are drawn as if viewed from the right side of the robot
  // so FR/RR don't look upside-down to the operator. Visually this is a
  // horizontal flip about the vertical axis of the stage. Pointer coords
  // have to be un-flipped before IK so the solver stays in the leg's own
  // kinematic frame (+x forward).
  const mirrorX = activeLeg === "FR" || activeLeg === "RR";
  const mirrorTransform = mirrorX
    ? `translate(${LEG_DRAWING.stageWidth} 0) scale(-1 1)`
    : undefined;

  function onPointerDown(event) {
    if (!dragModeOk || !svgRef.current || !pose) return;
    svgRef.current.setPointerCapture(event.pointerId);
    isDragging.current = true;
    dispatchFootAt(event);
  }

  function onPointerMove(event) {
    if (!isDragging.current) return;
    dispatchFootAt(event);
  }

  function onPointerUp(event) {
    if (!isDragging.current) return;
    isDragging.current = false;
    try {
      svgRef.current?.releasePointerCapture(event.pointerId);
    } catch {
      /* already released */
    }
  }

  function dispatchFootAt(event) {
    const svg = svgRef.current;
    if (!svg) return;
    const rect = svg.getBoundingClientRect();
    let px = ((event.clientX - rect.left) / rect.width) * LEG_DRAWING.stageWidth;
    const py = ((event.clientY - rect.top) / rect.height) * LEG_DRAWING.stageHeight;
    // Un-flip the pointer for mirrored legs so the IK target lands in the
    // leg's un-mirrored kinematic frame. Without this, dragging visually
    // right on a mirrored FR/RR would send the foot to the wrong X value.
    if (mirrorX) px = LEG_DRAWING.stageWidth - px;
    const absMm = fromCanvasPoint({ x: px, y: py }, LEG_DRAWING);

    const solved = solveDragAngles(absMm, effectiveAngles);
    if (solved) {
      const nextAngles = clampLegAngles(solved);
      onPreviewAngles?.({ leg: activeLeg, angles: nextAngles });
      onFootDrag?.({ x: absMm.x, z: absMm.y, leg: activeLeg, angles: nextAngles });
    }
  }

  return (
    <div className="leg-detail">
      <header className="leg-detail__header">
        <h2>{LEG_LABELS[activeLeg]}</h2>
        <LegSelector active={activeLeg} />
      </header>

      <svg
        ref={svgRef}
        className="leg-detail__svg"
        viewBox={`0 0 ${LEG_DRAWING.stageWidth} ${LEG_DRAWING.stageHeight}`}
        role="img"
        aria-label={`Sagittal view of leg ${activeLeg}`}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        style={{ touchAction: "none", cursor: dragModeOk ? "grab" : "default" }}
      >
        <StageGrid />
        {points ? (
          <>
            {/* Segments + dots + foot circle go inside the mirror group so
                they flip visually for FR/RR. The foot chip stays outside so
                its text reads normally; its anchor point is pre-mirrored. */}
            <g transform={mirrorTransform}>
              {SEGMENTS.map((seg) => (
                <path
                  key={seg.key}
                  d={canvasSegmentPath(points[seg.from], points[seg.to])}
                  stroke={seg.stroke}
                  strokeWidth={seg.width}
                  strokeLinecap="round"
                  fill="none"
                />
              ))}
              {DOT_KEYS.map((key) => (
                <circle
                  key={key}
                  cx={points[key].x}
                  cy={points[key].y}
                  r={5}
                  fill="#121212"
                />
              ))}
              <FootDot point={points.foot} interactive={dragModeOk} />
            </g>
            <FootChip
              pose={pose}
              point={{
                x: mirrorX ? LEG_DRAWING.stageWidth - points.foot.x : points.foot.x,
                y: points.foot.y,
              }}
            />
          </>
        ) : (
          <text x={LEG_DRAWING.stageWidth / 2} y={LEG_DRAWING.stageHeight / 2}
                textAnchor="middle" className="leg-detail__err">
            Unreachable pose
          </text>
        )}
      </svg>

      <StatsGrid activeLeg={activeLeg} angles={effectiveAngles} reachable={pose?.reachable ?? false} />
    </div>
  );
}

// ─── Subcomponents ───────────────────────────────────────────────────────

function StageGrid() {
  return (
    <g>
      <line
        x1={0} x2={LEG_DRAWING.stageWidth} y1={LEG_DRAWING.offset.y} y2={LEG_DRAWING.offset.y}
        stroke="rgba(0,0,0,0.08)" strokeDasharray="4 4"
      />
      <line
        x1={LEG_DRAWING.offset.x} x2={LEG_DRAWING.offset.x} y1={0}
        y2={LEG_DRAWING.stageHeight}
        stroke="rgba(0,0,0,0.05)" strokeDasharray="4 4"
      />
    </g>
  );
}

function FootDot({ point, interactive }) {
  // Andy-servo draws the foot as a single blue circle (r=7 static, r=11
  // when draggable). No halo under-circle — the larger radius in interactive
  // mode is what visually calls out "you can grab this".
  return (
    <circle
      cx={point.x} cy={point.y}
      r={interactive ? 11 : 7}
      fill="#1d8cff"
    />
  );
}

function FootChip({ pose, point }) {
  // `pose.foot` is in sagittal-frame millimeters (andy-servo's output). Label
  // x / z because the dashboard thinks of the vertical axis as z even though
  // andy-servo calls it y — 2-D sagittal's y IS the robot's z.
  const { x = 0, y = 0 } = pose.foot ?? {};
  const chipW = 110;
  const chipH = 30;
  const chipX = Math.min(LEG_DRAWING.stageWidth - chipW - 4, point.x + 12);
  const chipY = Math.max(4, point.y - chipH - 8);
  return (
    <g transform={`translate(${chipX}, ${chipY})`}>
      <rect width={chipW} height={chipH} rx={6} fill="rgba(17,17,17,0.85)" />
      <text x={8} y={20} fill="#fff" fontSize={12} fontFamily="ui-monospace, monospace">
        x {x.toFixed(1)}  z {y.toFixed(1)}
      </text>
    </g>
  );
}

function LegSelector({ active }) {
  // Emits a CustomEvent rather than consuming a context — lets the parent
  // App.jsx keep ownership of the selection while this component stays
  // self-contained.
  return (
    <div className="leg-selector" role="tablist" aria-label="Active leg">
      {LEG_IDS.map((leg) => (
        <button
          type="button"
          key={leg}
          role="tab"
          aria-selected={leg === active}
          className={`leg-selector__chip${leg === active ? " is-active" : ""}`}
          onClick={() =>
            window.dispatchEvent(new CustomEvent("argos:set-active-leg", { detail: leg }))
          }
        >
          {leg}
        </button>
      ))}
    </div>
  );
}

function StatsGrid({ activeLeg, angles, reachable }) {
  // 3 rows: coxa / femur / tibia × joint-rad / joint-deg / servo-deg /
  // Δ-from-center. "Servo" column uses the cal table's direction + offset
  // so what you see matches what the firmware actually sends to the
  // PCA9685 for each joint.
  const rows = JOINT_ROWS.map((row, i) => {
    const rad = angles[i];
    const jointName = `${activeLeg}_${row}_joint`;
    const deg = Number.isFinite(rad) ? rad * RAD2DEG : NaN;
    const servoDeg = Number.isFinite(rad) ? applyServoCal(jointName, rad) : NaN;
    return {
      row,
      rad,
      deg,
      servoDeg,
      servoFromCenter: Number.isFinite(servoDeg) ? servoDeg - SERVO_CENTER_DEG : NaN,
    };
  });

  return (
    <div className={`leg-detail__stats${reachable ? "" : " is-unreachable"}`}>
      <table>
        <thead>
          <tr>
            <th>joint</th>
            <th>rad</th>
            <th>deg</th>
            <th>servo °</th>
            <th>Δ from 90°</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => (
            <tr key={r.row}>
              <th scope="row">{r.row}</th>
              <td>{fmt(r.rad, 3)}</td>
              <td>{fmt(r.deg, 1)}</td>
              <td>{fmt(r.servoDeg, 1)}</td>
              <td>{fmt(r.servoFromCenter, 1)}</td>
            </tr>
          ))}
        </tbody>
      </table>
      {!reachable && <p className="leg-detail__warn">Target outside bell-crank envelope</p>}
    </div>
  );
}

function fmt(n, digits) {
  if (!Number.isFinite(n)) return "—";
  return n.toFixed(digits);
}

function canvasSegmentPath(a, b) {
  return `M ${a.x} ${a.y} L ${b.x} ${b.y}`;
}

function toLegacyJointPose(angles) {
  const femurDeg = (Number(angles?.[1]) || 0) * RAD2DEG;
  const tibiaDeg = (Number(angles?.[2]) || 0) * RAD2DEG;
  return {
    thigh: femurDeg,
    calf: tibiaDeg + LEGACY_CALF_FROM_TIBIA_OFFSET_DEG,
  };
}

function solveDragAngles(targetMm, currentAngles) {
  const coxa = Number(currentAngles?.[0]) || 0;
  const currentFemurDeg = (Number(currentAngles?.[1]) || 0) * RAD2DEG;
  const currentTibiaDeg = (Number(currentAngles?.[2]) || 0) * RAD2DEG;
  const { thighLength: upper, calfLength: lower } = LEGACY_LEG_GEOMETRY;
  const dist = Math.hypot(targetMm.x, targetMm.y);
  if (!Number.isFinite(dist) || dist < 1e-6) return null;

  const maxReach = upper + lower - 1e-6;
  const minReach = Math.abs(upper - lower) + 1e-6;
  const reach = Math.max(minReach, Math.min(maxReach, dist));
  const base = Math.atan2(targetMm.y, targetMm.x);
  const elbow = Math.acos(
    Math.max(-1, Math.min(1, (upper * upper + reach * reach - lower * lower) / (2 * upper * reach))),
  );
  let best = null;

  const consider = (femurRad) => {
    const knee = {
      x: upper * Math.cos(femurRad),
      y: upper * Math.sin(femurRad),
    };
    const lowerRad = Math.atan2(targetMm.y - knee.y, targetMm.x - knee.x);
    const femurDeg = femurRad * RAD2DEG;
    const tibiaDeg = lowerRad * RAD2DEG - LEGACY_CALF_FROM_TIBIA_OFFSET_DEG;
    const [angles] = clamp_joint_matrix(
      [[coxa, femurDeg * DEG2RAD, tibiaDeg * DEG2RAD]],
      0.0,
      DEFAULT_JOINT_LIMITS_RAD,
    );
    if (!angles) return;

    const pose = buildLegPoseFromJointAngles(toLegacyJointPose(angles));
    const foot = pose?.geometry?.foot;
    if (!foot) return;

    const dx = foot.x - targetMm.x;
    const dy = foot.y - targetMm.y;
    const femurDelta = angles[1] * RAD2DEG - currentFemurDeg;
    const tibiaDelta = angles[2] * RAD2DEG - currentTibiaDeg;
    const score = Math.hypot(dx, dy) + 0.03 * (Math.abs(femurDelta) + Math.abs(tibiaDelta));
    if (!best || score < best.score) best = { score, angles };
  };

  consider(base - elbow);
  consider(base + elbow);

  return best?.angles ?? null;
}

// ─── Helpers ─────────────────────────────────────────────────────────────

function readLegAnglesRad(jointState, leg) {
  // JointState messages come in with `name` + `position` parallel arrays.
  // We look up the three rows for this leg and return [coxa, femur, tibia]
  // in radians. Missing → 0 so the SVG always has something to draw.
  const out = [0, 0, 0];
  if (!jointState?.name || !jointState?.position) return out;
  const indexByName = Object.fromEntries(jointState.name.map((n, i) => [n, i]));
  JOINT_ROWS.forEach((row, i) => {
    const idx = indexByName[`${leg}_${row}_joint`];
    if (idx !== undefined && Number.isFinite(jointState.position[idx])) {
      out[i] = Number(jointState.position[idx]);
    }
  });
  return out;
}

function clampLegAngles(angles) {
  const clamped = clamp_joint_matrix([angles]);
  return clamped[0] ?? angles;
}

// Drag uses the same 2-link visual frame as the SVG, then sends the resulting
// [coxa, femur, tibia] joint vector through the same direct-joint path as the
// sliders. That keeps the sliders, preview, and ESP command in lock-step.

export function leg_angle_map(jointState) {
  // Utility for panels that want all 12 joints keyed by name.
  const out = {};
  if (!jointState?.name || !jointState?.position) return out;
  for (const n of JOINT_NAMES) {
    const idx = jointState.name.indexOf(n);
    if (idx >= 0 && Number.isFinite(jointState.position[idx])) out[n] = jointState.position[idx];
  }
  return out;
}
