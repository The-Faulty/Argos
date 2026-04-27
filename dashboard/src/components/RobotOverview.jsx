// 4-leg mini-overview.
//
// Four small leg cards arranged in a 2×2 grid, column 1 = left-side legs,
// column 2 = right-side legs. Clicking a card sets the active leg in the
// App shell — that's how the user zooms into a particular leg in LegDetail.
//
// Each card renders the sagittal chain at a reduced size so the mini preview
// matches the focused LegDetail view pixel-for-pixel. We share the same FK
// (buildLegPoseFromJointAngles from argos_kinematics.js — the same one the
// IK uses to validate poses on the Pi) so visual parity with LegDetail and
// with the physical robot is automatic.

import { useMemo } from "react";
import {
  buildLegPoseFromJointAngles,
  toCanvasPoint,
} from "../../shared/argos_kinematics.js";
import { JOINT_ROWS, LEG_DRAWING, LEG_IDS, LEG_LABELS } from "../../shared/robot-config.js";

// Grid order puts left-side legs (FL, RL) in column 1 and right-side legs
// (FR, RR) in column 2, so the 2×2 mini-overview reads top-down like a
// physical robot viewed from above. LEG_IDS keeps its canonical FR/FL/RR/RL
// order for every other consumer — only the rendering loop reorders.
const GRID_ORDER = ["FL", "FR", "RL", "RR"];

const RAD2DEG = 180.0 / Math.PI;
const CARD_W = 150;
const CARD_H = 120;

// Real FK is in meters; canvas helpers + LEG_DRAWING.scale=1.4 expect mm.
// Scale once at the FK→canvas boundary (matches LegDetail.jsx M_TO_MM).
const M_TO_MM = 1000.0;
function scalePoint(p) { return { x: p.x * M_TO_MM, y: p.y * M_TO_MM }; }

// Minimal segment set for the mini preview — thigh, calf, bell arms, horn,
// and the two push-rods. Geometry keys match argos_kinematics's FK output
// (same shape LegDetail consumes).
const MINI_SEGMENTS = [
  { key: "upper",     from: "O",      to: "knee",    stroke: "#141414", width: 6 },
  { key: "lower",     from: "knee",   to: "foot",    stroke: "#1d8cff", width: 6 },
  { key: "bell-a",    from: "O",      to: "bcRight", stroke: "#21a264", width: 4 },
  { key: "bell-b",    from: "O",      to: "bcLeft",  stroke: "#21a264", width: 4 },
  { key: "horn",      from: "O",      to: "horn",    stroke: "#ff5232", width: 4 },
  { key: "linkShort", from: "horn",   to: "bcRight", stroke: "#ef8a17", width: 4 },
  { key: "linkLong",  from: "bcLeft", to: "rod2",    stroke: "#6f4cff", width: 4 },
];

export default function RobotOverview({ activeLeg, onSelectLeg, jointState }) {
  // Compute all four poses at once — memoizing here avoids re-computing on
  // every parent re-render. buildLegPoseFromJointAngles takes radians and
  // returns geometry in meters; we scale to mm at the canvas boundary.
  const posesByLeg = useMemo(() => {
    const out = {};
    for (const leg of LEG_IDS) {
      const angles = readLegAngles(jointState, leg);
      try {
        const result = buildLegPoseFromJointAngles(angles, { legId: leg });
        out[leg] = result?.geometry
          ? { pose: { geometry: result.geometry, reachable: result.reachable }, angles }
          : { pose: null, angles };
      } catch {
        out[leg] = { pose: null, angles };
      }
    }
    return out;
  }, [jointState]);

  return (
    <div className="robot-overview" aria-label="Full-robot overview">
      <div className="robot-overview__grid">
        {GRID_ORDER.map((leg) => (
          <LegCard
            key={leg}
            leg={leg}
            isActive={leg === activeLeg}
            poseData={posesByLeg[leg]}
            onSelect={() => onSelectLeg?.(leg)}
          />
        ))}
      </div>
    </div>
  );
}

function LegCard({ leg, isActive, poseData, onSelect }) {
  const { pose, angles } = poseData ?? {};

  // Pre-map every geometry point to canvas pixels, same pattern as
  // LegDetail. If the pose is invalid we skip all the segment work.
  const points = pose?.geometry ? mapGeometry(pose.geometry) : null;

  // Right-side legs (FR, RR) are drawn mirrored about the vertical axis so
  // the mini cards read as "outside observer, one card per physical side"
  // rather than four identical sagittal projections. The same viewBox is
  // reused; a parent <g> flips x by translating to stageWidth then scaling
  // x by -1, which keeps segment geometry and stroke widths intact.
  const mirrorX = leg === "FR" || leg === "RR";

  return (
    <button
      type="button"
      className={`leg-card${isActive ? " is-active" : ""}`}
      onClick={onSelect}
      aria-label={`Focus ${LEG_LABELS[leg]}`}
    >
      <div className="leg-card__header">
        <span className="leg-card__id">{leg}</span>
        <span className="leg-card__name">{LEG_LABELS[leg]}</span>
      </div>

      <svg
        className="leg-card__svg"
        viewBox={`0 0 ${LEG_DRAWING.stageWidth} ${LEG_DRAWING.stageHeight}`}
        width={CARD_W}
        height={CARD_H}
        preserveAspectRatio="xMidYMid meet"
      >
        <g transform={mirrorX ? `translate(${LEG_DRAWING.stageWidth} 0) scale(-1 1)` : undefined}>
          {points ? (
            <>
              {MINI_SEGMENTS.map((seg) => (
                <path
                  key={seg.key}
                  d={`M ${points[seg.from].x} ${points[seg.from].y} L ${points[seg.to].x} ${points[seg.to].y}`}
                  stroke={seg.stroke}
                  strokeWidth={seg.width}
                  strokeLinecap="round"
                  fill="none"
                />
              ))}
              <circle cx={points.foot.x} cy={points.foot.y} r={14} fill="#1d8cff" />
            </>
          ) : (
            <text x={LEG_DRAWING.stageWidth / 2} y={LEG_DRAWING.stageHeight / 2} textAnchor="middle"
                  fontSize={40} fill="#dc2626">unreachable</text>
          )}
        </g>
      </svg>

      <dl className="leg-card__stats">
        {JOINT_ROWS.map((row, i) => (
          <div key={row}>
            <dt>{row}</dt>
            <dd>{Number.isFinite(angles?.[i]) ? (angles[i] * RAD2DEG).toFixed(1) + "°" : "—"}</dd>
          </div>
        ))}
      </dl>
    </button>
  );
}

function mapGeometry(geometry) {
  const p = (pt) => toCanvasPoint(scalePoint(pt), LEG_DRAWING);
  return {
    O:       p(geometry.O),
    knee:    p(geometry.knee),
    foot:    p(geometry.foot),
    horn:    p(geometry.horn),
    bcLeft:  p(geometry.bcLeft),
    bcRight: p(geometry.bcRight),
    rod2:    p(geometry.rod2),
  };
}

function readLegAngles(jointState, leg) {
  const out = [0, 0, 0];
  if (!jointState?.name || !jointState?.position) return out;
  JOINT_ROWS.forEach((row, i) => {
    const idx = jointState.name.indexOf(`${leg}_${row}_joint`);
    if (idx >= 0 && Number.isFinite(jointState.position[idx])) out[i] = Number(jointState.position[idx]);
  });
  return out;
}
