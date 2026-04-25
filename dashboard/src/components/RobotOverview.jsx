// 4-leg mini-overview.
//
// Four small leg cards arranged in a 2×2 grid, column 1 = left-side legs,
// column 2 = right-side legs. Clicking a card sets the active leg in the
// App shell — that's how the user zooms into a particular leg in LegDetail.
//
// Each card renders andy-servo's sagittal chain at a reduced size so the
// mini preview matches the focused LegDetail view pixel-for-pixel. We share
// the same buildLegPoseFromJointAngles pipeline (the legacy bell-crank
// reverse-solve) so that visual parity is automatic — if LegDetail's
// rendering changes, the mini cards follow.

import { useMemo } from "react";
import {
  LEGACY_LEG_GEOMETRY,
  LEGACY_NEUTRAL_CALIBRATION,
  solveGeometryLegacy,
  toCanvasPoint,
} from "../../shared/kinematics.js";
import { JOINT_ROWS, LEG_DRAWING, LEG_IDS, LEG_LABELS } from "../../shared/robot-config.js";

// Grid order puts left-side legs (FL, RL) in column 1 and right-side legs
// (FR, RR) in column 2, so the 2×2 mini-overview reads top-down like a
// physical robot viewed from above. LEG_IDS keeps its canonical FR/FL/RR/RL
// order for every other consumer — only the rendering loop reorders.
const GRID_ORDER = ["FL", "FR", "RL", "RR"];

const RAD2DEG = 180.0 / Math.PI;
const CARD_W = 150;
const CARD_H = 120;

// Minimal segment set for the mini preview — just the thigh, calf, and
// bell arms. Full 7-segment rendering would be unreadable at 150px wide.
// Colors + widths still come from andy-servo for visual parity with
// LegDetail, just scaled down (widths ÷ ~2).
const MINI_SEGMENTS = [
  { key: "upper",     from: "hip",          to: "knee",        stroke: "#141414", width: 6 },
  { key: "lower",     from: "knee",         to: "foot",        stroke: "#1d8cff", width: 6 },
  { key: "bell-a",    from: "hip",          to: "bellArmA",    stroke: "#21a264", width: 4 },
  { key: "bell-b",    from: "hip",          to: "bellArmB",    stroke: "#21a264", width: 4 },
  { key: "horn",      from: "servoPivot",   to: "servoHornEnd",stroke: "#ff5232", width: 4 },
  { key: "linkShort", from: "servoHornEnd", to: "bellArmA",    stroke: "#ef8a17", width: 4 },
  { key: "linkLong",  from: "bellArmB",     to: "calfAttach",  stroke: "#6f4cff", width: 4 },
];

export default function RobotOverview({ activeLeg, onSelectLeg, jointState }) {
  // Compute all four poses at once — they share the geometry table, and
  // memoizing here avoids re-computing on every parent re-render. Same FK
  // path as LegDetail: femur_rad → thetaThigh (direct-drive), tibia_rad →
  // servo-horn theta = neutralThetaServo + tibia_rad. Avoids the reverse-
  // solve clamp that used to pin rendering to legacy calf ∈ [-165°, -25°].
  const posesByLeg = useMemo(() => {
    const out = {};
    for (const leg of LEG_IDS) {
      const angles = readLegAngles(jointState, leg);
      try {
        const thetaThigh = angles[1];
        const thetaServo = LEGACY_NEUTRAL_CALIBRATION.thetaServo + angles[2];
        const geometry = solveGeometryLegacy(thetaThigh, thetaServo);
        const pose = {
          geometry,
          foot: {
            x: geometry.foot.x - LEGACY_LEG_GEOMETRY.footOriginOffset.x,
            y: geometry.foot.y - LEGACY_LEG_GEOMETRY.footOriginOffset.y,
          },
          reachable: geometry.valid,
        };
        out[leg] = { pose, angles };
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
  const p = (pt) => toCanvasPoint(pt, LEG_DRAWING);
  return {
    hip:          p(geometry.hip),
    knee:         p(geometry.knee),
    foot:         p(geometry.foot),
    servoPivot:   p(geometry.servoPivot),
    servoHornEnd: p(geometry.servoHornEnd),
    bellArmA:     p(geometry.bellArmA),
    bellArmB:     p(geometry.bellArmB),
    calfAttach:   p(geometry.calfAttach),
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
