// Big SVG leg view — one leg, rendered with andy-servo's exact bell-crank
// sagittal chain. Per user direction ("the leg design is perfect, do not
// alter it"), the 420×420 stage, 7 colored segments, 8 joint dots, canvas
// transform, and reverse-solve all come from andy-servo-control verbatim.
//
// Argos is 3-DOF (coxa/femur/tibia) vs andy-servo's 2-DOF (thigh/calf), so:
//   - Coxa is read from /joint_states and shown as a number in the stats
//     grid. It is NOT drawn — the sagittal view is a 2-D side projection.
//   - Femur → andy-servo's "thigh", tibia → andy-servo's "calf". We convert
//     rad → deg and pass them into buildLegPoseFromJointAngles, which runs
//     andy-servo's coarse 5° sweep + 0.25° refinement over the servo horn
//     angle until the bell-crank output matches the requested calf.
//
// Data flow:
//   1. Read [coxa, femur, tibia] radians from /joint_states for the active
//      leg (`readLegAnglesRad`).
//   2. Feed (thigh=femur°, calf=tibia°) to buildLegPoseFromJointAngles.
//   3. Map every geometry point through toCanvasPoint once; draw segments
//      and dots in andy-servo's exact colors/widths.
//   4. Pointer drag on the foot inverts to a sagittal-frame XY (mm) and
//      forwards as a foot target in direct_foot_xyz mode.

import { useMemo, useRef } from "react";
import {
  fromCanvasPoint,
  LEGACY_LEG_GEOMETRY,
  LEGACY_NEUTRAL_CALIBRATION,
  clamp_joint_matrix,
  segmentPath,
  solveGeometryLegacy,
  solveServoForJointAngles,
  toCanvasPoint,
} from "../../shared/kinematics.js";
import {
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_DRAWING,
  LEG_IDS,
  LEG_LABELS,
  SERVO_CENTER_DEG,
} from "../../shared/robot-config.js";
import { applyServoCal } from "../../shared/servo_cal.js";

const RAD2DEG = 180.0 / Math.PI;

// Seven andy-servo segments, identical colors/widths. Order matters only
// for painter's ordering (later entries draw on top). Keys reference the
// legacy geometry names returned by buildLegPoseFromJointAngles.
const SEGMENTS = [
  { key: "upper",     from: "hip",          to: "knee",        stroke: "#141414", width: 5 }, // thigh
  { key: "lower",     from: "knee",         to: "foot",        stroke: "#1d8cff", width: 5 }, // calf
  { key: "bell-a",    from: "hip",          to: "bellArmA",    stroke: "#21a264", width: 4 }, // bell crank arm A
  { key: "bell-b",    from: "hip",          to: "bellArmB",    stroke: "#21a264", width: 4 }, // bell crank arm B
  { key: "horn",      from: "servoPivot",   to: "servoHornEnd",stroke: "#ff5232", width: 4 }, // servo horn
  { key: "linkShort", from: "servoHornEnd", to: "bellArmA",    stroke: "#ef8a17", width: 4 }, // rod: horn → bell arm A
  { key: "linkLong",  from: "bellArmB",     to: "calfAttach",  stroke: "#6f4cff", width: 4 }, // rod: bell arm B → calf attach
];

// The 7 non-foot dots. Foot is rendered separately (bigger, blue).
const DOT_KEYS = [
  "hip", "knee", "servoPivot", "servoHornEnd",
  "bellArmA", "bellArmB", "calfAttach",
];

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
      // Argos joint radians are *servo offsets from the 90° neutral*, not
      // absolute link angles. The legacy bell-crank FK expects an absolute
      // thigh angle and a servo-horn theta, so:
      //   thetaThigh = femur_rad (direct-drive femur; servo-neutral = thigh
      //                0° per LEGACY_NEUTRAL_CALIBRATION.thetaThigh).
      //   thetaServo = neutralThetaServo + tibia_rad (servo-horn rotates
      //                linearly with the tibia servo offset).
      // Going through solveGeometryLegacy directly (instead of the iterative
      // `buildLegPoseFromJointAngles`) avoids clamping tibia values into the
      // legacy absolute-calf window [-165°, -25°], which was silently pinning
      // the slider above -25°.
      const thetaThigh = effectiveAngles[1];
      const thetaServo = LEGACY_NEUTRAL_CALIBRATION.thetaServo + effectiveAngles[2];
      const geometry = solveGeometryLegacy(thetaThigh, thetaServo);
      return {
        geometry,
        foot: {
          x: geometry.foot.x - LEGACY_LEG_GEOMETRY.footOriginOffset.x,
          y: geometry.foot.y - LEGACY_LEG_GEOMETRY.footOriginOffset.y,
        },
        reachable: geometry.valid,
      };
    } catch {
      return null;
    }
  }, [effectiveAngles]);

  // Pre-map every geometry point to canvas pixels once per render — the
  // same pattern andy-servo's LegDetail uses. `segmentPath` expects
  // pre-mapped points so we don't pay the toCanvasPoint cost twice.
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
    // fromCanvasPoint returns millimeters in andy-servo's math frame
    // (+x forward, +y up). `absMm` is hip-relative (hip at origin); we
    // subtract footOriginOffset for the neutral-foot-near-origin value
    // the drag consumer uses.
    const absMm = fromCanvasPoint({ x: px, y: py }, LEG_DRAWING);
    const offset = LEGACY_LEG_GEOMETRY.footOriginOffset;
    const localMm = { x: absMm.x - offset.x, y: absMm.y - offset.y };

    // Optimistic preview: 2-link sagittal IK to (thighRad, calfAbsRad)
    // absolute, then map calf-absolute → Argos tibia (servo offset from
    // 90° neutral) via the bell-crank reverse-solve so the value we stash
    // is in the same convention as the slider. Storing absolute angles in
    // the preview was the bug behind "tibia slider isn't working".
    const ik = twoLinkSagittalIK(
      absMm.x,
      absMm.y,
      LEGACY_LEG_GEOMETRY.thighLength,
      LEGACY_LEG_GEOMETRY.calfLength,
      effectiveAngles[1], // continuity hint in radians
    );
    if (ik) {
      const servoSolution = solveServoForJointAngles(ik.thighRad, ik.calfAbsRad);
      const tibiaRad = servoSolution?.thetaServo != null
        ? servoSolution.thetaServo - LEGACY_NEUTRAL_CALIBRATION.thetaServo
        : effectiveAngles[2];
      const nextAngles = clampLegAngles([effectiveAngles[0], ik.thighRad, tibiaRad]);
      onPreviewAngles?.({
        leg: activeLeg,
        angles: nextAngles,
      });
      onFootDrag?.({ x: localMm.x, z: localMm.y, leg: activeLeg, angles: nextAngles });
      return;
    }

    // andy-servo's frame labels the vertical axis y; the dashboard thinks
    // in (x, z) sagittal. Forward both under the same name contract the
    // old LegDetail used so upstream code doesn't need changes.
    onFootDrag?.({ x: localMm.x, z: localMm.y, leg: activeLeg });
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
                  d={segmentPath(points[seg.from], points[seg.to])}
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
  // Light horizontal baseline + vertical body-line so the pose has some
  // visual context. Ratios match andy-servo's feel without copying any of
  // their dashed-line gymnastics wholesale.
  const mid = LEG_DRAWING.stageHeight * 0.75;
  return (
    <g>
      <line
        x1={10} x2={LEG_DRAWING.stageWidth - 10} y1={mid} y2={mid}
        stroke="rgba(0,0,0,0.08)" strokeDasharray="4 4"
      />
      <line
        x1={LEG_DRAWING.offset.x} x2={LEG_DRAWING.offset.x} y1={10}
        y2={LEG_DRAWING.stageHeight - 10}
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

// ─── 2-link sagittal IK (local drag preview) ─────────────────────────────
//
// Standard planar two-link IK: hip at origin, thigh length L1, calf length
// L2, foot target (fx, fy) in andy-servo's math frame (+x forward, +y up).
// Returns absolute-angle pair {thighRad, calfAbsRad} in the same convention
// the bell-crank reverse-solve (buildLegPoseFromJointAngles) expects.
//
// There are two IK branches (elbow-up vs elbow-down). The leg has a visually
// "correct" branch depending on which side the knee should bend; we pick the
// one whose thigh angle is closer to the continuity hint (the current thigh
// angle from jointState), so the leg doesn't snap through itself mid-drag.
//
// If the target is outside the reachable annulus [|L1-L2|, L1+L2], the
// target is clamped to the closest reachable radius before the solve runs.
function twoLinkSagittalIK(fx, fy, L1, L2, hintThighRad) {
  const r2 = fx * fx + fy * fy;
  const maxR = L1 + L2 - 1e-6;
  const minR = Math.abs(L1 - L2) + 1e-6;
  let r = Math.sqrt(r2);
  if (r < 1e-6) return null;
  // Clamp target to reachable annulus.
  if (r > maxR) {
    const s = maxR / r;
    fx *= s; fy *= s; r = maxR;
  } else if (r < minR) {
    const s = minR / r;
    fx *= s; fy *= s; r = minR;
  }

  const cosE = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const clamped = Math.max(-1, Math.min(1, cosE));
  const elbowMagnitude = Math.acos(clamped);

  // Two branches: elbow = +E (knee bends one way) vs elbow = -E.
  const candidates = [+elbowMagnitude, -elbowMagnitude].map((elbow) => {
    const k1 = L1 + L2 * Math.cos(elbow);
    const k2 = L2 * Math.sin(elbow);
    const thighRad = Math.atan2(fy, fx) - Math.atan2(k2, k1);
    return { thighRad, calfAbsRad: thighRad + elbow };
  });

  // Pick the branch whose thigh angle is closest to the continuity hint.
  // Use a wrapped-angle distance so 179° and -179° are treated as close.
  const hint = Number.isFinite(hintThighRad) ? hintThighRad : -Math.PI / 2;
  const dist = (a, b) => {
    let d = a - b;
    while (d >  Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    return Math.abs(d);
  };
  candidates.sort((a, b) => dist(a.thighRad, hint) - dist(b.thighRad, hint));
  return candidates[0];
}

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
