// Andy-servo-control's sagittal kinematics, preserved verbatim for the
// dashboard's visual preview. Per user direction ("the leg design is
// perfect, do not alter it"), the 2-D sagittal leg diagram in LegDetail
// and RobotOverview must render with andy-servo's exact geometry keys,
// colors, weights, canvas transform, and coupled-linkage bell-crank
// solution. This file therefore re-implements andy-servo's full sagittal
// pipeline — FK, reverse-solve from (thigh, calf) joint angles to the
// servo pair that produces them, canvas helpers, and geometry defaults
// — in pixel space (millimeters).
//
// The Argos 3-DOF real-robot kinematics (for `/joint_command/raw`) stays
// in argos_kinematics.js. That path takes meters and runs through the
// Python-authoritative IK. Rendering uses the helpers here so the
// dashboard visually matches the firmware's bell-crank hardware model
// regardless of what the IK is doing.
//
// Argos has three joints per leg (coxa, femur, tibia). The coxa (hip
// abduction) is NOT drawn in the sagittal view — per-user direction it
// renders as a numeric stat only. LegDetail pipes (femur, tibia) in
// degrees into `buildLegPoseFromJointAngles` as {thigh, calf}; the solver
// reverse-iterates the servo horn angle so the resulting thetaCalf
// matches the requested tibia, then returns the full 8-point geometry.

import { LEG_DRAWING } from "./robot-config.js";

// ─── Legacy geometry defaults (pixel units, from andy-servo robot-config) ─
export const LEGACY_LEG_GEOMETRY = {
  thighLength: 127,
  calfLength: 127,
  hornLength: 20,
  linkShort: 30,
  bellLength: 40,
  linkLong: 150,
  servoPivot: { x: -20, y: -22 },
  hipPivot: { x: 0, y: 0 },
  calfAttachOffset: 30,
  footOriginOffset: { x: 40, y: -140 },
};

export const LEGACY_JOINT_LIMITS = {
  thighDeg: { min: -145, max: 15 },
  calfDeg:  { min: -165, max: -25 },
};

export const LEGACY_NEUTRAL_CALIBRATION = {
  thetaThigh: 0,
  thetaServo: -2.768896484375,
};

// Servo-space search bounds (model horn angle, 0–180°) and the coarse →
// fine refinement schedule used by the reverse-solve. Values match andy-
// servo's `kinematics.js` constants exactly.
const SERVO_SEARCH_STEP_DEG = 5;
const SERVO_REFINE_STEPS_DEG = [2, 1, 0.5, 0.25];
const SERVO_MIN_DEG = 0;
const SERVO_MAX_DEG = 180;

// ─── Scalar helpers ──────────────────────────────────────────────────────

function clampValue(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export const radToDeg = (r) => (r * 180) / Math.PI;
export const degToRad = (d) => (d * Math.PI) / 180;

export function clampAngle(angle) {
  const twoPi = Math.PI * 2;
  let value = angle % twoPi;
  if (value <= -Math.PI) value += twoPi;
  if (value >   Math.PI) value -= twoPi;
  return value;
}

export function normalizeJointLimits(limits = LEGACY_JOINT_LIMITS) {
  const thighMin = Number.isFinite(limits?.thighDeg?.min) ? limits.thighDeg.min : LEGACY_JOINT_LIMITS.thighDeg.min;
  const thighMax = Number.isFinite(limits?.thighDeg?.max) ? limits.thighDeg.max : LEGACY_JOINT_LIMITS.thighDeg.max;
  const calfMin  = Number.isFinite(limits?.calfDeg?.min)  ? limits.calfDeg.min  : LEGACY_JOINT_LIMITS.calfDeg.min;
  const calfMax  = Number.isFinite(limits?.calfDeg?.max)  ? limits.calfDeg.max  : LEGACY_JOINT_LIMITS.calfDeg.max;
  return {
    thighDeg: { min: Math.min(thighMin, thighMax), max: Math.max(thighMin, thighMax) },
    calfDeg:  { min: Math.min(calfMin,  calfMax ), max: Math.max(calfMin,  calfMax ) },
  };
}

export function clampJointAnglesToLimits(jointAnglesDeg, limits = LEGACY_JOINT_LIMITS) {
  const normalized = normalizeJointLimits(limits);
  return {
    thigh: clampValue(jointAnglesDeg.thigh, normalized.thighDeg.min, normalized.thighDeg.max),
    calf:  clampValue(jointAnglesDeg.calf,  normalized.calfDeg.min,  normalized.calfDeg.max),
  };
}

export function geometryWithinJointLimits(geometry, limits = LEGACY_JOINT_LIMITS) {
  if (!geometry?.valid) return false;
  const n = normalizeJointLimits(limits);
  const thighDeg = radToDeg(geometry.thetaThigh);
  const calfDeg  = radToDeg(geometry.thetaCalf);
  return (
    thighDeg >= n.thighDeg.min && thighDeg <= n.thighDeg.max &&
    calfDeg  >= n.calfDeg.min  && calfDeg  <= n.calfDeg.max
  );
}

export function createNeutralCalibration() {
  return {
    thetaThigh: LEGACY_NEUTRAL_CALIBRATION.thetaThigh,
    thetaServo: LEGACY_NEUTRAL_CALIBRATION.thetaServo,
  };
}

export function modelServoDegToTheta(modelDeg, neutralTheta) {
  return neutralTheta + degToRad(modelDeg - 90);
}

export function thetaToModelServoDeg(theta, neutralTheta) {
  return 90 + radToDeg(theta - neutralTheta);
}

// ─── Circle intersection (shared by FK + IK) ─────────────────────────────

export function circleIntersections(c0, r0, c1, r1) {
  const dx = c1.x - c0.x;
  const dy = c1.y - c0.y;
  const d = Math.hypot(dx, dy);
  if (d === 0 || d > r0 + r1 || d < Math.abs(r0 - r1)) return [];

  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const hSq = r0 * r0 - a * a;
  if (hSq < 0) return [];
  const h = Math.sqrt(hSq);
  const xm = c0.x + (a * dx) / d;
  const ym = c0.y + (a * dy) / d;
  const rx = (-dy * h) / d;
  const ry = ( dx * h) / d;
  return [
    { x: xm + rx, y: ym + ry },
    { x: xm - rx, y: ym - ry },
  ];
}

function chooseBellPoint(points) {
  return points.reduce((best, p) => (best == null || p.y > best.y ? p : best), null);
}

function chooseCalfAttachPoint(points, knee) {
  return points.reduce((best, p) => {
    const score = (knee.y - p.y) * 3 + Math.abs(knee.x - p.x);
    if (best == null || score < best.score) return { point: p, score };
    return best;
  }, null)?.point ?? null;
}

function makeFallback(thetaThigh, thetaServo, g) {
  const hip = g.hipPivot;
  const servoPivot = g.servoPivot;
  const knee = {
    x: hip.x + g.thighLength * Math.cos(thetaThigh),
    y: hip.y + g.thighLength * Math.sin(thetaThigh),
  };
  const servoHornEnd = {
    x: servoPivot.x + g.hornLength * Math.cos(thetaServo),
    y: servoPivot.y + g.hornLength * Math.sin(thetaServo),
  };
  return {
    hip, servoPivot, knee, servoHornEnd,
    bellArmA: { ...hip },
    bellArmB: { ...hip },
    calfAttach: { ...knee },
    foot: { ...knee },
    thetaThigh, thetaServo, thetaCalf: thetaThigh,
    valid: false,
  };
}

/** 2-DOF sagittal FK using andy-servo's bell-crank math. */
export function solveGeometryLegacy(thetaThigh, thetaServo, g = LEGACY_LEG_GEOMETRY) {
  const geom = makeFallback(thetaThigh, thetaServo, g);
  const { hip, servoPivot, knee, servoHornEnd } = geom;

  const bellCandidates = circleIntersections(hip, g.bellLength, servoHornEnd, g.linkShort);
  const bellArmA = chooseBellPoint(bellCandidates);
  if (!bellArmA) return geom;

  const bellAngle = Math.atan2(bellArmA.y - hip.y, bellArmA.x - hip.x);
  const bellArmB = {
    x: hip.x + g.bellLength * Math.cos(bellAngle - Math.PI / 2),
    y: hip.y + g.bellLength * Math.sin(bellAngle - Math.PI / 2),
  };
  geom.bellArmA = bellArmA;
  geom.bellArmB = bellArmB;

  const calfAttachCandidates = circleIntersections(bellArmB, g.linkLong, knee, g.calfAttachOffset);
  const calfAttach = chooseCalfAttachPoint(calfAttachCandidates, knee);
  if (!calfAttach) return geom;

  const thetaCalf = Math.atan2(knee.y - calfAttach.y, knee.x - calfAttach.x);
  geom.calfAttach = calfAttach;
  geom.foot = {
    x: knee.x + g.calfLength * Math.cos(thetaCalf),
    y: knee.y + g.calfLength * Math.sin(thetaCalf),
  };
  geom.thetaCalf = thetaCalf;
  geom.valid = true;
  return geom;
}

// Alias preserved so external callers can refer to the legacy FK as
// `solveGeometry` without knowing about the _Legacy suffix. The andy-servo
// reference used this name directly.
export const solveGeometry = solveGeometryLegacy;

// ─── Servo / joint search helpers (reverse-solve) ────────────────────────

function servoRangePenalty(modelDeg) {
  if (modelDeg < 0) return -modelDeg;
  if (modelDeg > 180) return modelDeg - 180;
  return 0;
}

function servoSolutionPenalty(thetaThigh, thetaServo, calibration) {
  if (!calibration) return 0;
  return (
    servoRangePenalty(thetaToModelServoDeg(thetaThigh, calibration.thetaThigh)) +
    servoRangePenalty(thetaToModelServoDeg(thetaServo, calibration.thetaServo))
  );
}

function jointCandidateBetter(candidate, best) {
  if (!best) return true;
  if (candidate.error < best.error - degToRad(0.75)) return true;
  if (candidate.error > best.error + degToRad(0.75)) return false;
  if (candidate.servoPenalty < best.servoPenalty - 1e-6) return true;
  if (candidate.servoPenalty > best.servoPenalty + 1e-6) return false;
  return candidate.error < best.error;
}

function jointServoCandidateBetter(candidate, best) {
  if (!best) return true;

  const a = { servoPenalty: candidate.servoPenalty, error: candidate.error };
  const b = { servoPenalty: best.servoPenalty,     error: best.error     };
  if (jointCandidateBetter(a, b)) return true;
  if (jointCandidateBetter(b, a)) return false;

  return candidate.continuityPenalty < best.continuityPenalty;
}

function evaluateJointServoCandidate(thetaThigh, modelCalfDeg, targetThetaCalf, limits, calibration, startModelDeg) {
  if (modelCalfDeg < SERVO_MIN_DEG || modelCalfDeg > SERVO_MAX_DEG) return null;

  const thetaServo = modelServoDegToTheta(modelCalfDeg, calibration.thetaServo);
  const geometry = solveGeometryLegacy(thetaThigh, thetaServo);

  if (!geometryWithinJointLimits(geometry, limits)) return null;

  return {
    modelCalfDeg,
    thetaServo,
    thetaCalf: geometry.thetaCalf,
    geometry,
    error: Math.abs(clampAngle(geometry.thetaCalf - targetThetaCalf)),
    servoPenalty: servoSolutionPenalty(thetaThigh, thetaServo, calibration),
    continuityPenalty: startModelDeg == null ? 0 : Math.abs(modelCalfDeg - startModelDeg),
  };
}

function refineJointServoCandidate(best, thetaThigh, targetThetaCalf, limits, calibration, startModelDeg) {
  if (!best) return null;

  for (const step of SERVO_REFINE_STEPS_DEG) {
    let improved = true;
    while (improved) {
      improved = false;
      for (const candidateCalfDeg of [best.modelCalfDeg + step, best.modelCalfDeg - step]) {
        const candidate = evaluateJointServoCandidate(
          thetaThigh, candidateCalfDeg, targetThetaCalf, limits, calibration, startModelDeg,
        );
        if (candidate && jointServoCandidateBetter(candidate, best)) {
          best = candidate;
          improved = true;
        }
      }
    }
  }
  return best;
}

/**
 * Reverse-solve the servo horn angle that produces the requested calf joint
 * angle for a given thigh angle. Uses andy-servo's coarse 5° sweep +
 * {2°,1°,0.5°,0.25°} refinement, so the returned geometry matches the
 * bell-crank output exactly.
 */
export function solveServoForJointAngles(
  thetaThigh,
  targetThetaCalf,
  startServo = Math.PI / 6,
  limits = LEGACY_JOINT_LIMITS,
  calibration = null,
) {
  const activeCalibration = calibration ?? createNeutralCalibration();
  const startModelDeg = clampValue(
    thetaToModelServoDeg(startServo, activeCalibration.thetaServo),
    SERVO_MIN_DEG, SERVO_MAX_DEG,
  );
  let bestSolution = null;

  for (let modelCalfDeg = SERVO_MIN_DEG; modelCalfDeg <= SERVO_MAX_DEG; modelCalfDeg += SERVO_SEARCH_STEP_DEG) {
    const candidate = evaluateJointServoCandidate(
      thetaThigh, modelCalfDeg, targetThetaCalf, limits, activeCalibration, startModelDeg,
    );
    if (candidate && jointServoCandidateBetter(candidate, bestSolution)) {
      bestSolution = candidate;
    }
  }

  bestSolution = refineJointServoCandidate(
    bestSolution, thetaThigh, targetThetaCalf, limits, activeCalibration, startModelDeg,
  );

  if (!bestSolution) {
    const fallbackGeometry = solveGeometryLegacy(thetaThigh, startServo);
    return {
      thetaServo: startServo,
      thetaCalf: fallbackGeometry.thetaCalf,
      jointError: Number.POSITIVE_INFINITY,
      servoPenalty: Number.POSITIVE_INFINITY,
      geometry: fallbackGeometry,
      hasSolution: false,
      success: false,
    };
  }

  return {
    thetaServo: bestSolution.thetaServo,
    thetaCalf: bestSolution.thetaCalf,
    jointError: bestSolution.error,
    servoPenalty: bestSolution.servoPenalty,
    geometry: bestSolution.geometry,
    hasSolution: bestSolution.geometry.valid && Number.isFinite(bestSolution.error),
    success: bestSolution.geometry.valid && Number.isFinite(bestSolution.error) && bestSolution.error < degToRad(3),
  };
}

// ─── Pose builders (LegDetail / RobotOverview consume these) ─────────────

/**
 * Build a full leg pose from a {thigh, calf} degree pair. Returns the exact
 * 8-point bell-crank geometry andy-servo's LegDetail rendering consumes, the
 * foot position in sagittal frame (relative to `footOriginOffset` so the
 * neutral foot sits near the origin), plus the matching model-space servo
 * angles.
 */
export function buildLegPoseFromJointAngles(jointAnglesDeg, calibration = createNeutralCalibration(), options = {}) {
  const limits = normalizeJointLimits(options.jointLimits);
  const clamped = clampJointAnglesToLimits(jointAnglesDeg, limits);
  const thetaThigh = degToRad(clamped.thigh);
  const servoSolution = solveServoForJointAngles(
    thetaThigh,
    degToRad(clamped.calf),
    options.startThetaServo ?? Math.PI / 6,
    limits,
    calibration,
  );
  const geometry = servoSolution.geometry;

  return {
    geometry,
    foot: {
      x: geometry.foot.x - LEGACY_LEG_GEOMETRY.footOriginOffset.x,
      y: geometry.foot.y - LEGACY_LEG_GEOMETRY.footOriginOffset.y,
    },
    jointAnglesDeg: { thigh: clamped.thigh, calf: clamped.calf },
    servoAnglesDeg: {
      thigh: thetaToModelServoDeg(thetaThigh, calibration.thetaThigh),
      calf:  thetaToModelServoDeg(servoSolution.thetaServo, calibration.thetaServo),
    },
    reachable: servoSolution.hasSolution,
    withinTolerance: servoSolution.success,
    footError: 0,
  };
}

// ─── Canvas helpers (pixel-space, with andy-servo's y-flip) ──────────────

/**
 * Map a sagittal-frame point (millimeters, math frame with +y up) into canvas
 * pixel coordinates. Andy-servo's original helper negates `y` so the leg
 * hangs down the way humans expect on screen; we preserve that sign exactly.
 */
export function toCanvasPoint(point, drawing = LEG_DRAWING) {
  return {
    x:  point.x * drawing.scale + drawing.offset.x,
    y: -point.y * drawing.scale + drawing.offset.y,
  };
}

/** Inverse of `toCanvasPoint`. Used by pointer-drag in LegDetail. */
export function fromCanvasPoint(point, drawing = LEG_DRAWING) {
  return {
    x:  (point.x - drawing.offset.x) / drawing.scale,
    y: -(point.y - drawing.offset.y) / drawing.scale,
  };
}

/**
 * SVG path string connecting two points. Expects pre-mapped canvas points
 * (run the endpoints through `toCanvasPoint` first) — matches andy-servo's
 * LegDetail pattern where the component builds `points = { hip: p(...), … }`
 * once per render and then threads those into segmentPath.
 */
export function segmentPath(a, b) {
  return `M ${a.x} ${a.y} L ${b.x} ${b.y}`;
}
