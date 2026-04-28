import { DEFAULT_JOINT_LIMITS, LEG_DRAWING, LEG_GEOMETRY, LEG_IDS, NEUTRAL_CALIBRATION, ROBOT_LAYOUT } from "./robot-config.js";

function clampValue(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function normalizeJointLimits(limits = DEFAULT_JOINT_LIMITS) {
  const hipYawMin = Number.isFinite(limits?.hipYawDeg?.min) ? limits.hipYawDeg.min : DEFAULT_JOINT_LIMITS.hipYawDeg.min;
  const hipYawMax = Number.isFinite(limits?.hipYawDeg?.max) ? limits.hipYawDeg.max : DEFAULT_JOINT_LIMITS.hipYawDeg.max;
  const thighMin = Number.isFinite(limits?.thighDeg?.min) ? limits.thighDeg.min : DEFAULT_JOINT_LIMITS.thighDeg.min;
  const thighMax = Number.isFinite(limits?.thighDeg?.max) ? limits.thighDeg.max : DEFAULT_JOINT_LIMITS.thighDeg.max;
  const calfMin = Number.isFinite(limits?.calfDeg?.min) ? limits.calfDeg.min : DEFAULT_JOINT_LIMITS.calfDeg.min;
  const calfMax = Number.isFinite(limits?.calfDeg?.max) ? limits.calfDeg.max : DEFAULT_JOINT_LIMITS.calfDeg.max;
  const minThighCalfAngleDeg = Number.isFinite(limits?.minThighCalfAngleDeg)
    ? clampValue(limits.minThighCalfAngleDeg, 0, 180)
    : null;
  const normalized = {
    hipYawDeg: {
      min: Math.min(hipYawMin, hipYawMax),
      max: Math.max(hipYawMin, hipYawMax)
    },
    thighDeg: {
      min: Math.min(thighMin, thighMax),
      max: Math.max(thighMin, thighMax)
    },
    calfDeg: {
      min: Math.min(calfMin, calfMax),
      max: Math.max(calfMin, calfMax)
    }
  };
  if (minThighCalfAngleDeg != null) {
    normalized.minThighCalfAngleDeg = minThighCalfAngleDeg;
  }
  return normalized;
}

export function clampJointAnglesToLimits(jointAnglesDeg, limits = DEFAULT_JOINT_LIMITS) {
  const normalized = normalizeJointLimits(limits);
  const clamped = {
    hipYaw: clampValue(jointAnglesDeg.hipYaw ?? 0, normalized.hipYawDeg.min, normalized.hipYawDeg.max),
    thigh: clampValue(jointAnglesDeg.thigh, normalized.thighDeg.min, normalized.thighDeg.max),
    calf: clampValue(jointAnglesDeg.calf, normalized.calfDeg.min, normalized.calfDeg.max)
  };

  if (!(normalized.minThighCalfAngleDeg > 0)) {
    return clamped;
  }

  const maxSegmentDeltaDeg = 180 - normalized.minThighCalfAngleDeg;
  const calfMinFromThigh = Math.max(normalized.calfDeg.min, clamped.thigh - maxSegmentDeltaDeg);
  const calfMaxFromThigh = Math.min(normalized.calfDeg.max, clamped.thigh + maxSegmentDeltaDeg);

  if (calfMinFromThigh <= calfMaxFromThigh) {
    clamped.calf = clampValue(clamped.calf, calfMinFromThigh, calfMaxFromThigh);
  }

  return clamped;
}

export function geometryWithinJointLimits(geometry, limits = DEFAULT_JOINT_LIMITS) {
  if (!geometry?.valid) {
    return false;
  }
  const normalized = normalizeJointLimits(limits);
  const thighDeg = radToDeg(geometry.thetaThigh);
  const calfDeg = radToDeg(geometry.thetaCalf);
  const thighCalfAngleDeg = getThighCalfAngleDeg(geometry);
  return (
    thighDeg >= normalized.thighDeg.min &&
    thighDeg <= normalized.thighDeg.max &&
    calfDeg >= normalized.calfDeg.min &&
    calfDeg <= normalized.calfDeg.max &&
    thighCalfAngleDeg >= (normalized.minThighCalfAngleDeg ?? 0)
  );
}

export function getThighCalfAngleDeg(geometry) {
  if (!geometry?.knee || !geometry?.hip || !geometry?.foot) {
    return 0;
  }

  const thighVector = {
    x: geometry.hip.x - geometry.knee.x,
    y: geometry.hip.y - geometry.knee.y,
  };
  const calfVector = {
    x: geometry.foot.x - geometry.knee.x,
    y: geometry.foot.y - geometry.knee.y,
  };
  const thighLength = Math.hypot(thighVector.x, thighVector.y);
  const calfLength = Math.hypot(calfVector.x, calfVector.y);

  if (thighLength < 1e-9 || calfLength < 1e-9) {
    return 0;
  }

  const cosine = clampValue(
    ((thighVector.x * calfVector.x) + (thighVector.y * calfVector.y)) / (thighLength * calfLength),
    -1,
    1,
  );
  return radToDeg(Math.acos(cosine));
}

export function clampAngle(angle) {
  const twoPi = Math.PI * 2;
  let value = angle % twoPi;
  if (value <= -Math.PI) value += twoPi;
  if (value > Math.PI) value -= twoPi;
  return value;
}

export function radToDeg(radians) {
  return (radians * 180) / Math.PI;
}

export function degToRad(degrees) {
  return (degrees * Math.PI) / 180;
}

function clampServoAngleDeg(value, fallback = 90) {
  const numeric = Number.isFinite(value) ? value : fallback;
  return clampValue(numeric, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

function normalizeServoAngles(servoAnglesDeg = {}) {
  return {
    hipYaw: clampServoAngleDeg(servoAnglesDeg.hipYaw, 90),
    thigh: clampServoAngleDeg(servoAnglesDeg.thigh, 90),
    calf: clampServoAngleDeg(servoAnglesDeg.calf, 90),
  };
}

function servoRangePenalty(modelDeg) {
  if (modelDeg < 0) {
    return -modelDeg;
  }
  if (modelDeg > 180) {
    return modelDeg - 180;
  }
  return 0;
}

function servoSolutionPenalty(thetaThigh, thetaServo, calibration) {
  if (!calibration) {
    return 0;
  }

  return (
    servoRangePenalty(thetaToModelServoDeg(thetaThigh, calibration.thetaThigh)) +
    servoRangePenalty(thetaToModelServoDeg(thetaServo, calibration.thetaServo))
  );
}

function footCandidateBetter(candidate, best) {
  if (!best) {
    return true;
  }
  if (candidate.error < best.error - 0.5) {
    return true;
  }
  if (candidate.error > best.error + 0.5) {
    return false;
  }
  if (candidate.servoPenalty < best.servoPenalty - 1e-6) {
    return true;
  }
  if (candidate.servoPenalty > best.servoPenalty + 1e-6) {
    return false;
  }
  return candidate.error < best.error;
}

function jointCandidateBetter(candidate, best) {
  if (!best) {
    return true;
  }
  if (candidate.error < best.error - degToRad(0.75)) {
    return true;
  }
  if (candidate.error > best.error + degToRad(0.75)) {
    return false;
  }
  if (candidate.servoPenalty < best.servoPenalty - 1e-6) {
    return true;
  }
  if (candidate.servoPenalty > best.servoPenalty + 1e-6) {
    return false;
  }
  return candidate.error < best.error;
}

const SERVO_SEARCH_STEP_DEG = 5;
const SERVO_REFINE_STEPS_DEG = [2, 1, 0.5, 0.25];
const SERVO_MIN_DEG = 0;
const SERVO_MAX_DEG = 180;

function candidateContinuityPenalty(candidate, startModelDeg) {
  if (!startModelDeg) {
    return 0;
  }

  return (
    Math.abs(candidate.modelThighDeg - startModelDeg.thigh) +
    Math.abs(candidate.modelCalfDeg - startModelDeg.calf)
  );
}

function footServoCandidateBetter(candidate, best) {
  if (!best) {
    return true;
  }

  if (
    footCandidateBetter(
      { servoPenalty: candidate.servoPenalty, error: candidate.error },
      { servoPenalty: best.servoPenalty, error: best.error },
    )
  ) {
    return true;
  }

  if (
    footCandidateBetter(
      { servoPenalty: best.servoPenalty, error: best.error },
      { servoPenalty: candidate.servoPenalty, error: candidate.error },
    )
  ) {
    return false;
  }

  return candidate.continuityPenalty < best.continuityPenalty;
}

function jointServoCandidateBetter(candidate, best) {
  if (!best) {
    return true;
  }

  if (
    jointCandidateBetter(
      { servoPenalty: candidate.servoPenalty, error: candidate.error },
      { servoPenalty: best.servoPenalty, error: best.error },
    )
  ) {
    return true;
  }

  if (
    jointCandidateBetter(
      { servoPenalty: best.servoPenalty, error: best.error },
      { servoPenalty: candidate.servoPenalty, error: candidate.error },
    )
  ) {
    return false;
  }

  return candidate.continuityPenalty < best.continuityPenalty;
}

function evaluateFootServoCandidate(modelThighDeg, modelCalfDeg, target, limits, calibration, startModelDeg) {
  if (
    modelThighDeg < SERVO_MIN_DEG ||
    modelThighDeg > SERVO_MAX_DEG ||
    modelCalfDeg < SERVO_MIN_DEG ||
    modelCalfDeg > SERVO_MAX_DEG
  ) {
    return null;
  }

  const thetaThigh = modelServoDegToTheta(modelThighDeg, calibration.thetaThigh);
  const thetaServo = modelServoDegToTheta(modelCalfDeg, calibration.thetaServo);
  const geometry = solveGeometry(thetaThigh, thetaServo);

  if (!geometryWithinJointLimits(geometry, limits)) {
    return null;
  }

  return {
    modelThighDeg,
    modelCalfDeg,
    thetaThigh,
    thetaServo,
    thetaCalf: geometry.thetaCalf,
    geometry,
    error: Math.hypot(geometry.foot.x - target.x, geometry.foot.y - target.y),
    servoPenalty: servoSolutionPenalty(thetaThigh, thetaServo, calibration),
    continuityPenalty: candidateContinuityPenalty({ modelThighDeg, modelCalfDeg }, startModelDeg),
  };
}

function evaluateJointServoCandidate(thetaThigh, modelCalfDeg, targetThetaCalf, limits, calibration, startModelDeg) {
  if (modelCalfDeg < SERVO_MIN_DEG || modelCalfDeg > SERVO_MAX_DEG) {
    return null;
  }

  const thetaServo = modelServoDegToTheta(modelCalfDeg, calibration.thetaServo);
  const geometry = solveGeometry(thetaThigh, thetaServo);

  if (!geometryWithinJointLimits(geometry, limits)) {
    return null;
  }

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

function refineFootServoCandidate(best, target, limits, calibration, startModelDeg) {
  if (!best) {
    return null;
  }

  for (const step of SERVO_REFINE_STEPS_DEG) {
    let improved = true;

    while (improved) {
      improved = false;
      const neighbors = [
        [best.modelThighDeg + step, best.modelCalfDeg],
        [best.modelThighDeg - step, best.modelCalfDeg],
        [best.modelThighDeg, best.modelCalfDeg + step],
        [best.modelThighDeg, best.modelCalfDeg - step],
        [best.modelThighDeg + step, best.modelCalfDeg + step],
        [best.modelThighDeg + step, best.modelCalfDeg - step],
        [best.modelThighDeg - step, best.modelCalfDeg + step],
        [best.modelThighDeg - step, best.modelCalfDeg - step],
      ];

      for (const [candidateThighDeg, candidateCalfDeg] of neighbors) {
        const candidate = evaluateFootServoCandidate(candidateThighDeg, candidateCalfDeg, target, limits, calibration, startModelDeg);
        if (candidate && footServoCandidateBetter(candidate, best)) {
          best = candidate;
          improved = true;
        }
      }
    }
  }

  return best;
}

function refineJointServoCandidate(best, thetaThigh, targetThetaCalf, limits, calibration, startModelDeg) {
  if (!best) {
    return null;
  }

  for (const step of SERVO_REFINE_STEPS_DEG) {
    let improved = true;

    while (improved) {
      improved = false;

      for (const candidateCalfDeg of [best.modelCalfDeg + step, best.modelCalfDeg - step]) {
        const candidate = evaluateJointServoCandidate(thetaThigh, candidateCalfDeg, targetThetaCalf, limits, calibration, startModelDeg);
        if (candidate && jointServoCandidateBetter(candidate, best)) {
          best = candidate;
          improved = true;
        }
      }
    }
  }

  return best;
}

export function circleIntersections(c0, r0, c1, r1) {
  const dx = c1.x - c0.x;
  const dy = c1.y - c0.y;
  const d = Math.hypot(dx, dy);

  if (d === 0 || d > r0 + r1 || d < Math.abs(r0 - r1)) {
    return [];
  }

  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const hSquared = r0 * r0 - a * a;
  if (hSquared < 0) {
    return [];
  }

  const h = Math.sqrt(hSquared);
  const xm = c0.x + (a * dx) / d;
  const ym = c0.y + (a * dy) / d;
  const rx = (-dy * h) / d;
  const ry = (dx * h) / d;

  return [
    { x: xm + rx, y: ym + ry },
    { x: xm - rx, y: ym - ry }
  ];
}

function chooseBellPoint(points) {
  return points.reduce((best, point) => (best == null || point.y > best.y ? point : best), null);
}

function chooseCalfAttachPoint(points, knee) {
  return points.reduce((best, point) => {
    const score = (knee.y - point.y) * 3 + Math.abs(knee.x - point.x);
    if (best == null || score < best.score) {
      return { point, score };
    }
    return best;
  }, null)?.point ?? null;
}

function makeFallbackGeometry(thetaThigh, thetaServo) {
  const hip = LEG_GEOMETRY.hipPivot;
  const servoPivot = LEG_GEOMETRY.servoPivot;
  const knee = {
    x: hip.x + LEG_GEOMETRY.thighLength * Math.cos(thetaThigh),
    y: hip.y + LEG_GEOMETRY.thighLength * Math.sin(thetaThigh)
  };
  const servoHornEnd = {
    x: servoPivot.x + LEG_GEOMETRY.hornLength * Math.cos(thetaServo),
    y: servoPivot.y + LEG_GEOMETRY.hornLength * Math.sin(thetaServo)
  };

  return {
    hip,
    servoPivot,
    knee,
    servoHornEnd,
    bellArmA: { ...hip },
    bellArmB: { ...hip },
    calfAttach: { ...knee },
    foot: { ...knee },
    thetaThigh,
    thetaServo,
    thetaCalf: thetaThigh,
    valid: false
  };
}

export function solveGeometry(thetaThigh, thetaServo) {
  const g = LEG_GEOMETRY;
  const geometry = makeFallbackGeometry(thetaThigh, thetaServo);
  const hip = geometry.hip;
  const servoPivot = geometry.servoPivot;

  const knee = {
    x: hip.x + g.thighLength * Math.cos(thetaThigh),
    y: hip.y + g.thighLength * Math.sin(thetaThigh)
  };

  const servoHornEnd = {
    x: servoPivot.x + g.hornLength * Math.cos(thetaServo),
    y: servoPivot.y + g.hornLength * Math.sin(thetaServo)
  };
  geometry.knee = knee;
  geometry.servoHornEnd = servoHornEnd;

  const bellCandidates = circleIntersections(hip, g.bellLength, servoHornEnd, g.linkShort);
  const bellArmA = chooseBellPoint(bellCandidates);
  if (!bellArmA) {
    return geometry;
  }

  const bellAngle = Math.atan2(bellArmA.y - hip.y, bellArmA.x - hip.x);
  const bellArmB = {
    x: hip.x + g.bellLength * Math.cos(bellAngle - Math.PI / 2),
    y: hip.y + g.bellLength * Math.sin(bellAngle - Math.PI / 2)
  };
  geometry.bellArmA = bellArmA;
  geometry.bellArmB = bellArmB;

  const calfAttachCandidates = circleIntersections(bellArmB, g.linkLong, knee, g.calfAttachOffset);
  const calfAttach = chooseCalfAttachPoint(calfAttachCandidates, knee);
  if (!calfAttach) {
    return geometry;
  }

  const thetaCalf = Math.atan2(knee.y - calfAttach.y, knee.x - calfAttach.x);
  const foot = {
    x: knee.x + g.calfLength * Math.cos(thetaCalf),
    y: knee.y + g.calfLength * Math.sin(thetaCalf)
  };
  geometry.calfAttach = calfAttach;
  geometry.foot = foot;
  geometry.thetaCalf = thetaCalf;
  geometry.valid = true;
  return geometry;
}

export function solveAnglesForFoot(target, startThigh = -Math.PI / 4, startServo = Math.PI / 6, limits = DEFAULT_JOINT_LIMITS, calibration = null) {
  const activeCalibration = calibration ?? createNeutralCalibration();
  const startModelDeg = {
    thigh: clampValue(thetaToModelServoDeg(startThigh, activeCalibration.thetaThigh), SERVO_MIN_DEG, SERVO_MAX_DEG),
    calf: clampValue(thetaToModelServoDeg(startServo, activeCalibration.thetaServo), SERVO_MIN_DEG, SERVO_MAX_DEG),
  };

  let bestSolution = null;
  for (let modelThighDeg = SERVO_MIN_DEG; modelThighDeg <= SERVO_MAX_DEG; modelThighDeg += SERVO_SEARCH_STEP_DEG) {
    for (let modelCalfDeg = SERVO_MIN_DEG; modelCalfDeg <= SERVO_MAX_DEG; modelCalfDeg += SERVO_SEARCH_STEP_DEG) {
      const candidate = evaluateFootServoCandidate(modelThighDeg, modelCalfDeg, target, limits, activeCalibration, startModelDeg);
      if (candidate && footServoCandidateBetter(candidate, bestSolution)) {
        bestSolution = candidate;
      }
    }
  }

  bestSolution = refineFootServoCandidate(bestSolution, target, limits, activeCalibration, startModelDeg);

  if (!bestSolution) {
    const fallbackGeometry = solveGeometry(startThigh, startServo);
    return {
      thetaThigh: startThigh,
      thetaServo: startServo,
      thetaCalf: fallbackGeometry.thetaCalf,
      footError: Number.POSITIVE_INFINITY,
      geometry: fallbackGeometry,
      hasSolution: false,
      success: false,
    };
  }

  return {
    thetaThigh: bestSolution.thetaThigh,
    thetaServo: bestSolution.thetaServo,
    thetaCalf: bestSolution.thetaCalf,
    footError: bestSolution.error,
    geometry: bestSolution.geometry,
    hasSolution: bestSolution.geometry.valid && Number.isFinite(bestSolution.error),
    success: bestSolution.geometry.valid && Number.isFinite(bestSolution.error) && bestSolution.error < 10,
  };
}

export function solveServoForJointAngles(thetaThigh, targetThetaCalf, startServo = Math.PI / 6, limits = DEFAULT_JOINT_LIMITS, calibration = null) {
  const activeCalibration = calibration ?? createNeutralCalibration();
  const startModelDeg = clampValue(thetaToModelServoDeg(startServo, activeCalibration.thetaServo), SERVO_MIN_DEG, SERVO_MAX_DEG);
  let bestSolution = null;

  for (let modelCalfDeg = SERVO_MIN_DEG; modelCalfDeg <= SERVO_MAX_DEG; modelCalfDeg += SERVO_SEARCH_STEP_DEG) {
    const candidate = evaluateJointServoCandidate(thetaThigh, modelCalfDeg, targetThetaCalf, limits, activeCalibration, startModelDeg);
    if (candidate && jointServoCandidateBetter(candidate, bestSolution)) {
      bestSolution = candidate;
    }
  }

  bestSolution = refineJointServoCandidate(bestSolution, thetaThigh, targetThetaCalf, limits, activeCalibration, startModelDeg);

  if (!bestSolution) {
    const fallbackGeometry = solveGeometry(thetaThigh, startServo);
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

export function createNeutralCalibration() {
  return {
    thetaThigh: NEUTRAL_CALIBRATION.thetaThigh,
    thetaServo: NEUTRAL_CALIBRATION.thetaServo,
  };
}

export function modelServoDegToTheta(modelDeg, neutralTheta) {
  return neutralTheta + degToRad(modelDeg - 90);
}

export function thetaToModelServoDeg(theta, neutralTheta) {
  return 90 + radToDeg(theta - neutralTheta);
}

export function buildLegPoseFromServoAngles(servoAnglesDeg, calibration = createNeutralCalibration(), options = {}) {
  const normalizedServoAngles = normalizeServoAngles(servoAnglesDeg);
  const thetaThigh = modelServoDegToTheta(normalizedServoAngles.thigh, calibration.thetaThigh);
  const thetaServo = modelServoDegToTheta(normalizedServoAngles.calf, calibration.thetaServo);
  const geometry = solveGeometry(thetaThigh, thetaServo);
  const limits = normalizeJointLimits(options.jointLimits);
  const clampedJointAngles = clampJointAnglesToLimits(
    {
      thigh: radToDeg(geometry.thetaThigh),
      calf: radToDeg(geometry.thetaCalf)
    },
    limits
  );

  return {
    geometry,
    foot: {
      x: geometry.foot.x - LEG_GEOMETRY.footOriginOffset.x,
      y: geometry.foot.y - LEG_GEOMETRY.footOriginOffset.y
    },
    jointAnglesDeg: {
      hipYaw: clampValue(normalizedServoAngles.hipYaw - 90, limits.hipYawDeg.min, limits.hipYawDeg.max),
      thigh: clampedJointAngles.thigh,
      calf: clampedJointAngles.calf
    },
    servoAnglesDeg: {
      hipYaw: clampValue(normalizedServoAngles.hipYaw, 90 + limits.hipYawDeg.min, 90 + limits.hipYawDeg.max),
      thigh: normalizedServoAngles.thigh,
      calf: normalizedServoAngles.calf
    },
    reachable: geometry.valid,
    footError: 0
  };
}

export function buildLegPoseFromFoot(foot, calibration = createNeutralCalibration(), options = {}) {
  const limits = normalizeJointLimits(options.jointLimits);
  const hipYawDeg = clampValue(options.hipYawDeg ?? 0, limits.hipYawDeg.min, limits.hipYawDeg.max);
  const solution = solveAnglesForFoot({
    x: foot.x + LEG_GEOMETRY.footOriginOffset.x,
    y: foot.y + LEG_GEOMETRY.footOriginOffset.y
  }, options.startThetaThigh ?? -Math.PI / 4, options.startThetaServo ?? Math.PI / 6, limits, calibration);
  const geometry = solution.geometry;
  const clampedJointAngles = clampJointAnglesToLimits(
    {
      thigh: radToDeg(geometry.thetaThigh),
      calf: radToDeg(geometry.thetaCalf)
    },
    limits
  );

  return {
    geometry,
    foot: {
      x: geometry.foot.x - LEG_GEOMETRY.footOriginOffset.x,
      y: geometry.foot.y - LEG_GEOMETRY.footOriginOffset.y
    },
    jointAnglesDeg: {
      hipYaw: hipYawDeg,
      thigh: clampedJointAngles.thigh,
      calf: clampedJointAngles.calf
    },
    servoAnglesDeg: {
      hipYaw: 90 + hipYawDeg,
      thigh: thetaToModelServoDeg(solution.thetaThigh, calibration.thetaThigh),
      calf: thetaToModelServoDeg(solution.thetaServo, calibration.thetaServo)
    },
    reachable: solution.hasSolution,
    withinTolerance: solution.success,
    footError: solution.footError
  };
}

export function buildLegPoseFromJointAngles(jointAnglesDeg, calibration = createNeutralCalibration(), options = {}) {
  const limits = normalizeJointLimits(options.jointLimits);
  const clamped = clampJointAnglesToLimits(jointAnglesDeg, limits);
  const thetaThigh = degToRad(clamped.thigh);
  const servoSolution = solveServoForJointAngles(thetaThigh, degToRad(clamped.calf), options.startThetaServo ?? Math.PI / 6, limits, calibration);
  const geometry = servoSolution.geometry;

  return {
    geometry,
    foot: {
      x: geometry.foot.x - LEG_GEOMETRY.footOriginOffset.x,
      y: geometry.foot.y - LEG_GEOMETRY.footOriginOffset.y
    },
    jointAnglesDeg: {
      hipYaw: clamped.hipYaw,
      thigh: clamped.thigh,
      calf: clamped.calf
    },
    servoAnglesDeg: {
      hipYaw: 90 + clamped.hipYaw,
      thigh: thetaToModelServoDeg(thetaThigh, calibration.thetaThigh),
      calf: thetaToModelServoDeg(servoSolution.thetaServo, calibration.thetaServo)
    },
    reachable: servoSolution.hasSolution,
    withinTolerance: servoSolution.success,
    footError: 0
  };
}

export function toCanvasPoint(point, drawing = LEG_DRAWING) {
  return {
    x: point.x * drawing.scale + drawing.offset.x,
    y: -point.y * drawing.scale + drawing.offset.y
  };
}

function rotateAroundYAxis(point, pivot, angleRad) {
  const dx = point.x - pivot.x;
  const dz = point.z - pivot.z;
  return {
    x: pivot.x + dx * Math.cos(angleRad) + dz * Math.sin(angleRad),
    y: point.y,
    z: pivot.z - dx * Math.sin(angleRad) + dz * Math.cos(angleRad),
  };
}

function buildSpatialLegPoint(point2d, hipYawDeg) {
  const pivot = LEG_GEOMETRY.jointRotationPoints3d.hipYaw;
  const thighOrigin = LEG_GEOMETRY.jointRotationPoints3d.thigh;
  const local3d = {
    x: thighOrigin.x + point2d.x,
    y: thighOrigin.y + point2d.y,
    z: thighOrigin.z,
  };
  return rotateAroundYAxis(local3d, pivot, degToRad(hipYawDeg));
}

export function robotOverviewGeometry(robotState, poseSource = "desired") {
  return LEG_IDS.map((legId) => {
    const legState = robotState.legs?.[legId]?.[poseSource] ?? robotState.legs?.[legId]?.desired;
    const anchor = ROBOT_LAYOUT.anchors[legId];
    const mirrored = legId.endsWith("right");
    const rearLeg = legId.startsWith("rear");
    const sign = mirrored ? -1 : 1;
    const depthSign = rearLeg ? -1 : 1;

    if (!legState?.geometry) {
      return { legId, anchor, points: [] };
    }

    const hipYawDeg = legState.jointAnglesDeg?.hipYaw ?? 0;
    const points = ["hip", "knee", "foot"].map((key) => {
      const point3d = buildSpatialLegPoint(legState.geometry[key], hipYawDeg);
      return {
        key,
        x: anchor.x + sign * point3d.x * 0.55,
        y: anchor.y + depthSign * point3d.z * 0.3
      };
    });

    return { legId, anchor, points };
  });
}
