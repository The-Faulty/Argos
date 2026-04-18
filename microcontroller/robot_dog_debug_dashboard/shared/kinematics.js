import { DEFAULT_JOINT_LIMITS, LEG_DRAWING, LEG_GEOMETRY, LEG_IDS, ROBOT_LAYOUT } from "./robot-config.js";

function clampValue(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function normalizeJointLimits(limits = DEFAULT_JOINT_LIMITS) {
  const thighMin = Number.isFinite(limits?.thighDeg?.min) ? limits.thighDeg.min : DEFAULT_JOINT_LIMITS.thighDeg.min;
  const thighMax = Number.isFinite(limits?.thighDeg?.max) ? limits.thighDeg.max : DEFAULT_JOINT_LIMITS.thighDeg.max;
  const calfMin = Number.isFinite(limits?.calfDeg?.min) ? limits.calfDeg.min : DEFAULT_JOINT_LIMITS.calfDeg.min;
  const calfMax = Number.isFinite(limits?.calfDeg?.max) ? limits.calfDeg.max : DEFAULT_JOINT_LIMITS.calfDeg.max;
  return {
    thighDeg: {
      min: Math.min(thighMin, thighMax),
      max: Math.max(thighMin, thighMax)
    },
    calfDeg: {
      min: Math.min(calfMin, calfMax),
      max: Math.max(calfMin, calfMax)
    }
  };
}

export function clampJointAnglesToLimits(jointAnglesDeg, limits = DEFAULT_JOINT_LIMITS) {
  const normalized = normalizeJointLimits(limits);
  return {
    thigh: clampValue(jointAnglesDeg.thigh, normalized.thighDeg.min, normalized.thighDeg.max),
    calf: clampValue(jointAnglesDeg.calf, normalized.calfDeg.min, normalized.calfDeg.max)
  };
}

export function geometryWithinJointLimits(geometry, limits = DEFAULT_JOINT_LIMITS) {
  if (!geometry?.valid) {
    return false;
  }
  const normalized = normalizeJointLimits(limits);
  const thighDeg = radToDeg(geometry.thetaThigh);
  const calfDeg = radToDeg(geometry.thetaCalf);
  return (
    thighDeg >= normalized.thighDeg.min &&
    thighDeg <= normalized.thighDeg.max &&
    calfDeg >= normalized.calfDeg.min &&
    calfDeg <= normalized.calfDeg.max
  );
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

export function solveAnglesForFoot(target, startThigh = -Math.PI / 4, startServo = Math.PI / 6, limits = DEFAULT_JOINT_LIMITS) {
  function refineFromSeed(seedThigh, seedServo) {
    let bestThigh = clampAngle(seedThigh);
    let bestServo = clampAngle(seedServo);
    let bestGeometry = solveGeometry(bestThigh, bestServo);
    let bestError = geometryWithinJointLimits(bestGeometry, limits) ? Math.hypot(bestGeometry.foot.x - target.x, bestGeometry.foot.y - target.y) : Number.POSITIVE_INFINITY;
    let stepThigh = 0.2;
    let stepServo = 0.2;

    for (let i = 0; i < 28; i += 1) {
      let improved = false;
      const candidates = [
        [bestThigh + stepThigh, bestServo],
        [bestThigh - stepThigh, bestServo],
        [bestThigh, bestServo + stepServo],
        [bestThigh, bestServo - stepServo],
        [bestThigh + stepThigh, bestServo + stepServo],
        [bestThigh + stepThigh, bestServo - stepServo],
        [bestThigh - stepThigh, bestServo + stepServo],
        [bestThigh - stepThigh, bestServo - stepServo]
      ];

      for (const [candidateThigh, candidateServo] of candidates) {
        const thigh = clampAngle(candidateThigh);
        const servo = clampAngle(candidateServo);
        const geometry = solveGeometry(thigh, servo);
        if (!geometryWithinJointLimits(geometry, limits)) {
          continue;
        }
        const error = Math.hypot(geometry.foot.x - target.x, geometry.foot.y - target.y);

        if (error < bestError) {
          bestError = error;
          bestThigh = thigh;
          bestServo = servo;
          bestGeometry = geometry;
          improved = true;
        }
      }

      if (!improved) {
        stepThigh *= 0.5;
        stepServo *= 0.5;
      }
    }

    return {
      thetaThigh: bestThigh,
      thetaServo: bestServo,
      thetaCalf: bestGeometry.thetaCalf,
      footError: bestError,
      geometry: bestGeometry
    };
  }

  const seedSet = new Map();
  const thighSeeds = [-1.35, -1.1, -0.85, -0.6, -0.35, -0.1];
  const servoSeeds = [-2.85, -2.55, -2.25, -1.95, -1.65, -1.35, -1.05, 0.75];

  function addSeed(thigh, servo) {
    seedSet.set(`${thigh.toFixed(3)}:${servo.toFixed(3)}`, [thigh, servo]);
  }

  addSeed(startThigh, startServo);
  for (const thigh of thighSeeds) {
    for (const servo of servoSeeds) {
      addSeed(thigh, servo);
    }
  }

  let bestSolution = null;
  for (const [seedThigh, seedServo] of seedSet.values()) {
    const candidate = refineFromSeed(seedThigh, seedServo);
    if (!candidate.geometry.valid || !Number.isFinite(candidate.footError)) {
      continue;
    }

    if (bestSolution == null || candidate.footError < bestSolution.footError) {
      bestSolution = candidate;
    }
  }

  if (!bestSolution) {
    bestSolution = refineFromSeed(startThigh, startServo);
  }

  return {
    thetaThigh: bestSolution.thetaThigh,
    thetaServo: bestSolution.thetaServo,
    thetaCalf: bestSolution.thetaCalf,
    footError: bestSolution.footError,
    geometry: bestSolution.geometry,
    hasSolution: bestSolution.geometry.valid && Number.isFinite(bestSolution.footError),
    success: bestSolution.geometry.valid && Number.isFinite(bestSolution.footError) && bestSolution.footError < 10
  };
}

export function solveServoForJointAngles(thetaThigh, targetThetaCalf, startServo = Math.PI / 6, limits = DEFAULT_JOINT_LIMITS) {
  let bestServo = startServo;
  let bestGeometry = solveGeometry(thetaThigh, bestServo);
  let bestError = geometryWithinJointLimits(bestGeometry, limits) ? Math.abs(clampAngle(bestGeometry.thetaCalf - targetThetaCalf)) : Number.POSITIVE_INFINITY;
  let step = 0.2;

  for (let i = 0; i < 28; i += 1) {
    let improved = false;
    for (const candidate of [bestServo + step, bestServo - step]) {
      const servo = clampAngle(candidate);
      const geometry = solveGeometry(thetaThigh, servo);
      if (!geometryWithinJointLimits(geometry, limits)) {
        continue;
      }
      const error = Math.abs(clampAngle(geometry.thetaCalf - targetThetaCalf));
      if (error < bestError) {
        bestServo = servo;
        bestGeometry = geometry;
        bestError = error;
        improved = true;
      }
    }

    if (!improved) {
      step *= 0.5;
    }
  }

  return {
    thetaServo: bestServo,
    thetaCalf: bestGeometry.thetaCalf,
    jointError: bestError,
    geometry: bestGeometry,
    hasSolution: bestGeometry.valid && Number.isFinite(bestError),
    success: bestGeometry.valid && Number.isFinite(bestError) && bestError < degToRad(3)
  };
}

export function createNeutralCalibration() {
  const solution = solveAnglesForFoot(LEG_GEOMETRY.footOriginOffset, -Math.PI / 4, Math.PI / 6, DEFAULT_JOINT_LIMITS);
  return {
    thetaThigh: solution.thetaThigh,
    thetaServo: solution.thetaServo
  };
}

export function modelServoDegToTheta(modelDeg, neutralTheta) {
  return neutralTheta + degToRad(modelDeg - 90);
}

export function thetaToModelServoDeg(theta, neutralTheta) {
  return 90 + radToDeg(theta - neutralTheta);
}

export function buildLegPoseFromServoAngles(servoAnglesDeg, calibration = createNeutralCalibration(), options = {}) {
  const thetaThigh = modelServoDegToTheta(servoAnglesDeg.thigh, calibration.thetaThigh);
  const thetaServo = modelServoDegToTheta(servoAnglesDeg.calf, calibration.thetaServo);
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
      thigh: clampedJointAngles.thigh,
      calf: clampedJointAngles.calf
    },
    servoAnglesDeg: { ...servoAnglesDeg },
    reachable: geometry.valid,
    footError: 0
  };
}

export function buildLegPoseFromFoot(foot, calibration = createNeutralCalibration(), options = {}) {
  const limits = normalizeJointLimits(options.jointLimits);
  const solution = solveAnglesForFoot({
    x: foot.x + LEG_GEOMETRY.footOriginOffset.x,
    y: foot.y + LEG_GEOMETRY.footOriginOffset.y
  }, options.startThetaThigh ?? -Math.PI / 4, options.startThetaServo ?? Math.PI / 6, limits);
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
      thigh: clampedJointAngles.thigh,
      calf: clampedJointAngles.calf
    },
    servoAnglesDeg: {
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
  const servoSolution = solveServoForJointAngles(thetaThigh, degToRad(clamped.calf), options.startThetaServo ?? Math.PI / 6, limits);
  const geometry = servoSolution.geometry;

  return {
    geometry,
    foot: {
      x: geometry.foot.x - LEG_GEOMETRY.footOriginOffset.x,
      y: geometry.foot.y - LEG_GEOMETRY.footOriginOffset.y
    },
    jointAnglesDeg: {
      thigh: clamped.thigh,
      calf: clamped.calf
    },
    servoAnglesDeg: {
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

    const points = ["hip", "knee", "foot"].map((key) => {
      const point = legState.geometry[key];
      return {
        key,
        x: anchor.x + sign * point.x * 0.55,
        y: anchor.y + depthSign * point.y * 0.3
      };
    });

    return { legId, anchor, points };
  });
}
