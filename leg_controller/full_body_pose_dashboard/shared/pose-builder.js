import {
  buildLegPoseFromFoot,
  createNeutralCalibration,
  degToRad,
  normalizeJointLimits,
  radToDeg,
} from "../../robot_dog_debug_dashboard/shared/kinematics.js";
import { createFullBodyPoseCommand } from "../../robot_dog_debug_dashboard/shared/locomotion.js";
import {
  DEFAULT_JOINT_LIMITS,
  DEFAULT_LEG_COMMAND,
  DEFAULT_SERVO_TRIM_DEG,
  LEG_GEOMETRY,
  LEG_IDS,
} from "../../robot_dog_debug_dashboard/shared/robot-config.js";

export const POSE_SCHEMA_VERSION = 1;
export const POSE_LIBRARY_STORAGE_KEY = "argos.poseBuilder.poses.v1";

export const LEG_PREFIXES = {
  front_left: "FL",
  front_right: "FR",
  rear_left: "RL",
  rear_right: "RR",
};

export const LEG_COXA_PIVOTS_MM = {
  front_left: { x: -16.983, y: -73.583, z: 14.583 },
  front_right: { x: -16.983, y: 55.217, z: 14.583 },
  rear_left: { x: 131.617, y: -73.583, z: 14.583 },
  rear_right: { x: 131.617, y: 55.217, z: 14.583 },
};

export const LEG_THIGH_PIVOTS_MM = {
  front_left: { x: -41.083, y: -111.383, z: 27.133 },
  front_right: { x: -41.083, y: 93.017, z: 27.133 },
  rear_left: { x: 176.517, y: -111.383, z: 27.133 },
  rear_right: { x: 176.517, y: 93.017, z: 27.133 },
};

export const LEG_MOUNTS_MM = LEG_THIGH_PIVOTS_MM;

const COXA_AXIS_SIGNS = {
  front_left: 1,
  front_right: 1,
  rear_left: 1,
  rear_right: -1,
};

const FEMUR_AXIS_SIGNS = {
  front_left: 1,
  front_right: -1,
  rear_left: 1,
  rear_right: 1,
};

const TIBIA_AXIS_SIGNS = {
  front_left: 1,
  front_right: -1,
  rear_left: 1,
  rear_right: 1,
};

function averageVisualMap(a, b) {
  return {
    offset: (a.offset + b.offset) / 2,
    scale: (a.scale + b.scale) / 2,
  };
}

const FRONT_THIGH_VISUAL_MAP_DEG = averageVisualMap(
  { offset: -10.966, scale: -0.55 },
  { offset: -11.259, scale: -0.559 },
);
const REAR_THIGH_VISUAL_MAP_DEG = averageVisualMap(
  { offset: -21.063, scale: -0.675 },
  { offset: -21.626, scale: -0.686 },
);

// Visual angle maps are calibrated against the v4 CAD meshes so moved feet land
// on the same static targets used by the pose solver.
const URDF_THIGH_VISUAL_MAP_DEG = {
  front_left: FRONT_THIGH_VISUAL_MAP_DEG,
  front_right: FRONT_THIGH_VISUAL_MAP_DEG,
  rear_left: REAR_THIGH_VISUAL_MAP_DEG,
  rear_right: REAR_THIGH_VISUAL_MAP_DEG,
};

const FRONT_TIBIA_VISUAL_MAP_DEG = averageVisualMap(
  { offset: -152.59, scale: -1 },
  { offset: -152.901, scale: -1 },
);
const REAR_TIBIA_VISUAL_MAP_DEG = averageVisualMap(
  { offset: -149.089, scale: -1.001 },
  { offset: -148.513, scale: -1 },
);

const URDF_TIBIA_VISUAL_MAP_DEG = {
  front_left: FRONT_TIBIA_VISUAL_MAP_DEG,
  front_right: FRONT_TIBIA_VISUAL_MAP_DEG,
  rear_left: REAR_TIBIA_VISUAL_MAP_DEG,
  rear_right: REAR_TIBIA_VISUAL_MAP_DEG,
};

const VISUAL_IK_EXTENSION_EPSILON_MM = 0.001;
const URDF_FIT_BENT_MAX_CALF_DEG = -118;
const URDF_FIT_EXTENDED_MIN_CALF_DEG = -96;
const URDF_FIT_WEIGHTS_BENT = {
  femur: 20,
  tibia: 800,
};
const URDF_FIT_WEIGHTS_EXTENDED = {
  femur: 220,
  tibia: 9000,
};
const URDF_FIT_WEIGHTS_BLEND = {
  femur: (URDF_FIT_WEIGHTS_BENT.femur + URDF_FIT_WEIGHTS_EXTENDED.femur) / 2,
  tibia: (URDF_FIT_WEIGHTS_BENT.tibia + URDF_FIT_WEIGHTS_EXTENDED.tibia) / 2,
};

const URDF_VISUAL_CHAINS_MM = {
  front_left: {
    coxaOrigin: { x: -16.983, y: -73.583, z: 14.583 },
    coxaAxis: { x: 1, y: 0, z: 0 },
    femurOrigin: { x: -24.1, y: -37.8, z: 12.55 },
    femurAxis: { x: 0, y: 1, z: 0 },
    tibiaOrigin: { x: 105.946, y: 0.641, z: -70.032 },
    tibiaAxis: { x: 0, y: 1, z: 0 },
    footPoint: { x: -95.4, y: -15.7, z: -90.7 },
  },
  front_right: {
    coxaOrigin: { x: -16.983, y: 55.217, z: 14.583 },
    coxaAxis: { x: 1, y: 0, z: 0 },
    femurOrigin: { x: -24.1, y: 37.8, z: 12.55 },
    femurAxis: { x: 0, y: -1, z: 0 },
    tibiaOrigin: { x: 105.932, y: -0.641, z: -70.053 },
    tibiaAxis: { x: 0, y: -1, z: 0 },
    footPoint: { x: -96, y: 15.7, z: -90 },
  },
  rear_left: {
    coxaOrigin: { x: 131.617, y: -73.583, z: 14.583 },
    coxaAxis: { x: 1, y: 0, z: 0 },
    femurOrigin: { x: 44.9, y: -37.8, z: 12.55 },
    femurAxis: { x: 0, y: 1, z: 0 },
    tibiaOrigin: { x: 99.021, y: 0.641, z: -79.522 },
    tibiaAxis: { x: 0, y: 1, z: 0 },
    footPoint: { x: -98.9, y: -15.7, z: -86.8 },
  },
  rear_right: {
    coxaOrigin: { x: 131.617, y: 55.217, z: 14.583 },
    coxaAxis: { x: -1, y: 0, z: 0 },
    femurOrigin: { x: 44.9, y: 37.8, z: 12.55 },
    femurAxis: { x: 0, y: 1, z: 0 },
    tibiaOrigin: { x: 98.904, y: -0.641, z: -79.668 },
    tibiaAxis: { x: 0, y: 1, z: 0 },
    footPoint: { x: -98.2, y: 15.7, z: -87.6 },
  },
};

const calibration = createNeutralCalibration();
const DEFAULT_BODY = {
  positionMm: { x: 0, y: 0, z: 0 },
  rotationDeg: { roll: 0, pitch: 0, yaw: 0 },
};

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function numeric(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function inverseLerp(start, end, value) {
  if (start === end) {
    return 0;
  }
  return clamp((value - start) / (end - start), 0, 1);
}

function sanitizePoseName(name, fallback = "untitled-pose") {
  return typeof name === "string" && name.trim() ? name.trim() : fallback;
}

function normalizeVector3(value, fallback = { x: 0, y: 0, z: 0 }) {
  return {
    x: numeric(value?.x, fallback.x),
    y: numeric(value?.y, fallback.y),
    z: numeric(value?.z, fallback.z),
  };
}

function normalizeBody(body = {}, fallback = DEFAULT_BODY) {
  return {
    positionMm: normalizeVector3(body.positionMm, fallback.positionMm),
    rotationDeg: {
      roll: numeric(body.rotationDeg?.roll, fallback.rotationDeg.roll),
      pitch: numeric(body.rotationDeg?.pitch, fallback.rotationDeg.pitch),
      yaw: numeric(body.rotationDeg?.yaw, fallback.rotationDeg.yaw),
    },
  };
}

function subtractVector3(a, b) {
  return {
    x: a.x - b.x,
    y: a.y - b.y,
    z: a.z - b.z,
  };
}

function addVector3(a, b) {
  return {
    x: a.x + b.x,
    y: a.y + b.y,
    z: a.z + b.z,
  };
}

function dotVector3(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function crossVector3(a, b) {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}

function normalizeAxis(axis) {
  const length = Math.hypot(axis.x, axis.y, axis.z) || 1;
  return {
    x: axis.x / length,
    y: axis.y / length,
    z: axis.z / length,
  };
}

function rotateVectorAroundAxis(vector, axis, angleRad) {
  const unit = normalizeAxis(axis);
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  const cross = crossVector3(unit, vector);
  const dot = dotVector3(unit, vector);

  return {
    x: vector.x * cos + cross.x * sin + unit.x * dot * (1 - cos),
    y: vector.y * cos + cross.y * sin + unit.y * dot * (1 - cos),
    z: vector.z * cos + cross.z * sin + unit.z * dot * (1 - cos),
  };
}

function multiplyMatrixVector(matrix, vector) {
  return {
    x: matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z,
    y: matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z,
    z: matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z,
  };
}

function transposeMatrix(matrix) {
  return [
    [matrix[0][0], matrix[1][0], matrix[2][0]],
    [matrix[0][1], matrix[1][1], matrix[2][1]],
    [matrix[0][2], matrix[1][2], matrix[2][2]],
  ];
}

export function createNeutralFeetWorld() {
  const absoluteFoot = {
    x: DEFAULT_LEG_COMMAND.foot.x + LEG_GEOMETRY.footOriginOffset.x,
    z: DEFAULT_LEG_COMMAND.foot.y + LEG_GEOMETRY.footOriginOffset.y,
  };

  return Object.fromEntries(
    LEG_IDS.map((legId) => [
      legId,
      {
        x: LEG_MOUNTS_MM[legId].x + absoluteFoot.x,
        y: LEG_MOUNTS_MM[legId].y,
        z: LEG_MOUNTS_MM[legId].z + absoluteFoot.z,
      },
    ]),
  );
}

export function createNeutralStaticPose(name = "neutral-stand") {
  return {
    version: POSE_SCHEMA_VERSION,
    name: sanitizePoseName(name, "neutral-stand"),
    body: clone(DEFAULT_BODY),
    feetWorldMm: createNeutralFeetWorld(),
  };
}

export function normalizeStaticPose(candidate = {}, fallback = createNeutralStaticPose()) {
  const fallbackPose = normalizeFallbackPose(fallback);
  const feetWorldMm = {};

  for (const legId of LEG_IDS) {
    feetWorldMm[legId] = normalizeVector3(candidate.feetWorldMm?.[legId], fallbackPose.feetWorldMm[legId]);
  }

  return {
    version: POSE_SCHEMA_VERSION,
    name: sanitizePoseName(candidate.name, fallbackPose.name),
    body: normalizeBody(candidate.body, fallbackPose.body),
    feetWorldMm,
  };
}

function normalizeFallbackPose(fallback) {
  if (!fallback || fallback.version !== POSE_SCHEMA_VERSION || !fallback.feetWorldMm) {
    return createNeutralStaticPose();
  }

  return {
    version: POSE_SCHEMA_VERSION,
    name: sanitizePoseName(fallback.name, "neutral-stand"),
    body: normalizeBody(fallback.body, DEFAULT_BODY),
    feetWorldMm: Object.fromEntries(
      LEG_IDS.map((legId) => [
        legId,
        normalizeVector3(fallback.feetWorldMm?.[legId], createNeutralFeetWorld()[legId]),
      ]),
    ),
  };
}

export function serializeStaticPose(pose) {
  return JSON.stringify(normalizeStaticPose(pose), null, 2);
}

export function parseStaticPose(json, fallback = createNeutralStaticPose()) {
  return normalizeStaticPose(JSON.parse(json), fallback);
}

export function rotationMatrixFromBody(body) {
  const roll = degToRad(body.rotationDeg?.roll ?? 0);
  const pitch = degToRad(body.rotationDeg?.pitch ?? 0);
  const yaw = degToRad(body.rotationDeg?.yaw ?? 0);
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);

  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

export function bodyToWorldPoint(pointBodyMm, body) {
  return addVector3(
    multiplyMatrixVector(rotationMatrixFromBody(body), pointBodyMm),
    normalizeVector3(body.positionMm),
  );
}

export function worldToBodyPoint(pointWorldMm, body) {
  const shifted = subtractVector3(pointWorldMm, normalizeVector3(body.positionMm));
  return multiplyMatrixVector(transposeMatrix(rotationMatrixFromBody(body)), shifted);
}

export function legPlaneFootToWorld(pose, legId, footCommand, hipYawDeg = 0) {
  const normalized = normalizeStaticPose(pose);
  const mountBodyMm = LEG_MOUNTS_MM[legId] ?? { x: 0, y: 0, z: 0 };
  const verticalReachMm = numeric(footCommand?.y, 0) + LEG_GEOMETRY.footOriginOffset.y;
  const radialReachMm = Math.max(0, -verticalReachMm);
  const hipYawRad = degToRad(numeric(hipYawDeg, 0));
  const footBodyMm = addVector3(mountBodyMm, {
    x: numeric(footCommand?.x, 0) + LEG_GEOMETRY.footOriginOffset.x,
    y: radialReachMm * Math.sin(hipYawRad),
    z: -radialReachMm * Math.cos(hipYawRad),
  });

  return bodyToWorldPoint(footBodyMm, normalized.body);
}

export function updateBodyKeepingFeetLocked(pose, bodyPatch) {
  const normalized = normalizeStaticPose(pose);
  return normalizeStaticPose(
    {
      ...normalized,
      body: {
        positionMm: {
          ...normalized.body.positionMm,
          ...(bodyPatch.positionMm ?? {}),
        },
        rotationDeg: {
          ...normalized.body.rotationDeg,
          ...(bodyPatch.rotationDeg ?? {}),
        },
      },
      feetWorldMm: normalized.feetWorldMm,
    },
    normalized,
  );
}

export function updateFootWorld(pose, legId, footWorldMm) {
  const normalized = normalizeStaticPose(pose);
  return normalizeStaticPose(
    {
      ...normalized,
      feetWorldMm: {
        ...normalized.feetWorldMm,
        [legId]: {
          ...normalized.feetWorldMm[legId],
          ...footWorldMm,
        },
      },
    },
    normalized,
  );
}

export function getJointLimitsByLeg(robotState = {}) {
  return Object.fromEntries(
    LEG_IDS.map((legId) => [
      legId,
      normalizeJointLimits(robotState.legs?.[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS),
    ]),
  );
}

export function solveStaticPose(pose, options = {}) {
  const normalized = normalizeStaticPose(pose);
  const jointLimitsByLeg = options.jointLimitsByLeg ?? {};
  const previousSolutions = options.previousSolutions?.legs ?? {};
  const legs = {};
  let allReachable = true;

  for (const legId of LEG_IDS) {
    const limits = normalizeJointLimits(jointLimitsByLeg[legId] ?? DEFAULT_JOINT_LIMITS);
    const mountBodyMm = LEG_MOUNTS_MM[legId];
    const footBodyMm = worldToBodyPoint(normalized.feetWorldMm[legId], normalized.body);
    const deltaBodyMm = subtractVector3(footBodyMm, mountBodyMm);
    const verticalReachMm = -Math.hypot(deltaBodyMm.y, deltaBodyMm.z);
    const targetHipYawDeg = radToDeg(Math.atan2(deltaBodyMm.y, -deltaBodyMm.z));
    const hipYawDeg = clamp(targetHipYawDeg, limits.hipYawDeg.min, limits.hipYawDeg.max);
    const hipYawWithinLimits = Math.abs(targetHipYawDeg - hipYawDeg) < 0.001;
    const previousDesired = previousSolutions[legId]?.desired ?? previousSolutions[legId];
    const footCommand = {
      x: deltaBodyMm.x - LEG_GEOMETRY.footOriginOffset.x,
      y: verticalReachMm - LEG_GEOMETRY.footOriginOffset.y,
    };
    const desired = buildLegPoseFromFoot(footCommand, calibration, {
      startThetaThigh: previousDesired?.geometry?.thetaThigh,
      startThetaServo: previousDesired?.geometry?.thetaServo,
      jointLimits: limits,
      hipYawDeg,
    });
    const preview = solveVisualLegPlanePose({
      desired,
      footCommand,
      targetHipYawDeg,
    });
    const withinTolerance = desired.withinTolerance ?? desired.reachable;
    const reachable = Boolean(desired.reachable && withinTolerance && hipYawWithinLimits);
    allReachable = allReachable && reachable;

    legs[legId] = {
      desired,
      preview,
      footWorldMm: normalized.feetWorldMm[legId],
      footBodyMm,
      deltaBodyMm,
      footCommand,
      targetHipYawDeg,
      hipYawWithinLimits,
      reachable,
      footErrorMm: desired.footError ?? 0,
      jointLimits: limits,
      status: reachable ? "reachable" : "limited",
    };
  }

  return {
    pose: normalized,
    legs,
    reachable: allReachable,
  };
}

export function createPoseApplyCommand(solvedPose, robotState = {}) {
  const legs = {};

  for (const legId of LEG_IDS) {
    legs[legId] = {
      desired: solvedPose.legs[legId].desired,
      servoTrimDeg: robotState.legs?.[legId]?.servoTrimDeg ?? DEFAULT_SERVO_TRIM_DEG,
    };
  }

  return createFullBodyPoseCommand(legs);
}

function urdfVisualPoint(chain, jointValues) {
  const tibiaLocal = rotateVectorAroundAxis(chain.footPoint, chain.tibiaAxis, jointValues.tibia);
  const tibiaFrame = addVector3(chain.tibiaOrigin, tibiaLocal);
  const femurLocal = rotateVectorAroundAxis(tibiaFrame, chain.femurAxis, jointValues.femur);
  const femurFrame = addVector3(chain.femurOrigin, femurLocal);
  const coxaLocal = rotateVectorAroundAxis(femurFrame, chain.coxaAxis, jointValues.coxa);
  return addVector3(chain.coxaOrigin, coxaLocal);
}

function squaredDistance(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

function angleDistance(a, b) {
  return Math.atan2(Math.sin(a - b), Math.cos(a - b));
}

function clampJointValue(value) {
  return clamp(value, -Math.PI, Math.PI);
}

function solveVisualLegPlanePose(solvedLeg) {
  const fallbackGeometry = solvedLeg.desired.geometry;
  const fallbackAngles = solvedLeg.desired.jointAnglesDeg;
  const hip = fallbackGeometry?.hip ?? LEG_GEOMETRY.hipPivot;
  const servoPivot = fallbackGeometry?.servoPivot ?? LEG_GEOMETRY.servoPivot;
  const target = {
    x: hip.x + numeric(solvedLeg.footCommand?.x, 0) + LEG_GEOMETRY.footOriginOffset.x,
    y: hip.y + numeric(solvedLeg.footCommand?.y, 0) + LEG_GEOMETRY.footOriginOffset.y,
  };
  const thighLength = LEG_GEOMETRY.thighLength;
  const calfLength = LEG_GEOMETRY.calfLength;
  const dx = target.x - hip.x;
  const dy = target.y - hip.y;
  const distance = Math.hypot(dx, dy);

  if (distance < VISUAL_IK_EXTENSION_EPSILON_MM) {
    return {
      geometry: fallbackGeometry,
      jointAnglesDeg: fallbackAngles,
      clipped: false,
    };
  }

  const minReach = Math.abs(thighLength - calfLength) + VISUAL_IK_EXTENSION_EPSILON_MM;
  const maxReach = thighLength + calfLength - VISUAL_IK_EXTENSION_EPSILON_MM;
  const clampedDistance = clamp(distance, minReach, maxReach);
  const scale = clampedDistance / distance;
  const foot = {
    x: hip.x + dx * scale,
    y: hip.y + dy * scale,
  };
  const targetAngle = Math.atan2(foot.y - hip.y, foot.x - hip.x);
  const thighOffset = Math.acos(
    clamp(
      (thighLength * thighLength + clampedDistance * clampedDistance - calfLength * calfLength) /
        (2 * thighLength * clampedDistance),
      -1,
      1,
    ),
  );
  const thetaThigh = targetAngle + thighOffset;
  const knee = {
    x: hip.x + thighLength * Math.cos(thetaThigh),
    y: hip.y + thighLength * Math.sin(thetaThigh),
  };
  const thetaCalf = Math.atan2(foot.y - knee.y, foot.x - knee.x);

  return {
    geometry: {
      ...fallbackGeometry,
      hip,
      servoPivot,
      knee,
      foot,
      thetaThigh,
      thetaCalf,
      valid: true,
      previewClipped: Math.abs(clampedDistance - distance) > 1e-6,
    },
    jointAnglesDeg: {
      hipYaw: numeric(solvedLeg.targetHipYawDeg, fallbackAngles.hipYaw ?? 0),
      thigh: radToDeg(thetaThigh),
      calf: radToDeg(thetaCalf),
    },
    clipped: Math.abs(clampedDistance - distance) > 1e-6,
  };
}

function projectChainAngleDeg(vector) {
  return radToDeg(Math.atan2(vector.z, vector.x));
}

function getUrdfNeutralCalibration(legId) {
  const chain = URDF_VISUAL_CHAINS_MM[legId];
  const femurAxisSign = FEMUR_AXIS_SIGNS[legId] ?? 1;
  const tibiaAxisSign = TIBIA_AXIS_SIGNS[legId] ?? 1;
  const thighMap = URDF_THIGH_VISUAL_MAP_DEG[legId] ?? { offset: 0, scale: -1 };
  const tibiaMap = URDF_TIBIA_VISUAL_MAP_DEG[legId] ?? { offset: 0, scale: -1 };
  const neutralAngles = DEFAULT_LEG_COMMAND.jointAnglesDeg;
  const neutralFemurDeg = (thighMap.offset + thighMap.scale * neutralAngles.thigh) / femurAxisSign;
  const neutralTibiaDeg = (tibiaMap.offset + tibiaMap.scale * neutralAngles.calf) / tibiaAxisSign;
  const femurRestDeg = projectChainAngleDeg(chain.tibiaOrigin);
  const tibiaRestDeg = projectChainAngleDeg(chain.footPoint);
  const femurOffsetDeg = neutralFemurDeg - (femurRestDeg - neutralAngles.thigh) / femurAxisSign;
  const tibiaOffsetDeg = neutralTibiaDeg
    - (tibiaRestDeg - femurAxisSign * neutralFemurDeg - neutralAngles.calf) / tibiaAxisSign;

  return {
    femurRestDeg,
    tibiaRestDeg,
    femurOffsetDeg,
    tibiaOffsetDeg,
  };
}

function getUrdfFitBlendAmount(previewCalfDeg) {
  return inverseLerp(
    URDF_FIT_BENT_MAX_CALF_DEG,
    URDF_FIT_EXTENDED_MIN_CALF_DEG,
    previewCalfDeg,
  );
}

function blendJointValues(from, to, blend) {
  return {
    coxa: clampJointValue(from.coxa + angleDistance(to.coxa, from.coxa) * blend),
    femur: clampJointValue(from.femur + angleDistance(to.femur, from.femur) * blend),
    tibia: clampJointValue(from.tibia + angleDistance(to.tibia, from.tibia) * blend),
  };
}

function urdfVisualFootErrorMm(chain, jointValues, target) {
  return Math.sqrt(squaredDistance(urdfVisualPoint(chain, jointValues), target));
}

function urdfVisualObjective(chain, jointValues, target, initialValues, fitWeights) {
  const footError = squaredDistance(urdfVisualPoint(chain, jointValues), target);
  const femurError = angleDistance(jointValues.femur, initialValues.femur);
  const tibiaError = angleDistance(jointValues.tibia, initialValues.tibia);

  return (
    footError
    + fitWeights.femur * femurError * femurError
    + fitWeights.tibia * tibiaError * tibiaError
  );
}

function optimizeUrdfVisualJoints(chain, target, initialValues, fitWeights, steps = null) {
  let best = {
    coxa: initialValues.coxa,
    femur: initialValues.femur,
    tibia: initialValues.tibia,
  };
  let bestError = urdfVisualObjective(chain, best, target, initialValues, fitWeights);
  const searchSteps = steps ?? [
    degToRad(7),
    degToRad(3.5),
    degToRad(1.5),
    degToRad(0.65),
    degToRad(0.25),
  ];

  for (const step of searchSteps) {
    let improved = true;
    while (improved) {
      improved = false;
      const candidates = [
        { coxa: best.coxa - step, femur: best.femur, tibia: best.tibia },
        { coxa: best.coxa + step, femur: best.femur, tibia: best.tibia },
        { coxa: best.coxa, femur: best.femur - step, tibia: best.tibia },
        { coxa: best.coxa, femur: best.femur + step, tibia: best.tibia },
        { coxa: best.coxa, femur: best.femur, tibia: best.tibia - step },
        { coxa: best.coxa, femur: best.femur, tibia: best.tibia + step },
        { coxa: best.coxa, femur: best.femur - step, tibia: best.tibia + step },
        { coxa: best.coxa, femur: best.femur + step, tibia: best.tibia - step },
      ];

      for (const candidate of candidates) {
        const clamped = {
          coxa: clampJointValue(candidate.coxa),
          femur: clampJointValue(candidate.femur),
          tibia: clampJointValue(candidate.tibia),
        };
        const error = urdfVisualObjective(chain, clamped, target, initialValues, fitWeights);
        if (error < bestError - 1e-6) {
          best = clamped;
          bestError = error;
          improved = true;
        }
      }
    }
  }

  return {
    joints: best,
    objective: bestError,
  };
}

function solveUrdfVisualJoints(legId, solvedLeg, initialValues) {
  const chain = URDF_VISUAL_CHAINS_MM[legId];
  if (!chain) {
    return {
      joints: initialValues,
      mode: "extended",
      errors: {
        bent: 0,
        extended: 0,
        final: 0,
      },
    };
  }

  const target = solvedLeg.footBodyMm;
  const previewCalfDeg = solvedLeg.preview?.jointAnglesDeg?.calf
    ?? solvedLeg.desired.jointAnglesDeg.calf
    ?? -135;
  const bentCandidate = optimizeUrdfVisualJoints(
    chain,
    target,
    initialValues,
    URDF_FIT_WEIGHTS_BENT,
  );
  const extendedCandidate = optimizeUrdfVisualJoints(
    chain,
    target,
    initialValues,
    URDF_FIT_WEIGHTS_EXTENDED,
  );
  const blend = getUrdfFitBlendAmount(previewCalfDeg);
  const bentErrorMm = urdfVisualFootErrorMm(chain, bentCandidate.joints, target);
  const extendedErrorMm = urdfVisualFootErrorMm(chain, extendedCandidate.joints, target);

  if (blend <= 0) {
    return {
      joints: bentCandidate.joints,
      mode: "bent",
      errors: {
        bent: bentErrorMm,
        extended: extendedErrorMm,
        final: bentErrorMm,
      },
    };
  }

  if (blend >= 1) {
    return {
      joints: extendedCandidate.joints,
      mode: "extended",
      errors: {
        bent: bentErrorMm,
        extended: extendedErrorMm,
        final: extendedErrorMm,
      },
    };
  }

  const blendedSeed = blendJointValues(bentCandidate.joints, extendedCandidate.joints, blend);
  const blendedRefinement = optimizeUrdfVisualJoints(
    chain,
    target,
    blendedSeed,
    URDF_FIT_WEIGHTS_BLEND,
    [
      degToRad(3.5),
      degToRad(1.5),
      degToRad(0.65),
      degToRad(0.25),
    ],
  );
  const blendedSeedErrorMm = urdfVisualFootErrorMm(chain, blendedSeed, target);
  const refinedErrorMm = urdfVisualFootErrorMm(chain, blendedRefinement.joints, target);
  let finalJoints = blendedSeed;
  let finalErrorMm = blendedSeedErrorMm;

  for (const candidate of [
    { joints: blendedRefinement.joints, errorMm: refinedErrorMm },
    { joints: bentCandidate.joints, errorMm: bentErrorMm },
    { joints: extendedCandidate.joints, errorMm: extendedErrorMm },
  ]) {
    if (candidate.errorMm < finalErrorMm) {
      finalJoints = candidate.joints;
      finalErrorMm = candidate.errorMm;
    }
  }

  return {
    joints: finalJoints,
    mode: "blended",
    errors: {
      bent: bentErrorMm,
      extended: extendedErrorMm,
      final: finalErrorMm,
    },
  };
}

export function getUrdfJointValues(solvedPose) {
  const values = {};

  for (const legId of LEG_IDS) {
    const prefix = LEG_PREFIXES[legId];
    const solvedLeg = solvedPose.legs[legId];
    const angles = solvedLeg.preview?.jointAnglesDeg ?? solvedLeg.desired.jointAnglesDeg;
    const coxaAxisSign = COXA_AXIS_SIGNS[legId] ?? 1;
    const femurAxisSign = FEMUR_AXIS_SIGNS[legId] ?? 1;
    const tibiaAxisSign = TIBIA_AXIS_SIGNS[legId] ?? 1;
    const neutralCalibration = getUrdfNeutralCalibration(legId);
    const femurJointDeg = (
      (neutralCalibration.femurRestDeg - (angles.thigh ?? 0)) / femurAxisSign
    ) + neutralCalibration.femurOffsetDeg;
    const tibiaJointDeg = (
      (neutralCalibration.tibiaRestDeg - femurAxisSign * femurJointDeg - (angles.calf ?? 0))
      / tibiaAxisSign
    ) + neutralCalibration.tibiaOffsetDeg;
    const initialVisualJoints = {
      coxa: degToRad(coxaAxisSign * (angles.hipYaw ?? 0)),
      femur: degToRad(femurJointDeg),
      tibia: degToRad(tibiaJointDeg),
    };
    const visualFit = solveUrdfVisualJoints(legId, solvedLeg, initialVisualJoints);

    solvedLeg.visualFitMode = visualFit.mode;
    solvedLeg.visualFitErrorMm = visualFit.errors;

    values[`${prefix}_coxa_joint`] = visualFit.joints.coxa;
    values[`${prefix}_femur_joint`] = visualFit.joints.femur;
    values[`${prefix}_tibia_joint`] = visualFit.joints.tibia;
  }

  return values;
}
