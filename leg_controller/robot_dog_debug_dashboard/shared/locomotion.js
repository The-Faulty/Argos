import { buildLegPoseFromFoot, createNeutralCalibration } from "./kinematics.js";
import {
  DEFAULT_DRIVE_COMMAND,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_STANCE,
  LEG_IDS,
} from "./robot-config.js";
import { normalizeDriveCommand } from "./protocol.js";

const calibration = createNeutralCalibration();
const PHASE_OFFSETS = {
  front_left: 0,
  front_right: 0.5,
  rear_left: 0.5,
  rear_right: 0,
};
const RIGHT_LEG_IDS = new Set(["front_right", "rear_right"]);

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function roundServo(value) {
  return Math.round((value ?? 90) * 100) / 100;
}

function mirrorServoAroundNeutral(value) {
  return 180 - (value ?? 90);
}

function clampServo(value) {
  return clamp(value, 0, 180);
}

export function createMotionStatePatch({
  driveCommand = DEFAULT_DRIVE_COMMAND,
  motionMode = "idle",
  timeMs = 0,
  stance = DEFAULT_STANCE,
  jointLimitsByLeg = {},
} = {}) {
  const normalizedDrive = normalizeDriveCommand(driveCommand);
  const strideMagnitude = Math.max(Math.abs(normalizedDrive.vx), Math.abs(normalizedDrive.vy), Math.abs(normalizedDrive.yawRate));
  const footHeightOffset = -(stance.height ?? 0);
  const cycleTimeSec = 1.8 - strideMagnitude * 0.35;
  const timeSec = Math.max(0, timeMs) / 1000;
  const legs = {};

  for (const legId of LEG_IDS) {
    const phaseOffset = PHASE_OFFSETS[legId];
    const cycle = cycleTimeSec > 0 ? ((timeSec / cycleTimeSec) + phaseOffset) % 1 : phaseOffset;
    const swing = cycle < 0.5;
    const phase = swing ? cycle / 0.5 : (cycle - 0.5) / 0.5;
    const strideX = normalizedDrive.vx * 34 * (stance.strideScale ?? 1);
    const strideY = normalizedDrive.vy * 16 * (stance.strideScale ?? 1);
    const liftY = strideMagnitude > 0.01 ? 16 : 0;
    const rotateBias = (legId.includes("left") ? 1 : -1) * normalizedDrive.yawRate * 12;
    const hipYawDeg = clamp(
      normalizedDrive.vy * 22 + rotateBias + (stance.hipYawBiasDeg ?? 0),
      DEFAULT_JOINT_LIMITS.hipYawDeg.min,
      DEFAULT_JOINT_LIMITS.hipYawDeg.max,
    );

    let footX = 0;
    let footY = footHeightOffset;
    if (motionMode === "drive") {
      const phaseWave = Math.sin(phase * Math.PI);
      footX = swing ? (-strideX / 2) + strideX * phase : (strideX / 2) - strideX * phase;
      footX += swing ? strideY * 0.35 * phaseWave : -strideY * 0.2 * phaseWave;
      footY = swing ? footHeightOffset + phaseWave * liftY : footHeightOffset - 5 * phaseWave;
    }

    const pose = buildLegPoseFromFoot(
      { x: footX, y: footY },
      calibration,
      {
        startThetaThigh: calibration.thetaThigh,
        startThetaServo: calibration.thetaServo,
        jointLimits: jointLimitsByLeg[legId] ?? DEFAULT_JOINT_LIMITS,
        hipYawDeg,
      },
    );

    legs[legId] = {
      desired: pose,
      gaitPhase: cycle,
      status: motionMode,
    };
  }

  return {
    driveCommand: {
      ...normalizedDrive,
      updatedAt: timeMs,
    },
    motionMode,
    legs,
  };
}

export function flattenServoPose(legs) {
  const flattened = {};
  for (const legId of LEG_IDS) {
    const prefix = legId
      .split("_")
      .map((part) => part[0].toUpperCase())
      .join("");
    const servoAngles = legs[legId]?.desired?.servoAnglesDeg ?? {};
    const servoTrimDeg = legs[legId]?.servoTrimDeg ?? {};
    const mirror = RIGHT_LEG_IDS.has(legId);
    const hipYawDeg = servoAngles.hipYaw;
    const thighDeg = mirror ? mirrorServoAroundNeutral(servoAngles.thigh) : servoAngles.thigh;
    const calfDeg = mirror ? mirrorServoAroundNeutral(servoAngles.calf) : servoAngles.calf;
    flattened[`${prefix}HipYawDeg`] = roundServo(clampServo(hipYawDeg + (servoTrimDeg.hipYaw ?? 0)));
    flattened[`${prefix}ThighDeg`] = roundServo(clampServo(thighDeg + (servoTrimDeg.thigh ?? 0)));
    flattened[`${prefix}CalfDeg`] = roundServo(clampServo(calfDeg + (servoTrimDeg.calf ?? 0)));
  }
  return flattened;
}

export function createFullBodyPoseCommand(legs) {
  return {
    type: "apply_full_body_pose",
    ...flattenServoPose(legs),
  };
}
