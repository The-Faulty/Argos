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

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
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
  const heightBias = stance.height ?? 0;
  const cycleTimeSec = 1.2 - strideMagnitude * 0.45;
  const timeSec = Math.max(0, timeMs) / 1000;
  const legs = {};

  for (const legId of LEG_IDS) {
    const phaseOffset = PHASE_OFFSETS[legId];
    const cycle = cycleTimeSec > 0 ? ((timeSec / cycleTimeSec) + phaseOffset) % 1 : phaseOffset;
    const swing = cycle < 0.5;
    const phase = swing ? cycle / 0.5 : (cycle - 0.5) / 0.5;
    const strideX = normalizedDrive.vx * 24 * (stance.strideScale ?? 1);
    const liftY = strideMagnitude > 0.01 ? 10 : 0;
    const rotateBias = (legId.includes("left") ? 1 : -1) * normalizedDrive.yawRate * 8;
    const hipYawDeg = clamp(
      normalizedDrive.vy * 22 + rotateBias + (stance.hipYawBiasDeg ?? 0),
      DEFAULT_JOINT_LIMITS.hipYawDeg.min,
      DEFAULT_JOINT_LIMITS.hipYawDeg.max,
    );

    let footX = 0;
    let footY = heightBias;
    if (motionMode === "drive") {
      footX = swing ? (-strideX / 2) + strideX * phase : (strideX / 2) - strideX * phase;
      footY = swing ? heightBias + Math.sin(phase * Math.PI) * liftY : heightBias - 3 * Math.sin(phase * Math.PI);
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
    flattened[`${prefix}HipYawDeg`] = servoAngles.hipYaw ?? 90;
    flattened[`${prefix}ThighDeg`] = servoAngles.thigh ?? 90;
    flattened[`${prefix}CalfDeg`] = servoAngles.calf ?? 90;
  }
  return flattened;
}
