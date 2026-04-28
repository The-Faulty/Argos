import { buildLegPoseFromFoot, createNeutralCalibration } from "./kinematics.js";
import {
  DEFAULT_DRIVE_COMMAND,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_SERVO_TRIM_DEG,
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
const DRIVE_GAIT_EXAGGERATION = 2;
const DRIVE_FORWARD_STRIDE_MM = 34 * DRIVE_GAIT_EXAGGERATION;
const DRIVE_STRAFE_STRIDE_MM = 16 * DRIVE_GAIT_EXAGGERATION;
const DRIVE_FOOT_LIFT_MM = 28 * DRIVE_GAIT_EXAGGERATION;
const DRIVE_SUPPORT_DIP_MM = 5 * DRIVE_GAIT_EXAGGERATION;
const DRIVE_YAW_HIP_BIAS_DEG = 12 * DRIVE_GAIT_EXAGGERATION;
const DRIVE_STRAFE_HIP_YAW_DEG = 22 * DRIVE_GAIT_EXAGGERATION;
const LEG_TRANSPORT_SIGNS = {
  front_left: { hipYaw: 1, thigh: 1, calf: 1 },
  front_right: { hipYaw: -1, thigh: -1, calf: -1 },
  rear_left: { hipYaw: 1, thigh: 1, calf: 1 },
  rear_right: { hipYaw: -1, thigh: -1, calf: -1 },
};

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function roundServo(value) {
  return Math.round((value ?? 90) * 100) / 100;
}

function clampServo(value) {
  return clamp(value, 0, 180);
}

function normalizeServoValue(value, fallback = 90) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function normalizeServoTrim(servoTrimDeg = DEFAULT_SERVO_TRIM_DEG) {
  return {
    hipYaw: normalizeServoValue(servoTrimDeg?.hipYaw, DEFAULT_SERVO_TRIM_DEG.hipYaw),
    thigh: normalizeServoValue(servoTrimDeg?.thigh, DEFAULT_SERVO_TRIM_DEG.thigh),
    calf: normalizeServoValue(servoTrimDeg?.calf, DEFAULT_SERVO_TRIM_DEG.calf),
  };
}

function normalizeServoAngles(servoAnglesDeg = {}) {
  return {
    hipYaw: normalizeServoValue(servoAnglesDeg?.hipYaw ?? servoAnglesDeg?.hip, 90),
    thigh: normalizeServoValue(servoAnglesDeg?.thigh, 90),
    calf: normalizeServoValue(servoAnglesDeg?.calf, 90),
  };
}

function getTransportSigns(legId) {
  return LEG_TRANSPORT_SIGNS[legId] ?? LEG_TRANSPORT_SIGNS.front_left;
}

export function mapLegServoAnglesToHardware(
  legId,
  servoAnglesDeg = {},
  servoTrimDeg = DEFAULT_SERVO_TRIM_DEG,
) {
  const angles = normalizeServoAngles(servoAnglesDeg);
  const trim = normalizeServoTrim(servoTrimDeg);
  const signs = getTransportSigns(legId);

  return {
    hipYaw: roundServo(clampServo(angles.hipYaw + (trim.hipYaw / signs.hipYaw))),
    thigh: roundServo(clampServo(angles.thigh + (trim.thigh / signs.thigh))),
    calf: roundServo(clampServo(angles.calf + (trim.calf / signs.calf))),
  };
}

export function mapHardwareServoAnglesToLeg(
  legId,
  servoAnglesDeg = {},
  servoTrimDeg = DEFAULT_SERVO_TRIM_DEG,
) {
  const angles = normalizeServoAngles(servoAnglesDeg);
  const trim = normalizeServoTrim(servoTrimDeg);
  const signs = getTransportSigns(legId);

  return {
    hipYaw: roundServo(clampServo(angles.hipYaw - (trim.hipYaw / signs.hipYaw))),
    thigh: roundServo(clampServo(angles.thigh - (trim.thigh / signs.thigh))),
    calf: roundServo(clampServo(angles.calf - (trim.calf / signs.calf))),
  };
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
    const strideX = normalizedDrive.vx * DRIVE_FORWARD_STRIDE_MM * (stance.strideScale ?? 1);
    const strideY = normalizedDrive.vy * DRIVE_STRAFE_STRIDE_MM * (stance.strideScale ?? 1);
    const liftY = strideMagnitude > 0.01 ? DRIVE_FOOT_LIFT_MM : 0;
    const rotateBias = (legId.includes("left") ? 1 : -1) * normalizedDrive.yawRate * DRIVE_YAW_HIP_BIAS_DEG;
    const hipYawDeg = clamp(
      normalizedDrive.vy * DRIVE_STRAFE_HIP_YAW_DEG + rotateBias + (stance.hipYawBiasDeg ?? 0),
      DEFAULT_JOINT_LIMITS.hipYawDeg.min,
      DEFAULT_JOINT_LIMITS.hipYawDeg.max,
    );

    let footX = 0;
    let footY = footHeightOffset;
    if (motionMode === "drive") {
      const phaseWave = Math.sin(phase * Math.PI);
      footX = swing ? (-strideX / 2) + strideX * phase : (strideX / 2) - strideX * phase;
      footX += swing ? strideY * 0.35 * phaseWave : -strideY * 0.2 * phaseWave;
      footY = swing ? footHeightOffset + phaseWave * liftY : footHeightOffset - DRIVE_SUPPORT_DIP_MM * phaseWave;
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
    const hardwareAngles = mapLegServoAnglesToHardware(
      legId,
      legs[legId]?.desired?.servoAnglesDeg ?? {},
      legs[legId]?.servoTrimDeg ?? {},
    );
    flattened[`${prefix}HipYawDeg`] = hardwareAngles.hipYaw;
    flattened[`${prefix}ThighDeg`] = hardwareAngles.thigh;
    flattened[`${prefix}CalfDeg`] = hardwareAngles.calf;
  }
  return flattened;
}

export function createFullBodyPoseCommand(legs) {
  return {
    type: "apply_full_body_pose",
    ...flattenServoPose(legs),
  };
}
