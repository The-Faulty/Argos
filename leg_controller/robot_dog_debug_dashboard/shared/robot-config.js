export const LEG_IDS = ["front_left", "front_right", "rear_left", "rear_right"];
export const JOINT_IDS = ["hipYaw", "thigh", "calf"];

export const LEG_LABELS = {
  front_left: "Front Left",
  front_right: "Front Right",
  rear_left: "Rear Left",
  rear_right: "Rear Right",
};

export const ROBOT_LAYOUT = {
  body: {
    width: 280,
    height: 110,
  },
  anchors: {
    front_left: { x: -100, y: -40 },
    front_right: { x: 100, y: -40 },
    rear_left: { x: -100, y: 40 },
    rear_right: { x: 100, y: 40 },
  },
};

export const LEG_DRAWING = {
  scale: 1.4,
  stageWidth: 420,
  stageHeight: 420,
  offset: { x: 210, y: 170 },
};

export const LEG_GEOMETRY = {
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
  jointRotationPoints3d: {
    hipYaw: { x: 0, y: -11, z: -41.8 },
    thigh: { x: 0, y: 0, z: 0 },
    calfServo: { x: -20, y: -22, z: 0 },
  },
};

export const MOTION_MODE_OPTIONS = ["idle", "stand", "drive", "calibration"];

export const MODE_OPTIONS = [
  "idle",
  "stand",
  "drive",
  "calibration",
  "direct_foot_xy",
  "direct_joint_angles",
  "direct_servo_angles",
  "builtin_walk",
  "builtin_crouch",
  "animation_playback",
];

export const DEFAULT_DRIVE_COMMAND = {
  vx: 0,
  vy: 0,
  yawRate: 0,
  source: "idle",
  updatedAt: 0,
};

export const DEFAULT_LEG_COMMAND = {
  foot: { x: 0, y: 0 },
  jointAnglesDeg: { hipYaw: 0, thigh: 0, calf: -90 },
  servoAnglesDeg: { hipYaw: 90, thigh: 90, calf: 90 },
};

// Match the main-era PCA9685 wiring so hip-yaw stays on the dedicated upper channels.
export const DEFAULT_SERVO_CHANNEL_MAP = {
  front_left: { hipYaw: 8, thigh: 0, calf: 1 },
  front_right: { hipYaw: 9, thigh: 2, calf: 3 },
  rear_left: { hipYaw: 10, thigh: 4, calf: 5 },
  rear_right: { hipYaw: 11, thigh: 6, calf: 7 },
};

export const DEFAULT_JOINT_LIMITS = {
  hipYawDeg: { min: -35, max: 35 },
  thighDeg: { min: -145, max: 15 },
  calfDeg: { min: -165, max: -25 },
};

export const NEUTRAL_CALIBRATION = {
  thetaThigh: 0,
  thetaServo: -2.768896484375,
};

export const DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC = {
  hipYaw: 180,
  thigh: 180,
  calf: 180,
};

export const DEFAULT_SERVO_TRIM_DEG = {
  hipYaw: 0,
  thigh: 0,
  calf: 0,
};

export const STANCE_HEIGHT_RANGE_MM = {
  min: -25,
  max: 35,
  step: 1,
};

export const DEFAULT_STANCE = {
  height: 0,
  strideScale: 1,
  hipYawBiasDeg: 0,
};

export const ROBOT_CONFIG = {
  legs: LEG_IDS.length,
  jointsPerLeg: JOINT_IDS.length,
  servoChannels: LEG_IDS.length * JOINT_IDS.length,
  jointIds: [...JOINT_IDS],
  legIds: [...LEG_IDS],
  jointRotationPoints3d: cloneJointRotationPoints(),
  defaultServoTrimDeg: { ...DEFAULT_SERVO_TRIM_DEG },
};

function cloneJointRotationPoints() {
  return JSON.parse(JSON.stringify(LEG_GEOMETRY.jointRotationPoints3d));
}

export const DEFAULT_FULL_BODY_CLIP = {
  version: 2,
  name: "debug-step",
  duration: 2,
  tracks: {
    front_left: [
      { time: 0, foot: { x: 0, y: 0 } },
      { time: 0.5, foot: { x: 20, y: 8 } },
      { time: 1, foot: { x: 30, y: 0 } },
      { time: 1.5, foot: { x: 10, y: -8 } },
      { time: 2, foot: { x: 0, y: 0 } },
    ],
    front_right: [
      { time: 0, foot: { x: 30, y: 0 } },
      { time: 0.5, foot: { x: 10, y: -8 } },
      { time: 1, foot: { x: 0, y: 0 } },
      { time: 1.5, foot: { x: 20, y: 8 } },
      { time: 2, foot: { x: 30, y: 0 } },
    ],
    rear_left: [
      { time: 0, foot: { x: 30, y: 0 } },
      { time: 0.5, foot: { x: 10, y: -8 } },
      { time: 1, foot: { x: 0, y: 0 } },
      { time: 1.5, foot: { x: 20, y: 8 } },
      { time: 2, foot: { x: 30, y: 0 } },
    ],
    rear_right: [
      { time: 0, foot: { x: 0, y: 0 } },
      { time: 0.5, foot: { x: 20, y: 8 } },
      { time: 1, foot: { x: 30, y: 0 } },
      { time: 1.5, foot: { x: 10, y: -8 } },
      { time: 2, foot: { x: 0, y: 0 } },
    ],
  },
};
