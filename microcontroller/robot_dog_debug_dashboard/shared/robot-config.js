export const LEG_IDS = ["front_left", "front_right", "rear_left", "rear_right"];

export const LEG_LABELS = {
  front_left: "Front Left",
  front_right: "Front Right",
  rear_left: "Rear Left",
  rear_right: "Rear Right"
};

export const ROBOT_LAYOUT = {
  body: {
    width: 280,
    height: 110
  },
  anchors: {
    front_left: { x: -100, y: -40 },
    front_right: { x: 100, y: -40 },
    rear_left: { x: -100, y: 40 },
    rear_right: { x: 100, y: 40 }
  }
};

export const LEG_DRAWING = {
  scale: 1.4,
  stageWidth: 420,
  stageHeight: 420,
  offset: { x: 210, y: 170 }
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
  footOriginOffset: { x: 40, y: -140 }
};

export const MODE_OPTIONS = [
  "idle",
  "direct_foot_xy",
  "direct_joint_angles",
  "direct_servo_angles",
  "builtin_walk",
  "builtin_crouch",
  "animation_playback"
];

export const DEFAULT_LEG_COMMAND = {
  foot: { x: 0, y: 0 },
  jointAnglesDeg: { thigh: 0, calf: -90 },
  servoAnglesDeg: { thigh: 90, calf: 90 }
};

export const DEFAULT_SERVO_CHANNEL_MAP = {
  front_left: { thigh: 0, calf: 1 },
  front_right: { thigh: 2, calf: 3 },
  rear_left: { thigh: 4, calf: 5 },
  rear_right: { thigh: 6, calf: 7 }
};

export const DEFAULT_JOINT_LIMITS = {
  thighDeg: { min: -145, max: 15 },
  calfDeg: { min: -165, max: -25 }
};

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
      { time: 2, foot: { x: 0, y: 0 } }
    ],
    front_right: [
      { time: 0, foot: { x: 30, y: 0 } },
      { time: 0.5, foot: { x: 10, y: -8 } },
      { time: 1, foot: { x: 0, y: 0 } },
      { time: 1.5, foot: { x: 20, y: 8 } },
      { time: 2, foot: { x: 30, y: 0 } }
    ],
    rear_left: [
      { time: 0, foot: { x: 30, y: 0 } },
      { time: 0.5, foot: { x: 10, y: -8 } },
      { time: 1, foot: { x: 0, y: 0 } },
      { time: 1.5, foot: { x: 20, y: 8 } },
      { time: 2, foot: { x: 30, y: 0 } }
    ],
    rear_right: [
      { time: 0, foot: { x: 0, y: 0 } },
      { time: 0.5, foot: { x: 20, y: 8 } },
      { time: 1, foot: { x: 30, y: 0 } },
      { time: 1.5, foot: { x: 10, y: -8 } },
      { time: 2, foot: { x: 0, y: 0 } }
    ]
  }
};
