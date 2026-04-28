import test from "node:test";
import assert from "node:assert/strict";
import { BridgeState, _internals } from "../bridge/server.js";

class FakeRefactorTransport {
  constructor() {
    this.connected = true;
    this.calls = [];
  }

  requestState() {
    this.calls.push({ method: "requestState" });
    return Promise.resolve();
  }

  setMode(mode) {
    this.calls.push({ method: "setMode", mode });
    return Promise.resolve();
  }

  releaseServos() {
    this.calls.push({ method: "releaseServos" });
    return Promise.resolve();
  }

  setLegServoAngles(legId, angles) {
    this.calls.push({ method: "setLegServoAngles", legId, angles });
    return Promise.resolve();
  }

  setLegServoSpeedLimit(legId, limits) {
    this.calls.push({ method: "setLegServoSpeedLimit", legId, limits });
    return Promise.resolve();
  }

  setLegJointLimits(legId, limits) {
    this.calls.push({ method: "setLegJointLimits", legId, limits });
    return Promise.resolve();
  }

  setLegServoChannelMap(legId, channelMap) {
    this.calls.push({ method: "setLegServoChannelMap", legId, channelMap });
    return Promise.resolve();
  }

  enqueueRawCommand(command) {
    this.calls.push({ method: "enqueueRawCommand", command });
    return Promise.resolve();
  }
}

test("translateCommandForFirmware expands full-body poses into per-leg servo commands", () => {
  const commands = _internals.translateCommandForFirmware(
    {
      type: "apply_full_body_pose",
      FLHipYawDeg: 91,
      FLThighDeg: 72,
      FLCalfDeg: 84,
      FRHipYawDeg: 92,
      FRThighDeg: 73,
      FRCalfDeg: 85,
      RLHipYawDeg: 93,
      RLThighDeg: 74,
      RLCalfDeg: 86,
      RRHipYawDeg: 94,
      RRThighDeg: 75,
      RRCalfDeg: 87,
    },
    _internals.createStatus(),
  );

  assert.deepEqual(commands, [
    { type: "set_leg_servo_angles", legId: "front_left", hipServoDeg: 91, thighServoDeg: 72, calfServoDeg: 84 },
    { type: "set_leg_servo_angles", legId: "front_right", hipServoDeg: 92, thighServoDeg: 73, calfServoDeg: 85 },
    { type: "set_leg_servo_angles", legId: "rear_left", hipServoDeg: 93, thighServoDeg: 74, calfServoDeg: 86 },
    { type: "set_leg_servo_angles", legId: "rear_right", hipServoDeg: 94, thighServoDeg: 75, calfServoDeg: 87 },
  ]);
});

test("translateCommandForFirmware maps legacy dashboard field names to refactor firmware names", () => {
  const status = _internals.createStatus();

  assert.deepEqual(
    _internals.translateCommandForFirmware(
      {
        type: "set_leg_servo_speed_limit",
        legId: "front_left",
        hipYawDegPerSec: 120,
        thighDegPerSec: 180,
        calfDegPerSec: 160,
      },
      status,
    ),
    [
      {
        type: "set_leg_servo_speed_limit",
        legId: "front_left",
        hipDegPerSec: 120,
        thighDegPerSec: 180,
        calfDegPerSec: 160,
      },
    ],
  );

  assert.deepEqual(
    _internals.translateCommandForFirmware(
      {
        type: "set_leg_servo_channel_map",
        legId: "rear_right",
        hipYawChannel: 11,
        thighChannel: 6,
        calfChannel: 7,
      },
      status,
    ),
    [
      {
        type: "set_leg_servo_channel_map",
        legId: "rear_right",
        hipChannel: 11,
        thighChannel: 6,
        calfChannel: 7,
      },
    ],
  );
});

test("translateCommandForFirmware keeps dashboard servo angles aligned with firmware model space", () => {
  const status = _internals.createStatus();
  status.legs.front_left.servoTrimDeg = { hipYaw: 0, thigh: 0, calf: 0 };
  status.legs.front_right.servoTrimDeg = { hipYaw: 0, thigh: 0, calf: 0 };

  assert.deepEqual(
    _internals.translateCommandForFirmware(
      {
        type: "set_leg_servo_angles",
        legId: "front_right",
        hipYawServoDeg: 95,
        thighServoDeg: 80,
        calfServoDeg: 100,
      },
      status,
    ),
    [
      {
        type: "set_leg_servo_angles",
        legId: "front_right",
        hipServoDeg: 95,
        thighServoDeg: 80,
        calfServoDeg: 100,
      },
    ],
  );

  assert.deepEqual(
    _internals.translateCommandForFirmware(
      {
        type: "set_leg_servo_angles",
        legId: "front_left",
        hipYawServoDeg: 95,
        thighServoDeg: 80,
        calfServoDeg: 100,
      },
      status,
    ),
    [
      {
        type: "set_leg_servo_angles",
        legId: "front_left",
        hipServoDeg: 95,
        thighServoDeg: 80,
        calfServoDeg: 100,
      },
    ],
  );
});

test("incoming firmware state keeps dashboard joint limits while exposing raw firmware limits separately", () => {
  const currentStatus = _internals.createStatus();
  const normalized = _internals.normalizeFirmwareStatePayload(
    {
      mode: "direct_servo_angles",
      legs: {
        front_left: {
          servoChannelMap: { hip: 8, thigh: 0, calf: 1 },
          servoSpeedLimitDegPerSec: { hip: 120, thigh: 180, calf: 160 },
          jointLimits: {
            thighDeg: { min: -140, max: 10 },
            calfDeg: { min: -160, max: -20 },
          },
          desired: {
            servoAnglesDeg: { hip: 93, thigh: 71, calf: 82 },
          },
          current: {
            servoAnglesDeg: { hip: 92, thigh: 69, calf: 80 },
          },
        },
      },
    },
    currentStatus,
  );

  const bridge = new BridgeState();
  const state = bridge.normalizeStatePayload(normalized);

  assert.deepEqual(normalized.legs.front_left.servoChannelMap, {
    hipYaw: 8,
    thigh: 0,
    calf: 1,
  });
  assert.deepEqual(normalized.legs.front_left.servoSpeedLimitDegPerSec, {
    hipYaw: 120,
    thigh: 180,
    calf: 160,
  });
  assert.deepEqual(normalized.legs.front_left.firmwareJointLimits.hipYawDeg, currentStatus.legs.front_left.jointLimits.hipYawDeg);
  assert.deepEqual(normalized.legs.front_left.firmwareJointLimits.thighDeg, { min: -140, max: 10 });
  assert.deepEqual(normalized.legs.front_left.firmwareJointLimits.calfDeg, { min: -160, max: -20 });
  assert.deepEqual(state.legs.front_left.desired.servoAnglesDeg, {
    hipYaw: 93,
    thigh: 71,
    calf: 82,
  });
  assert.deepEqual(state.legs.front_left.jointLimits, currentStatus.legs.front_left.jointLimits);
});

test("incoming firmware state keeps right-leg servo telemetry in dashboard model space", () => {
  const currentStatus = _internals.createStatus();
  const normalized = _internals.normalizeFirmwareStatePayload(
    {
      mode: "direct_servo_angles",
      legs: {
        front_right: {
          desired: {
            servoAnglesDeg: { hip: 95, thigh: 80, calf: 100 },
          },
          current: {
            servoAnglesDeg: { hip: 95, thigh: 80, calf: 100 },
          },
        },
        rear_right: {
          desired: {
            servoAnglesDeg: { hip: 95, thigh: 80, calf: 100 },
          },
          current: {
            servoAnglesDeg: { hip: 95, thigh: 80, calf: 100 },
          },
        },
      },
    },
    currentStatus,
  );

  const state = new BridgeState().normalizeStatePayload(normalized);

  assert.deepEqual(state.legs.front_right.desired.servoAnglesDeg, {
    hipYaw: 95,
    thigh: 80,
    calf: 100,
  });
  assert.deepEqual(state.legs.rear_right.desired.servoAnglesDeg, {
    hipYaw: 95,
    thigh: 80,
    calf: 100,
  });
  assert.equal(state.legs.front_right.current.reachable, true);
  assert.equal(state.legs.rear_right.current.reachable, true);
  assert.deepEqual(state.legs.front_right.current.servoAnglesDeg, {
    hipYaw: 95,
    thigh: 80,
    calf: 100,
  });
});

test("incoming refactor bridge state normalizes back into legacy dashboard leg ids and joint keys", () => {
  const currentStatus = _internals.createStatus();
  const normalized = _internals.normalizeRefactorStatePayload(
    {
      mode: "direct_servo_angles",
      legs: {
        FL: {
          servoChannelMap: { coxa: 8, femur: 0, tibia: 1 },
          servoSpeedLimitDegPerSec: { coxa: 120, femur: 180, tibia: 160 },
          jointLimits: {
            femurDeg: { min: -140, max: 10 },
            tibiaDeg: { min: -160, max: -20 },
          },
          desired: {
            servoAnglesDeg: { coxa: 93, femur: 71, tibia: 82 },
          },
          current: {
            servoAnglesDeg: { coxa: 92, femur: 69, tibia: 80 },
          },
        },
      },
    },
    currentStatus,
  );

  assert.deepEqual(normalized.legs.front_left.servoChannelMap, {
    hipYaw: 8,
    thigh: 0,
    calf: 1,
  });
  assert.deepEqual(normalized.legs.front_left.servoSpeedLimitDegPerSec, {
    hipYaw: 120,
    thigh: 180,
    calf: 160,
  });
  assert.deepEqual(normalized.legs.front_left.firmwareJointLimits.thighDeg, { min: -140, max: 10 });
  assert.deepEqual(normalized.legs.front_left.firmwareJointLimits.calfDeg, { min: -160, max: -20 });
  assert.deepEqual(normalized.legs.front_left.desired.servoAnglesDeg, {
    hipYaw: 93,
    thigh: 71,
    calf: 82,
  });
});

test("BridgeState dispatches legacy commands through refactor transport leg and joint conventions", async () => {
  const bridge = new BridgeState({ pacingMs: 0, ackTimeoutMs: 0 });
  const serial = new FakeRefactorTransport();
  bridge.serial = serial;
  bridge.status.connected = true;

  await bridge.send({ type: "get_state" });
  await bridge.send({
    type: "set_leg_servo_angles",
    legId: "front_left",
    hipServoDeg: 90,
    thighServoDeg: 70,
    calfServoDeg: 80,
  });
  await bridge.send({
    type: "set_leg_servo_speed_limit",
    legId: "rear_right",
    hipDegPerSec: 120,
    thighDegPerSec: 180,
    calfDegPerSec: 160,
  });
  await bridge.send({
    type: "set_leg_joint_limits",
    legId: "front_right",
    thighMinDeg: -145,
    thighMaxDeg: 15,
    calfMinDeg: -165,
    calfMaxDeg: -25,
  });
  await bridge.send({
    type: "set_leg_servo_channel_map",
    legId: "rear_left",
    hipChannel: 10,
    thighChannel: 4,
    calfChannel: 5,
  });
  await bridge.send({ type: "play_animation", name: "wave" });

  assert.deepEqual(serial.calls, [
    { method: "requestState" },
    {
      method: "setLegServoAngles",
      legId: "FL",
      angles: { coxa: 90, femur: 70, tibia: 80 },
    },
    {
      method: "setLegServoSpeedLimit",
      legId: "RR",
      limits: { coxa: 120, femur: 180, tibia: 160 },
    },
    {
      method: "setLegJointLimits",
      legId: "FR",
      limits: { femur: [-145, 15], tibia: [-165, -25] },
    },
    {
      method: "setLegServoChannelMap",
      legId: "RL",
      channelMap: { coxa: 10, femur: 4, tibia: 5 },
    },
    {
      method: "enqueueRawCommand",
      command: { type: "play_animation", name: "wave" },
    },
  ]);
});

test("BridgeState uploads animation frames through the refactor transport raw command path", async () => {
  const bridge = new BridgeState({ pacingMs: 0, ackTimeoutMs: 0 });
  const serial = new FakeRefactorTransport();
  bridge.serial = serial;
  bridge.status.connected = true;

  await bridge.uploadAnimation({
    version: 2,
    name: "wave",
    duration: 1,
    tracks: {
      front_left: [{ time: 0, foot: { x: 0, y: 0 } }],
      front_right: [],
      rear_left: [],
      rear_right: [],
    },
  });

  assert.equal(serial.calls[0].method, "enqueueRawCommand");
  assert.deepEqual(serial.calls[0].command, {
    type: "upload_animation",
    stage: "begin",
    name: "wave",
    duration: 1,
    trackCount: 4,
  });

  const frameCall = serial.calls.find((call) => call.command?.stage === "frame");
  assert.deepEqual(frameCall?.command?.type, "upload_animation");
  assert.deepEqual(frameCall?.command?.legId, "front_left");
  assert.equal(typeof frameCall?.command?.hipServoDeg, "number");
  assert.equal(typeof frameCall?.command?.thighServoDeg, "number");
  assert.equal(typeof frameCall?.command?.calfServoDeg, "number");

  const lastCall = serial.calls.at(-1);
  assert.deepEqual(lastCall, {
    method: "enqueueRawCommand",
    command: {
      type: "upload_animation",
      stage: "commit",
      name: "wave",
    },
  });
});
