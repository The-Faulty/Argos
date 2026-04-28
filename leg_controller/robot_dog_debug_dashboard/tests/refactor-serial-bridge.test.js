import test from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { SerialBridge } from "../bridge/refactor-serial-bridge.js";

class FakeParser extends EventEmitter {}

class FakeSerialPort extends EventEmitter {
  constructor() {
    super();
    this.isOpen = false;
    this.writes = [];
    this.pendingWrites = [];
  }

  open(callback) {
    this.isOpen = true;
    callback?.(null);
    queueMicrotask(() => {
      this.emit("open");
    });
  }

  close(callback) {
    this.isOpen = false;
    callback?.(null);
    queueMicrotask(() => {
      this.emit("close");
    });
  }

  write(line, callback) {
    this.writes.push(JSON.parse(String(line).trim()));
    this.pendingWrites.push(callback);
  }

  drain(callback) {
    callback(null);
  }

  releaseNext(error = null) {
    const callback = this.pendingWrites.shift();
    if (!callback) {
      throw new Error("No pending serial write to release.");
    }
    callback(error);
  }
}

async function flushEventLoop() {
  await new Promise((resolve) => setTimeout(resolve, 0));
}

test("SerialBridge sends the refactor handshake in the canonical order", async () => {
  const port = new FakeSerialPort();
  const parser = new FakeParser();
  const transport = new SerialBridge({
    pacingMs: 0,
    ackTimeoutMs: 0,
    serialPortFactory: () => port,
    parserFactory: () => parser,
  });
  const errors = [];
  transport.on("error", (error) => {
    errors.push(error);
  });

  transport.connect();
  await flushEventLoop();

  assert.deepEqual(port.writes[0], {
    type: "hello",
    seq: 1,
  });

  port.releaseNext();
  await flushEventLoop();

  assert.deepEqual(port.writes[1], {
    type: "set_mode",
    mode: "direct_servo_angles",
    seq: 2,
  });

  port.releaseNext();
  await flushEventLoop();

  assert.deepEqual(port.writes[2], {
    type: "get_state",
    seq: 3,
  });

  port.releaseNext();
  await flushEventLoop();

  assert.deepEqual(errors, []);
});

test("SerialBridge coalesces per-leg servo writes while preserving FIFO priority for non-servo commands", async () => {
  const port = new FakeSerialPort();
  port.isOpen = true;
  const transport = new SerialBridge({ pacingMs: 0, ackTimeoutMs: 0 });
  const errors = [];
  transport.on("error", (error) => {
    errors.push(error);
  });
  transport.port = port;
  transport.connected = true;

  const statePromise = transport.requestState();
  const firstServoPromise = transport.setLegServoAngles("FL", {
    coxa: 90,
    femur: 70,
    tibia: 80,
  });
  const secondServoPromise = transport.setLegServoAngles("FL", {
    coxa: 91,
    femur: 71,
    tibia: 81,
  });

  assert.deepEqual(port.writes[0], {
    type: "get_state",
    seq: 1,
  });

  port.releaseNext();
  await flushEventLoop();

  assert.deepEqual(port.writes[1], {
    type: "set_leg_servo_angles",
    legId: "front_left",
    hipServoDeg: 91,
    thighServoDeg: 71,
    calfServoDeg: 81,
    seq: 2,
  });

  port.releaseNext();
  await Promise.all([statePromise, firstServoPromise, secondServoPromise]);
  assert.deepEqual(errors, []);
});

test("SerialBridge serializes per-leg limit and release commands with refactor wire field names", async () => {
  const port = new FakeSerialPort();
  port.isOpen = true;
  const transport = new SerialBridge({ pacingMs: 0, ackTimeoutMs: 0 });
  const errors = [];
  transport.on("error", (error) => {
    errors.push(error);
  });
  transport.port = port;
  transport.connected = true;

  const speedPromise = transport.setLegServoSpeedLimit("RR", {
    coxa: 120,
    femur: 180,
    tibia: 160,
  });

  assert.deepEqual(port.writes[0], {
    type: "set_leg_servo_speed_limit",
    legId: "rear_right",
    hipDegPerSec: 120,
    thighDegPerSec: 180,
    calfDegPerSec: 160,
    seq: 1,
  });
  port.releaseNext();
  await speedPromise;

  const limitsPromise = transport.setLegJointLimits("FR", {
    femur: [-145, 15],
    tibia: [-165, -25],
  });

  assert.deepEqual(port.writes[1], {
    type: "set_leg_joint_limits",
    legId: "front_right",
    thighMinDeg: -145,
    thighMaxDeg: 15,
    calfMinDeg: -165,
    calfMaxDeg: -25,
    seq: 2,
  });
  port.releaseNext();
  await limitsPromise;

  const releasePromise = transport.releaseServos();
  assert.deepEqual(port.writes[2], {
    type: "release_servos",
    seq: 3,
  });
  port.releaseNext();
  await releasePromise;

  assert.deepEqual(errors, []);
});
