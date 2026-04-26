import test from "node:test";
import assert from "node:assert/strict";
import { buildLegPoseFromFoot, buildLegPoseFromJointAngles, buildLegPoseFromServoAngles, createNeutralCalibration, geometryWithinJointLimits, normalizeJointLimits, robotOverviewGeometry } from "../shared/kinematics.js";
import { DEFAULT_JOINT_LIMITS } from "../shared/robot-config.js";

const calibration = createNeutralCalibration();

test("normalizeJointLimits sorts reversed ranges", () => {
  const limits = normalizeJointLimits({
    thighDeg: { min: 10, max: -40 },
    calfDeg: { min: -20, max: -120 },
  });

  assert.deepEqual(limits, {
    hipYawDeg: { min: -35, max: 35 },
    thighDeg: { min: -40, max: 10 },
    calfDeg: { min: -120, max: -20 },
  });
});

test("buildLegPoseFromJointAngles clamps to configured limits", () => {
  const pose = buildLegPoseFromJointAngles(
    { thigh: 30, calf: -180 },
    calibration,
    {
      jointLimits: {
        thighDeg: { min: -50, max: -10 },
        calfDeg: { min: -130, max: -60 },
      },
    },
  );

  assert.equal(pose.jointAnglesDeg.thigh, -10);
  assert.equal(pose.jointAnglesDeg.calf, -130);
});

test("buildLegPoseFromFoot returns a valid limited geometry for reachable targets", () => {
  const limits = {
    thighDeg: { min: -145, max: 15 },
    calfDeg: { min: -165, max: -25 },
  };
  const pose = buildLegPoseFromFoot(
    { x: 12, y: -8 },
    calibration,
    {
      startThetaThigh: calibration.thetaThigh,
      startThetaServo: calibration.thetaServo,
      jointLimits: limits,
    },
  );

  assert.equal(pose.reachable, true);
  assert.equal(pose.geometry.valid, true);
  assert.equal(geometryWithinJointLimits(pose.geometry, limits), true);
});

test("thigh servo 90 degrees maps to a ground-parallel thigh", () => {
  assert.equal(calibration.thetaThigh, 0);

  const pose = buildLegPoseFromJointAngles({ hipYaw: 0, thigh: 0, calf: -90 }, calibration);
  assert.ok(Math.abs(pose.servoAnglesDeg.thigh - 90) < 0.001);
});

test("reachable foot targets prefer servo angles inside 0 to 180 degrees", () => {
  const pose = buildLegPoseFromFoot(
    { x: 0, y: 0 },
    calibration,
    {
      startThetaThigh: calibration.thetaThigh,
      startThetaServo: calibration.thetaServo,
      jointLimits: {
        thighDeg: { min: -145, max: 15 },
        calfDeg: { min: -165, max: -25 },
      },
    },
  );

  assert.equal(pose.reachable, true);
  assert.ok(pose.servoAnglesDeg.thigh >= 0 && pose.servoAnglesDeg.thigh <= 180);
  assert.ok(pose.servoAnglesDeg.calf >= 0 && pose.servoAnglesDeg.calf <= 180);
});

test("leftward reachable foot targets keep their full workspace", () => {
  const pose = buildLegPoseFromFoot(
    { x: -60, y: 0 },
    calibration,
    {
      startThetaThigh: calibration.thetaThigh,
      startThetaServo: calibration.thetaServo,
      jointLimits: {
        thighDeg: { min: -145, max: 15 },
        calfDeg: { min: -165, max: -25 },
      },
    },
  );

  assert.equal(pose.reachable, true);
  assert.equal(pose.withinTolerance, true);
  assert.ok(Math.abs(pose.foot.x + 60) < 1.5);
  assert.ok(Math.abs(pose.foot.y) < 1.5);
  assert.ok(pose.servoAnglesDeg.thigh >= 0 && pose.servoAnglesDeg.thigh <= 180);
  assert.ok(pose.servoAnglesDeg.calf >= 0 && pose.servoAnglesDeg.calf <= 180);
});

test("a 90 degree thigh servo command maps to a horizontal thigh link", () => {
  const pose = buildLegPoseFromServoAngles(
    { hipYaw: 90, thigh: 90, calf: 90 },
    calibration,
    { jointLimits: DEFAULT_JOINT_LIMITS },
  );

  assert.ok(Math.abs(pose.geometry.thetaThigh) < 1e-9);
});

test("robotOverviewGeometry projects hip yaw around the configured hip rotation point", () => {
  const neutralPose = buildLegPoseFromFoot({ x: 12, y: -8 }, calibration, { hipYawDeg: 0 });
  const yawedPose = buildLegPoseFromFoot({ x: 12, y: -8 }, calibration, { hipYawDeg: 25 });
  const neutralState = {
    legs: {
      front_left: { desired: neutralPose },
      front_right: { desired: neutralPose },
      rear_left: { desired: neutralPose },
      rear_right: { desired: neutralPose },
    },
  };
  const yawedState = {
    legs: {
      front_left: { desired: yawedPose },
      front_right: { desired: yawedPose },
      rear_left: { desired: yawedPose },
      rear_right: { desired: yawedPose },
    },
  };

  const neutralOverview = robotOverviewGeometry(neutralState, "desired")[0];
  const yawedOverview = robotOverviewGeometry(yawedState, "desired")[0];

  assert.notEqual(neutralOverview.points[2].y, yawedOverview.points[2].y);
});
