import test from "node:test";
import assert from "node:assert/strict";
import { buildLegPoseFromFoot, buildLegPoseFromJointAngles, createNeutralCalibration, geometryWithinJointLimits, normalizeJointLimits } from "../shared/kinematics.js";

const calibration = createNeutralCalibration();

test("normalizeJointLimits sorts reversed ranges", () => {
  const limits = normalizeJointLimits({
    thighDeg: { min: 10, max: -40 },
    calfDeg: { min: -20, max: -120 },
  });

  assert.deepEqual(limits, {
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
