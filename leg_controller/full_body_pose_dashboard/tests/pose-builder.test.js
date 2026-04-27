import test from "node:test";
import assert from "node:assert/strict";
import {
  bodyToWorldPoint,
  createNeutralStaticPose,
  createPoseApplyCommand,
  getUrdfJointValues,
  LEG_THIGH_PIVOTS_MM,
  legPlaneFootToWorld,
  parseStaticPose,
  serializeStaticPose,
  solveStaticPose,
  updateBodyKeepingFeetLocked,
  updateFootWorld,
  worldToBodyPoint,
} from "../shared/pose-builder.js";

const TEST_LEG_PREFIXES = {
  front_left: "FL",
  front_right: "FR",
  rear_left: "RL",
  rear_right: "RR",
};

const TEST_URDF_CHAINS = {
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

function addVector3(a, b) {
  return {
    x: a.x + b.x,
    y: a.y + b.y,
    z: a.z + b.z,
  };
}

function urdfFootPoint(legId, joints) {
  const chain = TEST_URDF_CHAINS[legId];
  const tibiaLocal = rotateVectorAroundAxis(chain.footPoint, chain.tibiaAxis, joints.tibia);
  const tibiaFrame = addVector3(chain.tibiaOrigin, tibiaLocal);
  const femurLocal = rotateVectorAroundAxis(tibiaFrame, chain.femurAxis, joints.femur);
  const femurFrame = addVector3(chain.femurOrigin, femurLocal);
  const coxaLocal = rotateVectorAroundAxis(femurFrame, chain.coxaAxis, joints.coxa);
  return addVector3(chain.coxaOrigin, coxaLocal);
}

function urdfFootErrorMm(solvedPose, legId) {
  const values = getUrdfJointValues(solvedPose);
  const prefix = TEST_LEG_PREFIXES[legId];
  const point = urdfFootPoint(legId, {
    coxa: values[`${prefix}_coxa_joint`],
    femur: values[`${prefix}_femur_joint`],
    tibia: values[`${prefix}_tibia_joint`],
  });
  const target = solvedPose.legs[legId].footBodyMm;
  return Math.hypot(point.x - target.x, point.y - target.y, point.z - target.z);
}

function solveLegPose(neutral, legId, patch) {
  return solveStaticPose(updateFootWorld(neutral, legId, patch));
}

function getVisualJointTriplet(solvedPose, legId) {
  const values = getUrdfJointValues(solvedPose);
  const prefix = TEST_LEG_PREFIXES[legId];
  return [
    values[`${prefix}_coxa_joint`],
    values[`${prefix}_femur_joint`],
    values[`${prefix}_tibia_joint`],
  ];
}

function maxMirroredMagnitudeDelta(leftJoints, rightJoints) {
  return Math.max(
    Math.abs(Math.abs(leftJoints[0]) - Math.abs(rightJoints[0])),
    Math.abs(Math.abs(leftJoints[1]) - Math.abs(rightJoints[1])),
    Math.abs(Math.abs(leftJoints[2]) - Math.abs(rightJoints[2])),
  );
}

test("static pose serializes and parses with all feet", () => {
  const pose = createNeutralStaticPose("bench");
  const parsed = parseStaticPose(serializeStaticPose(pose));

  assert.equal(parsed.version, 1);
  assert.equal(parsed.name, "bench");
  assert.deepEqual(Object.keys(parsed.feetWorldMm).sort(), [
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
  ]);
});

test("body transform round-trips world and body points", () => {
  const pose = updateBodyKeepingFeetLocked(createNeutralStaticPose(), {
    positionMm: { x: 25, y: -12, z: 18 },
    rotationDeg: { roll: 6, pitch: -4, yaw: 18 },
  });
  const pointBody = { x: 42, y: -16, z: -120 };
  const pointWorld = bodyToWorldPoint(pointBody, pose.body);
  const roundTrip = worldToBodyPoint(pointWorld, pose.body);

  assert.ok(Math.abs(roundTrip.x - pointBody.x) < 1e-9);
  assert.ok(Math.abs(roundTrip.y - pointBody.y) < 1e-9);
  assert.ok(Math.abs(roundTrip.z - pointBody.z) < 1e-9);
});

test("body angle edits keep feet locked in world coordinates", () => {
  const pose = createNeutralStaticPose();
  const next = updateBodyKeepingFeetLocked(pose, {
    rotationDeg: { roll: 8, pitch: -7, yaw: 12 },
  });

  assert.deepEqual(next.feetWorldMm, pose.feetWorldMm);

  const before = solveStaticPose(pose).legs.front_left.deltaBodyMm;
  const after = solveStaticPose(next).legs.front_left.deltaBodyMm;
  assert.notEqual(after.z, before.z);
});

test("neutral feet are anchored from the URDF thigh pivots", () => {
  const pose = createNeutralStaticPose();

  for (const legId of Object.keys(LEG_THIGH_PIVOTS_MM)) {
    assert.equal(pose.feetWorldMm[legId].y, LEG_THIGH_PIVOTS_MM[legId].y);
    assert.ok(pose.feetWorldMm[legId].z < LEG_THIGH_PIVOTS_MM[legId].z);
  }
});

test("foot edits only update the selected leg target", () => {
  const pose = createNeutralStaticPose();
  const next = updateFootWorld(pose, "front_right", { x: pose.feetWorldMm.front_right.x + 12 });

  assert.equal(next.feetWorldMm.front_right.x, pose.feetWorldMm.front_right.x + 12);
  assert.deepEqual(next.feetWorldMm.front_left, pose.feetWorldMm.front_left);
  assert.deepEqual(next.feetWorldMm.rear_left, pose.feetWorldMm.rear_left);
  assert.deepEqual(next.feetWorldMm.rear_right, pose.feetWorldMm.rear_right);
});

test("leg-plane foot edits map back to the same solver target", () => {
  const pose = createNeutralStaticPose();
  const original = solveStaticPose(pose).legs.front_left.footCommand;
  const footWorldMm = legPlaneFootToWorld(pose, "front_left", original, 12);
  const next = updateFootWorld(pose, "front_left", footWorldMm);
  const solved = solveStaticPose(next).legs.front_left;

  assert.ok(Math.abs(solved.footCommand.x - original.x) < 1e-9);
  assert.ok(Math.abs(solved.footCommand.y - original.y) < 1e-9);
  assert.ok(Math.abs(solved.targetHipYawDeg - 12) < 1e-9);
});

test("solved pose produces a flattened full-body servo command", () => {
  const solved = solveStaticPose(createNeutralStaticPose());
  const command = createPoseApplyCommand(solved);

  assert.equal(command.type, "apply_full_body_pose");
  for (const prefix of ["FL", "FR", "RL", "RR"]) {
    assert.equal(typeof command[`${prefix}HipYawDeg`], "number");
    assert.equal(typeof command[`${prefix}ThighDeg`], "number");
    assert.equal(typeof command[`${prefix}CalfDeg`], "number");
  }
});

test("URDF joint values use calibrated visual deltas from the CAD pose", () => {
  const solved = solveStaticPose(createNeutralStaticPose());
  const values = getUrdfJointValues(solved);

  for (const jointName of [
    "FL_femur_joint",
    "FL_tibia_joint",
    "FR_femur_joint",
    "FR_tibia_joint",
    "RL_femur_joint",
    "RL_tibia_joint",
    "RR_femur_joint",
    "RR_tibia_joint",
  ]) {
    assert.ok(Math.abs(values[jointName]) < 0.36, `${jointName} is unexpectedly large`);
  }
});

test("URDF tibia keeps bending when the 2D calf angle opens past -138 degrees", () => {
  const neutral = createNeutralStaticPose();
  const opened = updateFootWorld(neutral, "front_left", {
    x: neutral.feetWorldMm.front_left.x + 40,
  });
  const neutralSolved = solveStaticPose(neutral);
  const openedSolved = solveStaticPose(opened);
  const neutralValues = getUrdfJointValues(neutralSolved);
  const openedValues = getUrdfJointValues(openedSolved);

  assert.ok(openedSolved.legs.front_left.desired.jointAnglesDeg.calf > -138);
  assert.ok(
    openedValues.FL_tibia_joint < neutralValues.FL_tibia_joint - 0.05,
    "visual tibia should keep tracking the preview calf angle past the neutral bend range",
  );
});

test("URDF visual calf keeps opening beyond the servo-limited 2D solution", () => {
  const neutral = createNeutralStaticPose();
  const low = updateFootWorld(neutral, "front_left", {
    z: neutral.feetWorldMm.front_left.z - 60,
  });
  const lower = updateFootWorld(neutral, "front_left", {
    z: neutral.feetWorldMm.front_left.z - 120,
  });
  const lowSolved = solveStaticPose(low);
  const lowerSolved = solveStaticPose(lower);
  const lowValues = getUrdfJointValues(lowSolved);
  const lowerValues = getUrdfJointValues(lowerSolved);

  assert.ok(lowSolved.legs.front_left.desired.servoAnglesDeg.calf >= 179.9);
  assert.equal(lowerSolved.legs.front_left.reachable, false);
  assert.ok(Math.abs(lowerSolved.legs.front_left.desired.jointAnglesDeg.calf + 114) < 1);
  assert.ok(
    lowerValues.FL_tibia_joint < lowValues.FL_tibia_joint - 0.4,
    "visual tibia should keep opening after the 2D command solver reaches its calf limit",
  );
});

test("preview joints keep opening after the hardware calf solver saturates", () => {
  const neutral = createNeutralStaticPose();
  const lower = updateFootWorld(neutral, "front_left", {
    z: neutral.feetWorldMm.front_left.z - 120,
  });
  const solved = solveStaticPose(lower);
  const previewAngles = solved.legs.front_left.preview.jointAnglesDeg;
  const commandAngles = solved.legs.front_left.desired.jointAnglesDeg;

  assert.ok(Math.abs(commandAngles.calf + 114) < 1);
  assert.ok(previewAngles.calf > commandAngles.calf + 20);
  assert.ok(previewAngles.thigh < commandAngles.thigh - 20);
});

test("all four legs share the same bent and extended visual-fit regimes", () => {
  const neutral = createNeutralStaticPose();
  const bentPatch = (legId) => ({ x: neutral.feetWorldMm[legId].x + 40 });
  const extendedPatch = (legId) => ({ z: neutral.feetWorldMm[legId].z - 90 });

  for (const legId of Object.keys(TEST_LEG_PREFIXES)) {
    const bentPose = solveLegPose(neutral, legId, bentPatch(legId));
    const extendedPose = solveLegPose(neutral, legId, extendedPatch(legId));
    const bentError = urdfFootErrorMm(bentPose, legId);
    const extendedError = urdfFootErrorMm(extendedPose, legId);

    assert.equal(bentPose.legs[legId].visualFitMode, "bent");
    assert.equal(extendedPose.legs[legId].visualFitMode, "extended");
    assert.ok(bentError < 2.5, `${legId} bent-range fit drifted to ${bentError.toFixed(2)} mm`);
    assert.ok(
      extendedError < 1.5,
      `${legId} straight-range fit drifted to ${extendedError.toFixed(2)} mm`,
    );
    assert.ok(
      bentPose.legs[legId].visualFitErrorMm.extended > bentPose.legs[legId].visualFitErrorMm.final,
      `${legId} bent candidate did not beat the extended candidate in the bent range`,
    );
  }
});

test("mirrored leg pairs stay aligned through both visual-fit regimes", () => {
  const neutral = createNeutralStaticPose();
  const scenarios = [
    {
      name: "bent",
      buildPatch: (legId) => ({ x: neutral.feetWorldMm[legId].x + 40 }),
      expectedMode: "bent",
      maxErrorDeltaMm: 0.4,
      maxJointDeltaRad: 0.08,
    },
    {
      name: "extended",
      buildPatch: (legId) => ({ z: neutral.feetWorldMm[legId].z - 90 }),
      expectedMode: "extended",
      maxErrorDeltaMm: 0.2,
      maxJointDeltaRad: 0.12,
    },
  ];
  const pairs = [
    ["front_left", "front_right"],
    ["rear_left", "rear_right"],
  ];

  for (const scenario of scenarios) {
    for (const [leftLegId, rightLegId] of pairs) {
      const leftPose = solveLegPose(neutral, leftLegId, scenario.buildPatch(leftLegId));
      const rightPose = solveLegPose(neutral, rightLegId, scenario.buildPatch(rightLegId));
      const leftError = urdfFootErrorMm(leftPose, leftLegId);
      const rightError = urdfFootErrorMm(rightPose, rightLegId);
      const leftJoints = getVisualJointTriplet(leftPose, leftLegId);
      const rightJoints = getVisualJointTriplet(rightPose, rightLegId);

      assert.equal(leftPose.legs[leftLegId].visualFitMode, scenario.expectedMode);
      assert.equal(rightPose.legs[rightLegId].visualFitMode, scenario.expectedMode);
      assert.ok(
        Math.abs(leftError - rightError) < scenario.maxErrorDeltaMm,
        `${scenario.name} mirrored error drifted by ${Math.abs(leftError - rightError).toFixed(3)} mm for ${leftLegId}/${rightLegId}`,
      );
      assert.ok(
        maxMirroredMagnitudeDelta(leftJoints, rightJoints) < scenario.maxJointDeltaRad,
        `${scenario.name} mirrored joint magnitudes drifted too far for ${leftLegId}/${rightLegId}`,
      );
    }
  }
});

test("all four legs keep a tight URDF fit through the blended transition band", () => {
  const neutral = createNeutralStaticPose();

  for (const legId of Object.keys(TEST_LEG_PREFIXES)) {
    const bentEdge = solveLegPose(neutral, legId, {
      z: neutral.feetWorldMm[legId].z - 48,
    });
    const bentToBlend = solveLegPose(neutral, legId, {
      z: neutral.feetWorldMm[legId].z - 49,
    });
    const blendToExtended = solveLegPose(neutral, legId, {
      z: neutral.feetWorldMm[legId].z - 80,
    });
    const extendedEdge = solveLegPose(neutral, legId, {
      z: neutral.feetWorldMm[legId].z - 81,
    });
    const bentJoints = getVisualJointTriplet(bentEdge, legId);
    const bentToBlendJoints = getVisualJointTriplet(bentToBlend, legId);
    const blendToExtendedJoints = getVisualJointTriplet(blendToExtended, legId);
    const extendedJoints = getVisualJointTriplet(extendedEdge, legId);
    const bentToBlendJump = Math.max(
      Math.abs(bentJoints[0] - bentToBlendJoints[0]),
      Math.abs(bentJoints[1] - bentToBlendJoints[1]),
      Math.abs(bentJoints[2] - bentToBlendJoints[2]),
    );
    const blendToExtendedJump = Math.max(
      Math.abs(blendToExtendedJoints[0] - extendedJoints[0]),
      Math.abs(blendToExtendedJoints[1] - extendedJoints[1]),
      Math.abs(blendToExtendedJoints[2] - extendedJoints[2]),
    );

    assert.equal(bentEdge.legs[legId].visualFitMode, "bent");
    assert.equal(bentToBlend.legs[legId].visualFitMode, "blended");
    assert.equal(blendToExtended.legs[legId].visualFitMode, "blended");
    assert.equal(extendedEdge.legs[legId].visualFitMode, "extended");
    assert.ok(bentToBlendJump < 0.03, `${legId} snapped entering the blended range`);
    assert.ok(blendToExtendedJump < 0.21, `${legId} snapped entering the extended range`);
  }
});
