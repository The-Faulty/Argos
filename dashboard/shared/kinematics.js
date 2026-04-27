// Barrel module re-exporting the kinematics helpers the dashboard actually
// consumes. Splitting the implementation keeps each concern isolated:
//
//   - argos_kinematics.js → 3-DOF real-robot IK (meters, Python-authoritative).
//     Used by the Pi server when it publishes to `/joint_command/raw`.
//   - kinematics_legacy.js → andy-servo's 2-DOF sagittal bell-crank pipeline
//     (pixel-space mm, math frame with y-flip). Used by LegDetail +
//     RobotOverview for rendering — per user direction the leg diagram must
//     match andy-servo's visual exactly, which means using andy-servo's FK,
//     reverse-solve, and canvas helpers byte-for-byte.
//
// Nothing here contains logic — just re-exports. Consumers should import
// from this barrel so the choice of source module stays encapsulated.

// 3-DOF Argos IK (real-robot path)
export {
  fourLegsInverseKinematics,
  leg_explicit_inverse_kinematics,
  leg_fk,
  _sagittal_fk_state,
  _circle_intersect,
  clamp_joint_matrix,
  coupled_bot_limits_joint_deg,
  coupled_bot_limits_joint_rad,
  coupled_bot_limits_servo_deg,
  coupled_top_limits_joint_deg,
  coupled_top_limits_joint_rad,
} from "./argos_kinematics.js";

// Andy-servo rendering pipeline (LegDetail + RobotOverview consume these)
export {
  buildLegPoseFromJointAngles,
  solveGeometry,
  solveGeometryLegacy,
  solveServoForJointAngles,
  createNeutralCalibration,
  normalizeJointLimits,
  clampJointAnglesToLimits,
  geometryWithinJointLimits,
  modelServoDegToTheta,
  thetaToModelServoDeg,
  toCanvasPoint,
  fromCanvasPoint,
  segmentPath,
  circleIntersections,
  radToDeg,
  degToRad,
  clampAngle,
  LEGACY_LEG_GEOMETRY,
  LEGACY_JOINT_LIMITS,
  LEGACY_NEUTRAL_CALIBRATION,
} from "./kinematics_legacy.js";
