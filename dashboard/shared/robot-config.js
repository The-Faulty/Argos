// SOURCE OF TRUTH: values mirror ros2_ws/argos_control/Config.py and firmware/esp32c6/main/main.c. A unit test (tests/servo_cal.test.js) diffs them at build time.
//
// This module is the single definition of Argos 3-DOF quadruped geometry for
// the dashboard stack. It is imported unchanged from Node (pi_server) and the
// browser (React app). Keep it a pure ES module: no Node-only APIs, no DOM.

const DEG2RAD = Math.PI / 180.0;

// ─── Leg / joint ordering (mirrors ros_support.LEG_ORDER + JOINT_NAMES) ───
// Every (3, 4) angle matrix in the stack follows FR / FL / RR / RL.
export const LEG_IDS = ["FR", "FL", "RR", "RL"];

// Joint rows inside each leg (coxa/femur/tibia).
export const JOINT_ROWS = ["coxa", "femur", "tibia"];

// 12-element JOINT_NAMES — must match ros_support.JOINT_NAMES literally
// (order matters, the ESP32 firmware looks these up by exact string).
export const JOINT_NAMES = [
  "FR_coxa_joint", "FR_femur_joint", "FR_tibia_joint",
  "FL_coxa_joint", "FL_femur_joint", "FL_tibia_joint",
  "RR_coxa_joint", "RR_femur_joint", "RR_tibia_joint",
  "RL_coxa_joint", "RL_femur_joint", "RL_tibia_joint",
];

// Human-readable labels for the UI. Keys match LEG_IDS.
export const LEG_LABELS = {
  FR: "Front Right",
  FL: "Front Left",
  RR: "Rear Right",
  RL: "Rear Left",
};

// ─── Leg geometry (meters, mirrors Config.py leg geometry block) ─────────
// Used by Kinematics.py's _sagittal_fk_state / _sagittal_ik. Values are the
// literal Config.py numbers — changing one here without updating Config.py
// will break the build-time diff test.
export const LEG_GEOMETRY = {
  // Hip abductor
  L1_HIP: 0.035563,
  PHI: 20.156 * DEG2RAD,

  // Sagittal links
  L_UPPER: 0.127063,
  L_LOWER: 0.127183,

  // Bell-crank (couples bottom servo to knee)
  R_BC_LEFT: 0.04,
  R_BC_RIGHT: 0.04,
  ALPHA_BC: 90.0 * DEG2RAD,

  // Push-rods + horn radius
  L_ROD1: 0.03,
  L_ROD2: 0.150,
  R_HORN: 0.024,

  // Rod 2 attach point above the knee pivot
  D_KNEE_OFFSET: 0.030023,
};

// Alias table consumed by Kinematics.py::leg_params. Keep these keys stable
// because argos_kinematics.js ports the Python implementation character-for-
// character; renaming a key here means renaming it everywhere in the IK chain.
export const LEG_PARAMS = {
  L1_hip: LEG_GEOMETRY.L1_HIP,
  phi:    LEG_GEOMETRY.PHI,
  L1:     LEG_GEOMETRY.L_UPPER,
  L2:     LEG_GEOMETRY.L_LOWER,
  rbl:    LEG_GEOMETRY.R_BC_LEFT,
  rbr:    LEG_GEOMETRY.R_BC_RIGHT,
  abc:    LEG_GEOMETRY.ALPHA_BC,
  Lr1:    LEG_GEOMETRY.L_ROD1,
  Lr2:    LEG_GEOMETRY.L_ROD2,
  rh:     LEG_GEOMETRY.R_HORN,
  dko:    LEG_GEOMETRY.D_KNEE_OFFSET,
};

// ─── Leg drawing (SVG preview stage) ─────────────────────────────────────
// Ported verbatim from andy-servo-control's robot-config.js. The rendering
// pipeline in `kinematics_legacy.js` operates in millimeter units (the same
// units andy-servo's geometry uses), so `scale=1.4` gives a leg diagram that
// matches the reference visuals pixel-for-pixel. `offset.y = 170` is the
// reference full-body pose dashboard's 420×420 transform.
//
// The dashboard's 3-DOF real-robot kinematics live in meters, but those
// values never reach this transform — LegDetail only feeds the solved
// sagittal geometry from `buildLegPoseFromJointAngles` (millimeters) through
// here. If you ever change `scale`, update the andy-servo reference too.
export const LEG_DRAWING = {
  scale: 1.4,
  stageWidth: 420,
  stageHeight: 420,
  offset: { x: 210, y: 170 },
};

// Simple top-down layout used by RobotOverview. Body box is 280×110 in px,
// anchors place each leg's hip relative to body center.
export const ROBOT_LAYOUT = {
  body: { width: 280, height: 110 },
  anchors: {
    FL: { x: -100, y: -40 },
    FR: {  x: 100, y: -40 },
    RL: { x: -100, y:  40 },
    RR: {  x: 100, y:  40 },
  },
};

// ─── Hip origins in body frame (Config.py LEG_ORIGINS) ───────────────────
// Matrix shape (3, 4): rows = x/y/z, columns = FR/FL/RR/RL.
// x = forward (body), y = left (body), z = up (body). The signs mirror
// Config.py: front legs have +LEG_FB, rear -LEG_FB; right legs have -LEG_LR,
// left legs +LEG_LR.
const _LEG_FB = 0.11165;
const _LEG_LR = 0.061;
export const LEG_ORIGINS = [
  [ _LEG_FB,  _LEG_FB, -_LEG_FB, -_LEG_FB], // x
  [-_LEG_LR,  _LEG_LR, -_LEG_LR,  _LEG_LR], // y
  [ 0,        0,        0,        0       ], // z
];

// ─── Joint limits (radians) ───────────────────────────────────────────────
// Mirrors Config.JOINT_LIMITS_DEG → radians. These are control-space limits
// relative to each servo's 90° neutral.
export const DEFAULT_JOINT_LIMITS_RAD = {
  coxa:  [-45.0 * DEG2RAD,  45.0 * DEG2RAD],
  femur: [-90.0 * DEG2RAD,   0.0 * DEG2RAD],
  tibia: [-90.0 * DEG2RAD,  90.0 * DEG2RAD],
};

// ─── Per-leg foot reach window (meters, body frame x) ─────────────────────
// Empirical IK-success windows from running fourLegsInverseKinematics in a
// sweep at the gait planner's actual stance (gait_planner STAND_FOOT_Y =
// ±0.0733 m, STAND_FOOT_Z = -0.190 m). Bounds are pulled in 2 mm from the
// measured edge of feasibility (probed at 0.5 mm resolution by
// scripts/probe_foot_reach.mjs) so the IK doesn't sit on the cusp where a
// sub-millimeter IMU-tilt or rounding nudge flips reachable → unreachable.
// Re-probe with `node scripts/probe_foot_reach.mjs --z-mm <stance_mm>`
// after any joint-limit revision, body-height change, or stance-y change.
//
// The gait planner uses these to clamp stance-phase foot targets and
// setFootTargets uses them too. Swing-phase foot targets use the
// _LIFTED window below — see gait_planner.clampFootInReach.
export const DEFAULT_FOOT_REACH_X = {
  FR: [ 0.056,  0.171],
  FL: [ 0.056,  0.171],
  RR: [-0.167, -0.053],
  RL: [-0.167, -0.053],
};

// ─── Per-leg lifted-foot reach window ─────────────────────────────────────
// Same probe as DEFAULT_FOOT_REACH_X but at swing-peak z (stance_z +
// z_clearance = -0.190 + 0.018 = -0.172 m). Used to clamp swing-phase foot
// targets only — at the lifted z the femur has different reach geometry
// (the linkage extends differently), so the reachable x window shifts and
// is significantly wider. Without this, swing-phase clamping was using the
// stance window and silently shrinking step amplitude as soon as the
// trajectory tried to put the foot past the stance edge.
//
// Re-probe with `node scripts/probe_foot_reach.mjs --z-mm <stance_z_mm>
// --lift-mm <z_clearance_mm>` after any z_clearance / body-height change.
export const DEFAULT_FOOT_REACH_X_LIFTED = {
  FR: [ 0.018,  0.198],
  FL: [ 0.018,  0.198],
  RR: [-0.206, -0.025],
  RL: [-0.206, -0.025],
};

// ─── Servo channel map (mirrors firmware s_servo_cal) ────────────────────
// 12 entries, channels 0..11, FR/FL/RR/RL × coxa/femur/tibia.
export const DEFAULT_SERVO_CHANNEL_MAP = {
  FR: { coxa: 0,  femur: 1,  tibia: 2  },
  FL: { coxa: 3,  femur: 4,  tibia: 5  },
  RR: { coxa: 6,  femur: 7,  tibia: 8  },
  RL: { coxa: 9,  femur: 10, tibia: 11 },
};

// ─── Coupled-linkage lookup tables (degrees) ─────────────────────────────
// Piecewise samples tying the top-servo angle to the allowed bottom-servo
// window. Consumers interpolate the same way Config.coupled_bot_limits_*
// does (np.interp over COUPLED_TOP_SAMPLES_DEG as the x-axis).
//
// Probed via scripts/limit_finder.py. Stored in servo-deg space (90 = neutral).
// Joint-space pairs measured at the safe-region boundary:
//   femur=-50:  tibia ∈ [-90, +5]
//   femur=-38:  tibia ∈ [-90, +30]
//   femur=-35:  tibia can still reach -90
//   femur=-25:  tibia ∈ [-70, +60]
//   femur=-13:  tibia ∈ [-50, +90]
//   femur= 0:   tibia ∈ [-25, +90]
//   femur=-90:  tibia=-90
export const COUPLED_TOP_SAMPLES_DEG     = [ 0.0,  40.0,  52.0,  55.0,  65.0,  77.0,  90.0];
export const COUPLED_BOT_MIN_SAMPLES_DEG = [ 0.0,   0.0,   0.0,   0.0,  20.0,  40.0,  65.0];
export const COUPLED_BOT_MAX_SAMPLES_DEG = [ 0.0,  95.0, 120.0, 127.0, 150.0, 180.0, 180.0];

// ─── Servo-space horn limits (mirrors Config.SERVO_LIMITS_DEG) ───────────
// Bench-measured direct-servo travel with the linkage assembled. Absolute
// horn commands in the servo's native 0..180 deg space, per joint row
// (hip / top / bottom = coxa / femur / tibia).
export const SERVO_CENTER_DEG = 90.0;
export const SERVO_LIMITS_DEG = [
  [45.0, 135.0],   // hip (coxa)        — joint ±45
  [ 0.0,  90.0],   // top (femur)       — joint -90..0   (coupled with tibia)
  [ 0.0, 180.0],   // bottom (tibia)    — joint -90..+90 (probed via limit_finder)
];

// ─── Default stance (meters, body frame) ─────────────────────────────────
// Config.py exposes default_stance (footprint at z=0) and default_z_ref; the
// actual neutral pose adds z_ref to the z row. We expose both the raw stance
// and the composite XYZ per leg for convenience.
const _delta_x = 0.100;
const _delta_y = 0.1106;
const _front_leg_x_shift = 0.00;
const _rear_leg_x_shift  = -0.04;
const _default_z_ref = -0.189;

export const DEFAULT_STANCE = {
  FR: [  _delta_x + _front_leg_x_shift, -_delta_y, _default_z_ref ],
  FL: [  _delta_x + _front_leg_x_shift,  _delta_y, _default_z_ref ],
  RR: [ -_delta_x + _rear_leg_x_shift,  -_delta_y, _default_z_ref ],
  RL: [ -_delta_x + _rear_leg_x_shift,   _delta_y, _default_z_ref ],
};

export const DEFAULT_Z_REF = _default_z_ref;

// ─── Default animation clip ───────────────────────────────────────────────
// Used by animation.js as the fallback when a user-uploaded clip has no
// duration or an unparseable one. Keeping this tiny on purpose: the
// dashboard only consumes `.duration` off it.
export const DEFAULT_FULL_BODY_CLIP = {
  version: 2,
  name: "default-clip",
  duration: 2.0,
  tracks: { FR: [], FL: [], RR: [], RL: [] },
};

// ─── Available runtime modes (dashboard dropdown + pi_server validation) ──
// Must match gait_planner_node.py's accepted set literally — `extend`, not
// `extended`. A test in `robot_config.test.js` asserts this stays in sync.
export const MODE_OPTIONS = [
  "idle",
  "crouch",
  "stand",
  "extend",
  "crawl",
  "trot",
  "direct_foot_xyz",
  "direct_joint_angles",
  "direct_servo_angles",
  "animation_playback",
];

// ─── Servo speed + PWM rate defaults (ported from andy-servo-control) ────
// Tibia raised to 360°/s (was 240) so the per-tick cap (40 ms × 360 = 14.4°)
// covers the ~14.6°/tick peak demand at the swing→stance handoff. Below
// that, the limiter throttles the foot's landing arc to ~65% of commanded
// and the visible step shrinks. Femur stays at 180 (peak demand 6°/tick is
// well inside cap) and coxa at 120 (only active during yaw/strafe). The
// upper bound on each row is SERVO_SPEED_LIMIT_BOUNDS_DEG_PER_SEC.max
// (600); going past 360 risks servo stall under load.
export const DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC = {
  coxa:  120.0,
  femur: 180.0,
  tibia: 360.0,
};
export const SERVO_SPEED_LIMIT_BOUNDS_DEG_PER_SEC = { min: 30.0, max: 600.0 };

export const DEFAULT_SERVO_UPDATE_RATE_HZ = 50.0;
export const SERVO_UPDATE_RATE_BOUNDS_HZ  = { min: 40.0, max: 200.0 };

// ─── Rotate-in-place settings ─────────────────────────────────────────────
// Bounds/defaults for the "rotate by N degrees" buttons. Persisted on the Pi
// side in ~/.argos/rotate_settings.json; surfaced in the Settings drawer.
export const DEFAULT_ROTATE_SETTINGS = {
  rotate_increment_deg: 20.0,
  rotate_rate_rad_s: 1.4,
  rotate_calibration_factor: 1.12,
};
export const ROTATE_SETTINGS_BOUNDS = {
  rotate_increment_deg:       { min: 5,    max: 45 },
  rotate_rate_rad_s:          { min: 0.2,  max: 2.5 },
  rotate_calibration_factor:  { min: 0.80, max: 1.50 },
};

// ─── Stabilizer param bounds (dashboard Settings drawer) ──────────────────
export const STABILIZER_PARAM_BOUNDS = {
  stabilization_roll_gain:          { min: 0.0, max: 2.0, default: 0.5 },
  stabilization_pitch_gain:         { min: 0.0, max: 2.0, default: 0.5 },
  stabilization_max_correction_rad: { min: 0.0, max: 0.5, default: 0.2 },
  imu_filter_alpha:                 { min: 0.0, max: 1.0, default: 0.2 },
};

// ─── Live-tuneable gait parameters ────────────────────────────────────────
export const GAIT_TUNABLE_PARAMS = {
  delta_x_mm: {
    name: "delta_x_mm",
    label: "Front/back stance spread",
    min: 60,
    // IK-reachable upper bound at default_z_ref = -189 mm. The femur joint
    // limit ([-50°, 0°]) makes the per-leg foot.x window only ~75 mm wide
    // (see DEFAULT_FOOT_REACH_X). Front feet are at delta_x; rear feet at
    // -delta_x + rear_shift (-0.04 m), so the rear bound is the binding
    // constraint: -delta_x - 0.04 ≥ -0.145 → delta_x ≤ 0.105. Cap matches
    // that so both front and rear stance feet stay inside their reach
    // windows at slider max.
    max: 105,
    default: 100,
    units: "mm",
  },
  delta_y_mm: {
    name: "delta_y_mm",
    label: "Left/right stance spread",
    min: 80,
    max: 140,
    default: 110.6,
    units: "mm",
  },
  swing_time_ms: {
    name: "swing_time_ms",
    label: "Swing time per foot",
    min: 40,
    // Capped at 500 ms so the operator can slow the gait further if servos
    // aren't keeping up. Default 300 ms spreads the per-swing tibia demand
    // across ~7 ticks at 25 Hz instead of ~5, which keeps the peak servo
    // delta inside the 240°/s × 40 ms tick budget and lets each step
    // visibly complete its commanded arc on the real robot.
    max: 500,
    default: 300,
    units: "ms",
  },
  rotate_rate_max: {
    name: "rotate_rate_max",
    label: "Max yaw rate",
    min: 0.1,
    max: 3.0,
    default: 2.0,
    units: "rad/s",
  },
  default_z_ref_mm: {
    name: "default_z_ref_mm",
    label: "Body height",
    min: -230,
    max: -140,
    default: -189,
    units: "mm",
  },
};

// ─── Joystick → SI mapping ────────────────────────────────────────────────
// Joystick UI emits unit-vector deflection in [-1, 1]. The planner clamps
// twist at ±0.6 m/s (gait_planner.setTwist) and ±rotate_rate_max rad/s, so
// MAX_LIN_VEL must stay ≤ 0.6. DEADZONE is a radial fraction — any |stick|
// below it sends zero so a slightly off-center stick doesn't dribble
// commands.
//
// MAX_LIN_VEL is the velocity the planner sees on a full-deflection arrow
// hold. The ceiling is the servo speed budget (240°/s × 40 ms = 9.6°/tick
// for tibia) and the IK branch-flip cliff above ~0.24 m/s, where the foot
// drops faster than the hint-continuous tibia angle can track and the IK
// falls back to the global sweep on the wrong branch.
//
// 0.16 paired with the 300 ms swing default delivers ~61 mm of visible
// horizontal swing arc — bigger steps than the older 0.20 / 220 ms combo
// because the longer swing spreads the per-tick servo demand thinner so
// each tick stays inside the 9.6° cap and the commanded arc actually gets
// realized on the hardware.
export const JOYSTICK_SCALE = {
  MAX_LIN_VEL: 0.16, // m/s at full deflection (arrow-button hold)
  MAX_ANG_VEL: 1.4,  // rad/s; reserved for yaw stick
  DEADZONE: 0.08,    // normalized fraction (unused for arrow buttons)
};
