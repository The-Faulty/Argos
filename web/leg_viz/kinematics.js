// Forward kinematics for the Argos 3-DOF leg, ported from
// ros2_ws/argos_control/Kinematics.py so the browser can render the
// bell-crank linkage exactly the same way the robot computes it.
//
// All lengths in meters, all angles in radians. Sagittal x points forward
// along the body; sagittal y points downward from the hip.

export const LEG_PARAMS = {
  // Hip abductor (coxa): servo shaft -> femur pivot.
  L1_hip: 0.035563,
  phi: (20.156 * Math.PI) / 180,

  // Sagittal links.
  L1: 0.127063,   // upper leg: top servo -> knee
  L2: 0.127183,   // lower leg: knee -> foot

  // Bell-crank coupled linkage.
  rbl: 0.04,
  rbr: 0.04,
  abc: Math.PI / 2,
  Lr1: 0.03,
  Lr2: 0.150,
  rh: 0.024,
  dko: 0.030023,
};

// Joint limits (rad). theta_top upper range is widened beyond Config.py's
// +10° to accommodate the crouch preset (servo 115 → control +25°), which
// the physical robot reaches without issue.
export const JOINT_LIMITS = {
  theta1: [(-45 * Math.PI) / 180, (45 * Math.PI) / 180],
  theta_top: [(-75 * Math.PI) / 180, (30 * Math.PI) / 180],
  theta_bot: [(-20 * Math.PI) / 180, (80 * Math.PI) / 180],
};

export function deg(rad) {
  return (rad * 180) / Math.PI;
}
export function rad(deg) {
  return (deg * Math.PI) / 180;
}

function circleIntersect(P1, r1, P2, r2) {
  const dx = P2[0] - P1[0];
  const dy = P2[1] - P1[1];
  const d = Math.sqrt(dx * dx + dy * dy);
  if (d < 1e-9 || d > r1 + r2 + 1e-9 || d < Math.abs(r1 - r2) - 1e-9) {
    return null;
  }
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const mx = P1[0] + (a * dx) / d;
  const my = P1[1] + (a * dy) / d;
  const px = -dy / d;
  const py = dx / d;
  return [
    [mx + h * px, my + h * py],
    [mx - h * px, my - h * py],
  ];
}

function mod2pi(x) {
  const two = 2 * Math.PI;
  return ((x % two) + two) % two;
}

// Port of _sagittal_fk_state from Kinematics.py.
// Returns every joint in the sagittal plane so the renderer can draw
// each linkage segment. Returns null if the linkage is infeasible.
export function sagittalFkState(thetaTop, thetaBot, P) {
  const L1 = P.L1;
  const L2 = P.L2;
  const O = [0, 0];

  const knee = [L1 * Math.sin(thetaTop), L1 * Math.cos(thetaTop)];
  const horn = [P.rh * Math.sin(thetaBot), P.rh * Math.cos(thetaBot)];

  let pair = circleIntersect(O, P.rbr, horn, P.Lr1);
  if (!pair) return null;
  const [A1, B1] = pair;

  const bclX = (c) => Math.sin(Math.atan2(c[0], c[1]) + P.abc);
  const bc_right = bclX(A1) < bclX(B1) ? A1 : B1;

  const phi_r = Math.atan2(bc_right[0], bc_right[1]);
  const phi_l = phi_r + P.abc;
  const bc_left = [P.rbl * Math.sin(phi_l), P.rbl * Math.cos(phi_l)];

  pair = circleIntersect(knee, P.dko, bc_left, P.Lr2);
  if (!pair) return null;
  const [A2, B2] = pair;

  const diff = (c) => {
    const legDir = [knee[0] - c[0], knee[1] - c[1]];
    const delta = mod2pi(Math.atan2(legDir[0], legDir[1]) - thetaTop);
    return delta > Math.PI ? delta - 2 * Math.PI : delta;
  };
  const dA = diff(A2);
  const dB = diff(B2);
  let rod2;
  if (dA > 0 && dA < Math.PI) rod2 = A2;
  else if (dB > 0 && dB < Math.PI) rod2 = B2;
  else rod2 = Math.abs(dA - Math.PI / 2) < Math.abs(dB - Math.PI / 2) ? A2 : B2;

  const dirX = knee[0] - rod2[0];
  const dirY = knee[1] - rod2[1];
  const n = Math.hypot(dirX, dirY);
  const lowerDir = [dirX / n, dirY / n];
  const foot = [knee[0] + L2 * lowerDir[0], knee[1] + L2 * lowerDir[1]];
  const lowerAngle = Math.atan2(lowerDir[0], lowerDir[1]);

  return {
    hipPivot: [0, 0],   // femur pivot in sagittal frame
    knee,
    hornTip: horn,
    bellCrankRight: bc_right,
    bellCrankLeft: bc_left,
    rod2Attach: rod2,
    foot,
    lowerAngle,
  };
}

// 3x3 rotation about X.
function rotX(a) {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [
    [1, 0, 0],
    [0, c, -s],
    [0, s, c],
  ];
}

function mvec(M, v) {
  return [
    M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
    M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
    M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
  ];
}

// Lift a sagittal-plane 2D point into the 3D body frame for a given
// hip-abductor angle. This is the forward direction of the transform
// that _hip_abductor_ik in Kinematics.py runs in reverse:
//   sag = rotX(-R2) @ (p_body_in_R1_frame - coxaOffset)
// so the inverse is:
//   p_body_in_R1_frame = rotX(R2) @ sag + coxaOffset
//   p_body             = rotX(R1) @ p_body_in_R1_frame
// For right legs the final Y is negated (matches the `if is_right: y = -y`
// at the top of the IK).
//
// legIndex: 0=FR, 1=FL, 2=RR, 3=RL. Only the Y-flip differs per leg.
export function sagittalTo3D(p2, theta1, legIndex, P) {
  const R1 = Math.PI / 2 - P.phi;
  const R2 = theta1 + P.phi - Math.PI / 2;

  // Sagittal 2D (x_sag, y_sag) -> sagittal 3D: (x_sag, 0, -y_sag).
  const sag3 = [p2[0], 0, -p2[1]];

  // Inverse of rotX(-R2) is rotX(R2).
  const inter = mvec(rotX(R2), sag3);

  // Add the coxa offset (femur attachment relative to hip origin, in R1 frame).
  const off = [0, P.L1_hip * Math.cos(theta1), P.L1_hip * Math.sin(theta1)];
  const interOff = [inter[0] + off[0], inter[1] + off[1], inter[2] + off[2]];

  // Inverse of rotX(-R1) is rotX(R1).
  const body = mvec(rotX(R1), interOff);

  // Right legs (FR=0, RR=2) have a mirrored Y in the IK.
  const isRight = legIndex === 0 || legIndex === 2;
  if (isRight) body[1] = -body[1];

  return body;
}

// 3D position of the femur pivot (top of the sagittal chain) in the body
// frame — i.e. the point the sagittal plane is anchored at. This is just
// the coxa offset lifted out of the R1 frame.
export function hipAnchor3D(theta1, legIndex, P) {
  return sagittalTo3D([0, 0], theta1, legIndex, P);
}

// Full-leg FK: compute every joint in the 3D body frame. Returns null if
// the bell-crank linkage is infeasible for the given (thetaTop, thetaBot).
export function legFk3D(theta1, thetaTop, thetaBot, legIndex = 0, P = LEG_PARAMS) {
  const sag = sagittalFkState(thetaTop, thetaBot, P);
  if (!sag) return null;

  const lift = (p2) => sagittalTo3D(p2, theta1, legIndex, P);

  return {
    hipOrigin: [0, 0, 0],
    femurPivot: lift([0, 0]),
    knee: lift(sag.knee),
    hornTip: lift(sag.hornTip),
    bellCrankRight: lift(sag.bellCrankRight),
    bellCrankLeft: lift(sag.bellCrankLeft),
    rod2Attach: lift(sag.rod2Attach),
    foot: lift(sag.foot),
    footSag: sag.foot,           // (x_sag, y_sag) in meters, for readout
    lowerAngle: sag.lowerAngle,
  };
}

// Servo (0..270, centered at 90) <-> control-space radians. Mirrors
// _servo_deg_to_ik / _ik_to_servo_us from single_leg_test.py.
//
// Matches the defaults in single_leg_test.py: HIP_MULTIPLIER=-1 (hip
// servo runs backward relative to IK), TOP and BOT multipliers +1,
// all offsets zero. With these values, 90/90/90 servo → (0, 0, 0)
// control, which is the neutral stand pose the user operates at.
export const SERVO_CENTER_DEG = 90;

export const MULTIPLIERS = { hip: -1, top: +1, bot: +1 };
export const OFFSETS = { hip: 0, top: 0, bot: 0 };

export function servoDegToControlRad(servoDeg, mult, offset = 0) {
  return rad((servoDeg - SERVO_CENTER_DEG - offset) / mult);
}
export function controlRadToServoDeg(controlRad, mult, offset = 0) {
  return SERVO_CENTER_DEG + mult * deg(controlRad) + offset;
}
