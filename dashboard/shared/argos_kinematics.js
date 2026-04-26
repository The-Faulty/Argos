// Argos 3-DOF inverse + forward kinematics. This is the only IK in the
// stack — the Pi gait planner imports the same module the React dashboard
// uses for previews, so the angles you see in the UI are the angles that
// hit the servos. Golden-fixture tests in dashboard/tests/kinematics.test.js
// pin us to 1e-4 rad per joint against canned trajectories.
//
// All distances in meters. All angles in radians. Joint ordering follows
// LEG_IDS = ["FR", "FL", "RR", "RL"] and JOINT_ROWS = ["coxa","femur","tibia"]
// which the Python side calls (theta_1, theta_top, theta_bot).

import {
  LEG_PARAMS,
  LEG_ORIGINS,
  LEG_DRAWING,
  DEFAULT_JOINT_LIMITS_RAD,
  COUPLED_TOP_SAMPLES_DEG,
  COUPLED_BOT_MIN_SAMPLES_DEG,
  COUPLED_BOT_MAX_SAMPLES_DEG,
  SERVO_LIMITS_DEG,
  SERVO_CENTER_DEG,
} from "./robot-config.js";

const PI = Math.PI;
const DEG2RAD = PI / 180.0;
const RAD2DEG = 180.0 / PI;
const LEG_NAMES = ["FR", "FL", "RR", "RL"];

// ─── Low-level helpers ────────────────────────────────────────────────────

function _pointToRad(p1, p2) {
  // Mirrors (atan2(p2, p1) + 2π) % 2π in Kinematics.py.
  return (Math.atan2(p2, p1) + 2 * PI) % (2 * PI);
}

function _wrapToPi(angle) {
  return ((angle + PI) % (2 * PI) + 2 * PI) % (2 * PI) - PI;
}

function _rotXApply(angle, v) {
  // Returns _rotx(angle) @ v, i.e. 3x3 rotation about X applied to (x,y,z).
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]];
}

export function _circle_intersect(P1, r1, P2, r2) {
  // Port of Kinematics.py::_circle_intersect. Returns [A, B] or [null, null].
  const dx = P2[0] - P1[0];
  const dy = P2[1] - P1[1];
  const d = Math.sqrt(dx * dx + dy * dy);
  if (d < 1e-9 || d > r1 + r2 + 1e-9 || d < Math.abs(r1 - r2) - 1e-9) {
    return [null, null];
  }
  const a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
  const h = Math.sqrt(Math.max(0.0, r1 * r1 - a * a));
  const mx = P1[0] + (a * dx) / d;
  const my = P1[1] + (a * dy) / d;
  const px = -dy / d;
  const py =  dx / d;
  return [
    [mx + h * px, my + h * py],
    [mx - h * px, my - h * py],
  ];
}

// ─── Sagittal FK state ────────────────────────────────────────────────────

/**
 * Port of Kinematics.py::_sagittal_fk_state. Returns the foot (x, y) in the
 * sagittal plane plus every intermediate pivot — the dashboard renders all
 * 8 of these as the bell-crank linkage in LegDiagram.
 */
export function _sagittal_fk_state(thetaTop, thetaBot, params = LEG_PARAMS) {
  const { L1, L2, rh, rbr, rbl, Lr1, Lr2, dko, abc } = params;
  const O = [0, 0];

  const knee = [L1 * Math.sin(thetaTop), L1 * Math.cos(thetaTop)];
  const horn = [rh * Math.sin(thetaBot), rh * Math.cos(thetaBot)];

  let [A, B] = _circle_intersect(O, rbr, horn, Lr1);
  if (A === null) return null;

  const bclX = (c) => Math.sin(Math.atan2(c[0], c[1]) + abc);
  const bcRight = bclX(A) < bclX(B) ? A : B;

  const phiR = Math.atan2(bcRight[0], bcRight[1]);
  const phiL = phiR + abc;
  const bcLeft = [rbl * Math.sin(phiL), rbl * Math.cos(phiL)];

  [A, B] = _circle_intersect(knee, dko, bcLeft, Lr2);
  if (A === null) return null;

  const _diff = (c) => {
    const legDirX = knee[0] - c[0];
    const legDirY = knee[1] - c[1];
    const delta = ((Math.atan2(legDirX, legDirY) - thetaTop) % (2 * PI) + 2 * PI) % (2 * PI);
    return delta > PI ? delta - 2 * PI : delta;
  };
  const dA = _diff(A);
  const dB = _diff(B);
  let rod2;
  if (0 < dA && dA < PI) rod2 = A;
  else if (0 < dB && dB < PI) rod2 = B;
  else rod2 = Math.abs(dA - PI / 2) < Math.abs(dB - PI / 2) ? A : B;

  const legVx = knee[0] - rod2[0];
  const legVy = knee[1] - rod2[1];
  const legLen = Math.sqrt(legVx * legVx + legVy * legVy);
  const lowerDir = [legVx / legLen, legVy / legLen];
  const foot = [knee[0] + L2 * lowerDir[0], knee[1] + L2 * lowerDir[1]];

  return {
    O,
    knee,
    horn,
    bcRight,
    bcLeft,
    rod2,
    foot,
    lowerAngle: Math.atan2(lowerDir[0], lowerDir[1]),
  };
}

export function leg_fk(thetaTop, thetaBot, params = LEG_PARAMS) {
  const s = _sagittal_fk_state(thetaTop, thetaBot, params);
  if (s === null) return null;
  return [s.foot[0], s.foot[1]];
}

// ─── Coupled bell-crank envelope (Config.py::coupled_bot_limits_*) ────────

function _interp(x, xs, ys) {
  // np.interp equivalent for a monotonically-increasing xs array.
  if (x <= xs[0]) return ys[0];
  if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
  for (let i = 1; i < xs.length; i++) {
    if (x <= xs[i]) {
      const t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
      return ys[i - 1] + t * (ys[i] - ys[i - 1]);
    }
  }
  return ys[ys.length - 1];
}

export function coupled_bot_limits_servo_deg(topServoDeg, marginDeg = 0.0) {
  const [topLo, topHi] = SERVO_LIMITS_DEG[1];
  const clampedTop = Math.min(topHi, Math.max(topLo, topServoDeg));
  const m = Math.max(0.0, marginDeg);
  const [botLoHard, botHiHard] = SERVO_LIMITS_DEG[2];
  const botMinSamples = COUPLED_BOT_MIN_SAMPLES_DEG.map(
    (v) => Math.min(botHiHard, Math.max(botLoHard, v + m))
  );
  const botMaxSamples = COUPLED_BOT_MAX_SAMPLES_DEG.map(
    (v) => Math.min(botHiHard, Math.max(botLoHard, v - m))
  );
  let botMin = _interp(clampedTop, COUPLED_TOP_SAMPLES_DEG, botMinSamples);
  let botMax = _interp(clampedTop, COUPLED_TOP_SAMPLES_DEG, botMaxSamples);
  if (botMin > botMax) {
    const mid = 0.5 * (botMin + botMax);
    botMin = mid;
    botMax = mid;
  }
  return [botMin, botMax];
}

export function coupled_bot_limits_joint_deg(topJointDeg, marginDeg = 0.0) {
  const [botMinSrv, botMaxSrv] = coupled_bot_limits_servo_deg(
    SERVO_CENTER_DEG + topJointDeg,
    marginDeg
  );
  return [botMinSrv - SERVO_CENTER_DEG, botMaxSrv - SERVO_CENTER_DEG];
}

export function coupled_bot_limits_joint_rad(topJointRad, marginDeg = 0.0) {
  const [lo, hi] = coupled_bot_limits_joint_deg(topJointRad * RAD2DEG, marginDeg);
  return [lo * DEG2RAD, hi * DEG2RAD];
}

// ─── Joint limit clamping (Config.py::clamp_joint_matrix) ─────────────────

/**
 * Clamp a 4x3 matrix `angles[leg][joint]` (legs FR/FL/RR/RL, joints coxa/
 * femur/tibia) to the hardware envelope. Mirrors Config.clamp_joint_matrix
 * but with the more-JS-friendly 4x3 row-major layout the dashboard uses.
 */
export function clamp_joint_matrix(angles4x3, coupledMarginDeg = 0.0, limits = DEFAULT_JOINT_LIMITS_RAD) {
  const [hipLo, hipHi]   = limits.coxa;
  const [topLo, topHi]   = limits.femur;
  const [botLoG, botHiG] = limits.tibia;
  const clamped = angles4x3.map((leg) => leg.slice());
  for (let i = 0; i < clamped.length; i++) {
    clamped[i][0] = Math.min(hipHi, Math.max(hipLo, clamped[i][0]));
    clamped[i][1] = Math.min(topHi, Math.max(topLo, clamped[i][1]));
    let [botLo, botHi] = coupled_bot_limits_joint_rad(clamped[i][1], coupledMarginDeg);
    botLo = Math.max(botLoG, botLo);
    botHi = Math.min(botHiG, botHi);
    if (botLo > botHi) {
      const mid = 0.5 * (botLo + botHi);
      botLo = mid;
      botHi = mid;
    }
    clamped[i][2] = Math.min(botHi, Math.max(botLo, clamped[i][2]));
  }
  return clamped;
}

// ─── Hip abductor + sagittal IK ───────────────────────────────────────────

function _hipAbductorIK(rBodyFoot, legIndex, L1Hip, phi) {
  const isRight = legIndex === 0 || legIndex === 2; // FR=0, RR=2
  let [x, y, z] = [rBodyFoot[0], rBodyFoot[1], rBodyFoot[2]];
  if (isRight) y = -y;

  const R1 = PI / 2 - phi;
  [x, y, z] = _rotXApply(-R1, [x, y, z]);

  let lenA = Math.sqrt(y * y + z * z);
  if (lenA < 1e-9) lenA = 1e-9;

  const a1 = _pointToRad(y, z);
  const asinArg = Math.min(1.0, Math.max(-1.0, (Math.sin(phi) * L1Hip) / lenA));
  const a2 = Math.asin(asinArg);
  const a3 = PI - a2 - phi;
  let theta1 = a1 + a3;
  if (theta1 >= 2 * PI) theta1 = theta1 % (2 * PI);
  theta1 = _wrapToPi(theta1);

  const offY = L1Hip * Math.cos(theta1);
  const offZ = L1Hip * Math.sin(theta1);
  const translated = [x, y - offY, z - offZ];
  const R2 = theta1 + phi - PI / 2;
  const sag = _rotXApply(-R2, translated);
  const xSag = sag[0];
  const zSag = sag[2];
  return [theta1, xSag, -zSag];
}

function _sagittalIK(xSag, ySag, params, opts = {}) {
  const { topLimits = null, botLimitFn = null, hint = null } = opts;
  const { L1, L2 } = params;

  const dOrig = Math.sqrt(xSag * xSag + ySag * ySag);
  const maxR = (L1 + L2) * 0.99;
  const minR = Math.abs(L1 - L2) * 1.01;

  let x = xSag;
  let y = ySag;
  let d = dOrig;
  if (d > maxR) {
    const s = maxR / d;
    x *= s;
    y *= s;
    d = maxR;
  }
  if (d < minR) {
    if (d < 1e-9) {
      x = 0.0;
      y = minR;
      d = minR;
    } else {
      const s = minR / d;
      x *= s;
      y *= s;
      d = minR;
    }
  }

  const cosB = Math.min(1.0, Math.max(-1.0, (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)));
  const tUpper = Math.atan2(x, y) - Math.acos(cosB);

  let botLoDeg = -90.0;
  let botHiDeg =  90.0;
  if (topLimits !== null) {
    const [topLo, topHi] = topLimits;
    if (tUpper < topLo - 1e-9 || tUpper > topHi + 1e-9) return null;
  }
  if (botLimitFn !== null) {
    const [blRad, bhRad] = botLimitFn(tUpper);
    botLoDeg = Math.max(botLoDeg, blRad * RAD2DEG);
    botHiDeg = Math.min(botHiDeg, bhRad * RAD2DEG);
    if (botLoDeg > botHiDeg) return null;
  }

  let bestBot = null;
  let bestErr = Infinity;
  const sweep = (loDeg, hiDeg, stepDeg) => {
    let bd = loDeg;
    while (bd <= hiDeg) {
      const tBot = bd * DEG2RAD;
      const fk = leg_fk(tUpper, tBot, params);
      if (fk !== null) {
        const err = Math.sqrt((fk[0] - x) ** 2 + (fk[1] - y) ** 2);
        if (err < bestErr) {
          bestErr = err;
          bestBot = tBot;
        }
      }
      bd += stepDeg;
    }
  };

  if (hint !== null) {
    const hd = hint * RAD2DEG;
    sweep(Math.max(botLoDeg, hd - 12), Math.min(botHiDeg, hd + 12), 0.5);
    if (bestBot !== null && bestErr < 0.002) {
      const cd = bestBot * RAD2DEG;
      sweep(Math.max(botLoDeg, cd - 1), Math.min(botHiDeg, cd + 1), 0.1);
      return [tUpper, bestBot];
    }
  }

  bestBot = null;
  bestErr = Infinity;
  sweep(botLoDeg, botHiDeg, 2);
  if (bestBot === null || bestErr > 0.01) return null;
  const cd = bestBot * RAD2DEG;
  sweep(Math.max(botLoDeg, cd - 2), Math.min(botHiDeg, cd + 2), 0.1);
  if (bestErr > 0.002) return null;
  return [tUpper, bestBot];
}

// Per-leg warm-start cache (see Kinematics.py _ik_hint).
const _ikHint = {};

/**
 * Full 3-DOF IK for one leg. Returns [coxa, femur, tibia] rad or null.
 * rBodyFoot is [x, y, z] in meters, expressed RELATIVE TO THE HIP ORIGIN
 * (subtract LEG_ORIGINS[:, legIndex] beforehand, like the Python version).
 */
export function leg_explicit_inverse_kinematics(rBodyFoot, legIndex, opts = {}) {
  const {
    params = LEG_PARAMS,
    limits = DEFAULT_JOINT_LIMITS_RAD,
  } = opts;

  const [theta1, xSag, ySag] = _hipAbductorIK(rBodyFoot, legIndex, params.L1_hip, params.phi);
  const [hipLo, hipHi] = limits.coxa;
  if (theta1 < hipLo - 1e-9 || theta1 > hipHi + 1e-9) return null;

  const sag = _sagittalIK(xSag, ySag, params, {
    topLimits: limits.femur,
    botLimitFn: (tTop) => coupled_bot_limits_joint_rad(tTop),
    hint: _ikHint[legIndex] ?? null,
  });
  if (sag === null) return null;
  const [thetaTop, thetaBot] = sag;
  _ikHint[legIndex] = thetaBot;
  return [theta1, thetaTop, thetaBot];
}

/**
 * Solve IK for all four legs.
 *   feet: array of 4 [x, y, z] meters in body frame (FR/FL/RR/RL order)
 *   opts.legOrigins, opts.limits, opts.params as in the single-leg call.
 *
 * Returns `{ok: true, angles: [4][3]}` on success
 * or `{ok: false, leg: "FR"|..., reason: string}` if any leg fails.
 */
export function fourLegsInverseKinematics(feet, opts = {}) {
  const {
    legOrigins = LEG_ORIGINS,
    params = LEG_PARAMS,
    limits = DEFAULT_JOINT_LIMITS_RAD,
  } = opts;

  if (!Array.isArray(feet) || feet.length !== 4) {
    return { ok: false, leg: null, reason: "feet must be an array of 4 [x,y,z]" };
  }

  const angles = [];
  for (let i = 0; i < 4; i++) {
    const target = feet[i];
    if (!Array.isArray(target) || target.length !== 3) {
      return { ok: false, leg: LEG_NAMES[i], reason: "foot target must be [x,y,z]" };
    }
    const rLeg = [
      target[0] - legOrigins[0][i],
      target[1] - legOrigins[1][i],
      target[2] - legOrigins[2][i],
    ];
    const solved = leg_explicit_inverse_kinematics(rLeg, i, { params, limits });
    if (solved === null) {
      return {
        ok: false,
        leg: LEG_NAMES[i],
        reason: `target (${target[0].toFixed(3)}, ${target[1].toFixed(3)}, ${target[2].toFixed(3)}) outside workspace`,
      };
    }
    angles.push(solved);
  }
  return { ok: true, angles };
}

// ─── Canvas helpers (for LegDiagram) ──────────────────────────────────────

export function toCanvasPoint(point, drawing = LEG_DRAWING) {
  // Argos convention: kinematic +y points down-along-leg, which matches
  // screen +y (no flip). Contrast andy-servo where +y was up (math frame)
  // so the old helper negated y.
  //
  // `drawing` defaults to LEG_DRAWING from robot-config so callers in
  // LegDetail / RobotOverview can skip the second arg and still hit the
  // project-standard canvas transform.
  return {
    x: point.x * drawing.scale + drawing.offset.x,
    y: point.y * drawing.scale + drawing.offset.y,
  };
}

export function fromCanvasPoint(point, drawing = LEG_DRAWING) {
  return {
    x: (point.x - drawing.offset.x) / drawing.scale,
    y: (point.y - drawing.offset.y) / drawing.scale,
  };
}

export function segmentPath(a, b, drawing = LEG_DRAWING) {
  // Translate sagittal-frame points into canvas space so the returned
  // SVG d-string can be dropped straight into a <path>. Callers previously
  // had to pre-map every endpoint through toCanvasPoint; baking it in here
  // removes that foot-gun and matches how andy-servo consumed it.
  const pa = toCanvasPoint(a, drawing);
  const pb = toCanvasPoint(b, drawing);
  return `M ${pa.x} ${pa.y} L ${pb.x} ${pb.y}`;
}

// ─── Leg pose builder used by LegDiagram ──────────────────────────────────

/**
 * Build a full per-leg pose from a [coxa, femur, tibia] radian triple.
 * Returns the sagittal geometry (named points used by LegDiagram), the foot
 * in sagittal-plane (x, y) meters, and the matching servo-space angles.
 *
 * The sagittal rendering ignores coxa — it's just passed through in
 * `jointAnglesRad` for the stats grid, and the 3rd-DOF abductor is not
 * drawn on this 2-D preview (per user direction: "numeric stat only").
 */
export function buildLegPoseFromJointAngles(jointAnglesRad, opts = {}) {
  const { legId = "FR", params = LEG_PARAMS } = opts;
  const [coxa, femur, tibia] = jointAnglesRad;

  const state = _sagittal_fk_state(femur, tibia, params);
  let geometry;
  // Key contract (mirrors andy-servo's sagittal pose + the raw state keys
  // from _sagittal_fk_state): O / knee / horn / bcLeft / bcRight / rod2 /
  // foot. LegDetail's SEGMENTS and DOT_KEYS, plus RobotOverview, read
  // these names directly — renaming them here breaks the consumer
  // contract, so don't.
  if (state === null) {
    geometry = {
      O:       { x: 0, y: 0 },
      knee:    { x: 0, y: 0 },
      horn:    { x: 0, y: 0 },
      bcLeft:  { x: 0, y: 0 },
      bcRight: { x: 0, y: 0 },
      rod2:    { x: 0, y: 0 },
      foot:    { x: 0, y: 0 },
      valid: false,
    };
  } else {
    // The sagittal FK puts the leg origin at (0,0) with +x forward-swing
    // and +y distance-down-the-leg (right-handed, z=up implied). state
    // holds every named joint as a [x, y] tuple; we convert to {x, y}
    // objects so the canvas helpers can consume them uniformly.
    const toPt = (p) => ({ x: p[0], y: p[1] });
    geometry = {
      O:       toPt(state.O),        // shoulder / upper-leg pivot
      knee:    toPt(state.knee),     // upper→lower leg elbow
      horn:    toPt(state.horn),     // end of the bottom-servo horn
      bcLeft:  toPt(state.bcLeft),   // left arm of the bell-crank (long push-rod anchor)
      bcRight: toPt(state.bcRight),  // right arm of the bell-crank (horn rod anchor)
      rod2:    toPt(state.rod2),     // where rod 2 attaches above the knee
      foot:    toPt(state.foot),
      valid: true,
    };
  }

  return {
    geometry,
    foot: geometry.foot,
    jointAnglesRad: [coxa, femur, tibia],
    servoAnglesDeg: _servoDegForLeg(legId, [coxa, femur, tibia]),
    reachable: geometry.valid,
  };
}

// Convert (coxa, femur, tibia) joint-space rad → (coxa, femur, tibia)
// servo-space deg for a specific leg. Matches the offset math in
// firmware/esp32c6/main/main.c (servo = 90 + direction * joint_deg + offset).
// The only per-leg divergence is the sign on coxa (RR/RL flipped).
function _servoDegForLeg(legId, jointAnglesRad) {
  const [coxa, femur, tibia] = jointAnglesRad;
  const coxaSign = legId === "RR" || legId === "RL" ? -1 : 1;
  return [
    SERVO_CENTER_DEG + coxaSign * coxa * RAD2DEG,
    SERVO_CENTER_DEG + femur * RAD2DEG,
    SERVO_CENTER_DEG + tibia * RAD2DEG,
  ];
}
