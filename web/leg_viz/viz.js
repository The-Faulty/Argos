// Three.js scene + animation loop for the Argos single-leg visualization.
// Standalone, no bundler: imports from a CDN via the <script type="importmap">
// declared in index.html.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

import {
  LEG_PARAMS,
  JOINT_LIMITS,
  legFk3D,
  deg,
  rad,
  servoDegToControlRad,
  controlRadToServoDeg,
  MULTIPLIERS,
  OFFSETS,
} from "./kinematics.js";

// ---------- scene setup ---------------------------------------------------

const canvas = document.getElementById("viewport");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
resizeRenderer();

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12151c);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
camera.up.set(0, 1, 0);
camera.position.set(0.35, 0.15, 0.4);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, -0.08, 0);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xbfd1ff, 0x1f2430, 0.9));
const key = new THREE.DirectionalLight(0xffffff, 0.7);
key.position.set(0.5, 0.8, 0.5);
scene.add(key);

// Ground grid at the stand-height Z (~-0.165 m) just as a visual floor.
const STAND_FOOT_Y = -0.165;
const grid = new THREE.GridHelper(0.8, 16, 0x2a3142, 0x20252f);
grid.position.y = STAND_FOOT_Y;
scene.add(grid);

// Body stub — a small box representing the corner of the chassis the leg
// is mounted to. Purely visual anchor.
const bodyStub = new THREE.Mesh(
  new THREE.BoxGeometry(0.08, 0.03, 0.06),
  new THREE.MeshStandardMaterial({ color: 0x3b4458, metalness: 0.3, roughness: 0.6 }),
);
bodyStub.position.set(0, 0.015, 0);   // sits just above the hip pivot
scene.add(bodyStub);

// Hip pivot axle — a thin cylinder along body-X to show the abduction axis.
const axle = new THREE.Mesh(
  new THREE.CylinderGeometry(0.003, 0.003, 0.07, 16),
  new THREE.MeshStandardMaterial({ color: 0x8796ad }),
);
axle.rotation.z = Math.PI / 2;   // default cylinder Y -> X
scene.add(axle);

// ---------- leg segments --------------------------------------------------

function makeSegment(color, radius) {
  // Unit cylinder along +Y; we'll stretch + orient it per frame.
  const geom = new THREE.CylinderGeometry(radius, radius, 1, 16);
  geom.translate(0, 0.5, 0);   // pivot at its base
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.4, roughness: 0.5 });
  const mesh = new THREE.Mesh(geom, mat);
  scene.add(mesh);
  return mesh;
}

function makeJoint(color, radius) {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 16, 12),
    new THREE.MeshStandardMaterial({ color, metalness: 0.2, roughness: 0.4 }),
  );
  scene.add(mesh);
  return mesh;
}

const segments = {
  coxa:  makeSegment(0xd6b46a, 0.009),
  femur: makeSegment(0xcf7a4a, 0.008),
  tibia: makeSegment(0xcf7a4a, 0.007),
  bcR:   makeSegment(0x7fb4ff, 0.0045),
  bcL:   makeSegment(0x7fb4ff, 0.0045),
  horn:  makeSegment(0xffa1c2, 0.0045),
  rod1:  makeSegment(0xe0e6f0, 0.0035),
  rod2:  makeSegment(0xe0e6f0, 0.0035),
};

const joints = {
  hip:     makeJoint(0xffbe3e, 0.011),
  femur:   makeJoint(0xffbe3e, 0.010),
  knee:    makeJoint(0xff6a3e, 0.009),
  foot:    makeJoint(0xff3e6a, 0.012),
  hornTip: makeJoint(0xff83b3, 0.006),
  bcR:     makeJoint(0x7fb4ff, 0.006),
  bcL:     makeJoint(0x7fb4ff, 0.006),
  rod2At:  makeJoint(0xffffff, 0.005),
};

// Drop line from foot to ground, pure visual depth cue.
const dropGeom = new THREE.BufferGeometry().setFromPoints([
  new THREE.Vector3(), new THREE.Vector3(),
]);
const dropLine = new THREE.Line(
  dropGeom,
  new THREE.LineDashedMaterial({ color: 0x5a6578, dashSize: 0.01, gapSize: 0.008 }),
);
scene.add(dropLine);

// ---------- frame mapping -------------------------------------------------
// Python body frame: x=forward, y=left, z=up.
// Three.js scene:    x=right (in screen), y=up, z=toward camera.
// Mapping so the default view sees the leg from a natural 3/4 angle:
//   scene.x =  python.y      (lateral)
//   scene.y =  python.z      (vertical)
//   scene.z = -python.x      (forward points away from camera by default)
function toScene(pyVec) {
  return new THREE.Vector3(pyVec[1], pyVec[2], -pyVec[0]);
}

// Place a unit cylinder (base at origin, pointing +Y) between two points.
function setSegment(mesh, pA, pB) {
  const a = toScene(pA);
  const b = toScene(pB);
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  mesh.position.copy(a);
  mesh.scale.set(1, Math.max(len, 1e-6), 1);
  if (len > 1e-9) {
    const up = new THREE.Vector3(0, 1, 0);
    mesh.quaternion.setFromUnitVectors(up, dir.clone().normalize());
  }
}

function setJoint(mesh, pyVec) {
  mesh.position.copy(toScene(pyVec));
}

// ---------- state + UI binding --------------------------------------------

const state = {
  theta1: 0,     // rad, hip abductor
  thetaTop: 0,   // rad, upper leg (top servo)
  thetaBot: 0,   // rad, bell-crank (bottom servo)
  lastValid: null,    // last feasible FK result, for when linkage goes infeasible
  animation: null,    // { fn(t), startTime } or null
};

const sliders = {
  theta1: document.getElementById("slider-theta1"),
  thetaTop: document.getElementById("slider-top"),
  thetaBot: document.getElementById("slider-bot"),
};
const readouts = {
  theta1Ctrl: document.getElementById("readout-theta1-ctrl"),
  thetaTopCtrl: document.getElementById("readout-top-ctrl"),
  thetaBotCtrl: document.getElementById("readout-bot-ctrl"),
  theta1Srv: document.getElementById("readout-theta1-srv"),
  thetaTopSrv: document.getElementById("readout-top-srv"),
  thetaBotSrv: document.getElementById("readout-bot-srv"),
  foot: document.getElementById("readout-foot"),
  servoSummary: document.getElementById("readout-servo-summary"),
  status: document.getElementById("readout-status"),
};

function clampToLimit(val, [lo, hi]) {
  return Math.min(hi, Math.max(lo, val));
}

function setJointsFromServo(hipServo, topServo, botServo, { fromUi = false } = {}) {
  state.theta1 = clampToLimit(
    servoDegToControlRad(hipServo, MULTIPLIERS.hip, OFFSETS.hip),
    JOINT_LIMITS.theta1,
  );
  state.thetaTop = clampToLimit(
    servoDegToControlRad(topServo, MULTIPLIERS.top, OFFSETS.top),
    JOINT_LIMITS.theta_top,
  );
  state.thetaBot = clampToLimit(
    servoDegToControlRad(botServo, MULTIPLIERS.bot, OFFSETS.bot),
    JOINT_LIMITS.theta_bot,
  );
  syncSlidersFromState();
  if (!fromUi) cancelAnimationIfUserDriven();
}

function setJointsFromControl(t1, tt, tb, { fromUi = false } = {}) {
  state.theta1 = clampToLimit(t1, JOINT_LIMITS.theta1);
  state.thetaTop = clampToLimit(tt, JOINT_LIMITS.theta_top);
  state.thetaBot = clampToLimit(tb, JOINT_LIMITS.theta_bot);
  syncSlidersFromState();
  if (!fromUi) cancelAnimationIfUserDriven();
}

function syncSlidersFromState() {
  sliders.theta1.value = deg(state.theta1);
  sliders.thetaTop.value = deg(state.thetaTop);
  sliders.thetaBot.value = deg(state.thetaBot);
}

// Slider inputs write back into state but do NOT loop animations.
function onSliderInput() {
  state.theta1 = rad(parseFloat(sliders.theta1.value));
  state.thetaTop = rad(parseFloat(sliders.thetaTop.value));
  state.thetaBot = rad(parseFloat(sliders.thetaBot.value));
  state.animation = null;   // dragging a slider cancels any running preset
}
sliders.theta1.addEventListener("input", onSliderInput);
sliders.thetaTop.addEventListener("input", onSliderInput);
sliders.thetaBot.addEventListener("input", onSliderInput);

function cancelAnimationIfUserDriven() {
  // Presets call setJoints* with fromUi=false, so the animation layer
  // manages itself; slider drags null out state.animation directly.
}

// Init clamps + ranges on the sliders to match the Python joint limits.
sliders.theta1.min = deg(JOINT_LIMITS.theta1[0]);
sliders.theta1.max = deg(JOINT_LIMITS.theta1[1]);
sliders.thetaTop.min = deg(JOINT_LIMITS.theta_top[0]);
sliders.thetaTop.max = deg(JOINT_LIMITS.theta_top[1]);
sliders.thetaBot.min = deg(JOINT_LIMITS.theta_bot[0]);
sliders.thetaBot.max = deg(JOINT_LIMITS.theta_bot[1]);

// ---------- presets -------------------------------------------------------
// Poses are specified in servo degrees so the 90/90/90 reference stays front
// and center. All four presets (Stand + gait states) are rooted there.

// All preset servo values are chosen so the calibration offsets in
// kinematics.js land the foot at visually sensible sagittal positions.
// Values came from a sweep of the ported FK — see the README.

// Stand: servo 90/90/90. With the baked-in offsets, foot lands at
// ~(0, 165 mm) sagittal — the default stand height from Config.py.
const PRESET_STAND = { hip: 90, top: 90, bot: 90 };

// Crouch: foot ~(0, 130 mm) — body 35 mm lower over the foot.
const PRESET_CROUCH = { hip: 90, top: 81, bot: 96 };

// Stretch: foot ~(0, 230 mm) — near max leg extension, directly below.
const PRESET_STRETCH = { hip: 90, top: 115, bot: 46 };

// Gait arcs: foot follows a sinusoidal arc around the stand pose.
// The servo deltas here are small (±8°) so the foot stays near the
// standing workspace and the linkage stays feasible throughout the
// cycle. Walk = slow (1.6 s); trot = fast (0.75 s).
function gaitAnimation(periodSec) {
  return (t) => {
    const ph = (t % periodSec) / periodSec;     // 0..1
    const w = 2 * Math.PI * ph;
    // Swing half (ph<0.5): foot lifts; stance half: foot plants & pushes.
    const swing = ph < 0.5;
    const lift = swing ? 8 * Math.sin(2 * w) : 0;     // knee bend during swing
    const fwd = 6 * Math.cos(w);                      // fore/aft sweep
    return {
      hip: 90,
      top: 90 - fwd - lift * 0.5,
      bot: 90 + fwd * 0.8 + lift,
    };
  };
}

function startPreset(pose) {
  state.animation = null;
  setJointsFromServo(pose.hip, pose.top, pose.bot);
}

function startAnimation(fn) {
  state.animation = { fn, start: performance.now() / 1000 };
}

document.getElementById("btn-stand").addEventListener("click", () =>
  startPreset(PRESET_STAND),
);
document.getElementById("btn-crouch").addEventListener("click", () =>
  startPreset(PRESET_CROUCH),
);
document.getElementById("btn-stretch").addEventListener("click", () =>
  startPreset(PRESET_STRETCH),
);
document.getElementById("btn-walk").addEventListener("click", () =>
  startAnimation(gaitAnimation(1.6)),
);
document.getElementById("btn-trot").addEventListener("click", () =>
  startAnimation(gaitAnimation(0.75)),
);

// ---------- robot control layer -------------------------------------------
// When enabled, POSTs the current servo angles to /api/servo on the Pi.
// Throttled to 20 Hz and only fires when the pose actually changed, so the
// 60 fps animation loop doesn't hammer the server.

const robot = {
  enabled: false,
  lastPostMs: 0,
  lastSent: null,            // {hip, top, bot}
  inflight: false,
  minPeriodMs: 50,           // 20 Hz cap
  endpoint: "/api/servo",
  releaseEndpoint: "/api/release",
  statusEndpoint: "/api/status",
};

const robotUi = {
  toggle: document.getElementById("robot-toggle"),
  status: document.getElementById("robot-status"),
  release: document.getElementById("btn-release"),
};

function setRobotStatus(text, color) {
  robotUi.status.textContent = text;
  robotUi.status.style.color = color;
}

async function probeRobot() {
  try {
    const r = await fetch(robot.statusEndpoint, { cache: "no-store" });
    if (!r.ok) throw new Error("HTTP " + r.status);
    const s = await r.json();
    if (s.sim) setRobotStatus("robot: sim mode", "#e8a04a");
    else if (s.connected) setRobotStatus("robot: connected", "#9adb8a");
    else setRobotStatus("robot: no hardware", "#e8a04a");
    return true;
  } catch (e) {
    setRobotStatus("robot: unreachable", "#e06868");
    return false;
  }
}

async function postPose(hip, top, bot) {
  if (robot.inflight) return;
  robot.inflight = true;
  try {
    const r = await fetch(robot.endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ hip, top, bot }),
    });
    if (!r.ok) throw new Error("HTTP " + r.status);
  } catch (e) {
    setRobotStatus("robot: send failed (" + e.message + ")", "#e06868");
    robot.enabled = false;
    robotUi.toggle.checked = false;
  } finally {
    robot.inflight = false;
  }
}

function maybePostCurrentPose() {
  if (!robot.enabled) return;
  const now = performance.now();
  if (now - robot.lastPostMs < robot.minPeriodMs) return;

  const hip = controlRadToServoDeg(state.theta1, MULTIPLIERS.hip, OFFSETS.hip);
  const top = controlRadToServoDeg(state.thetaTop, MULTIPLIERS.top, OFFSETS.top);
  const bot = controlRadToServoDeg(state.thetaBot, MULTIPLIERS.bot, OFFSETS.bot);

  const last = robot.lastSent;
  if (
    last &&
    Math.abs(last.hip - hip) < 0.1 &&
    Math.abs(last.top - top) < 0.1 &&
    Math.abs(last.bot - bot) < 0.1
  ) {
    return;   // pose unchanged, don't POST
  }
  robot.lastPostMs = now;
  robot.lastSent = { hip, top, bot };
  postPose(hip, top, bot);
}

robotUi.toggle.addEventListener("change", async (e) => {
  if (e.target.checked) {
    const ok = await probeRobot();
    if (!ok) {
      e.target.checked = false;
      return;
    }
    robot.enabled = true;
    robot.lastSent = null;   // force first POST
  } else {
    robot.enabled = false;
    setRobotStatus("robot: offline", "#8893a5");
  }
});

robotUi.release.addEventListener("click", async () => {
  try {
    await fetch(robot.releaseEndpoint, { method: "POST" });
    setRobotStatus("robot: released", "#e8a04a");
  } catch (e) {
    setRobotStatus("robot: release failed", "#e06868");
  }
});

// Probe once on load so the user can see whether a server is reachable
// before flipping the toggle.
probeRobot();

// ---------- main loop -----------------------------------------------------

function stepAnimation() {
  if (!state.animation) return;
  const t = performance.now() / 1000 - state.animation.start;
  const pose = state.animation.fn(t);
  setJointsFromServo(pose.hip, pose.top, pose.bot);
}

function updateLeg() {
  const fk = legFk3D(state.theta1, state.thetaTop, state.thetaBot, 0, LEG_PARAMS);

  // If the bell-crank linkage doesn't close for this pose, keep the last
  // valid frame on screen and flag it. Everything else keeps rendering.
  const valid = !!fk;
  const use = valid ? fk : state.lastValid;
  if (valid) state.lastValid = fk;

  if (use) {
    setSegment(segments.coxa, use.hipOrigin, use.femurPivot);
    setSegment(segments.femur, use.femurPivot, use.knee);
    setSegment(segments.tibia, use.knee, use.foot);
    setSegment(segments.horn, use.femurPivot, use.hornTip);
    setSegment(segments.bcR, use.femurPivot, use.bellCrankRight);
    setSegment(segments.bcL, use.femurPivot, use.bellCrankLeft);
    setSegment(segments.rod1, use.hornTip, use.bellCrankRight);
    setSegment(segments.rod2, use.bellCrankLeft, use.rod2Attach);

    setJoint(joints.hip, use.hipOrigin);
    setJoint(joints.femur, use.femurPivot);
    setJoint(joints.knee, use.knee);
    setJoint(joints.foot, use.foot);
    setJoint(joints.hornTip, use.hornTip);
    setJoint(joints.bcR, use.bellCrankRight);
    setJoint(joints.bcL, use.bellCrankLeft);
    setJoint(joints.rod2At, use.rod2Attach);

    // Update drop line.
    const footScene = toScene(use.foot);
    const groundScene = footScene.clone();
    groundScene.y = STAND_FOOT_Y;
    dropLine.geometry.setFromPoints([footScene, groundScene]);
    dropLine.computeLineDistances();
  }

  updateReadouts(valid, use);
}

function updateReadouts(valid, fk) {
  const t1d = deg(state.theta1);
  const ttd = deg(state.thetaTop);
  const tbd = deg(state.thetaBot);
  const hipSrv = controlRadToServoDeg(state.theta1, MULTIPLIERS.hip, OFFSETS.hip);
  const topSrv = controlRadToServoDeg(state.thetaTop, MULTIPLIERS.top, OFFSETS.top);
  const botSrv = controlRadToServoDeg(state.thetaBot, MULTIPLIERS.bot, OFFSETS.bot);

  readouts.theta1Ctrl.textContent = t1d.toFixed(2) + "°";
  readouts.thetaTopCtrl.textContent = ttd.toFixed(2) + "°";
  readouts.thetaBotCtrl.textContent = tbd.toFixed(2) + "°";
  readouts.theta1Srv.textContent = hipSrv.toFixed(1) + "°";
  readouts.thetaTopSrv.textContent = topSrv.toFixed(1) + "°";
  readouts.thetaBotSrv.textContent = botSrv.toFixed(1) + "°";
  readouts.servoSummary.textContent =
    `Servo H/T/B = ${hipSrv.toFixed(1)} / ${topSrv.toFixed(1)} / ${botSrv.toFixed(1)}`;

  if (fk) {
    const [fx, fy, fz] = fk.foot;
    const [sx, sy] = fk.footSag;
    readouts.foot.textContent =
      `foot body: (${(fx * 1000).toFixed(1)}, ${(fy * 1000).toFixed(1)}, ${(fz * 1000).toFixed(1)}) mm` +
      `   sagittal: (${(sx * 1000).toFixed(1)}, ${(sy * 1000).toFixed(1)}) mm`;
  } else {
    readouts.foot.textContent = "foot: —";
  }

  readouts.status.textContent = valid ? "linkage: OK" : "linkage: INFEASIBLE (showing last valid frame)";
  readouts.status.style.color = valid ? "#9adb8a" : "#e8a04a";
}

function resizeRenderer() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (renderer.domElement.width !== w || renderer.domElement.height !== h) {
    renderer.setSize(w, h, false);
  }
  if (camera) {
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}

function tick() {
  resizeRenderer();
  stepAnimation();
  updateLeg();
  maybePostCurrentPose();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

// Initialize at the stand pose and start rendering.
setJointsFromServo(PRESET_STAND.hip, PRESET_STAND.top, PRESET_STAND.bot);
window.addEventListener("resize", resizeRenderer);
tick();
