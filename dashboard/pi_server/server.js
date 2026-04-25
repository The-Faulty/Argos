// Pi-side Node server for the Argos dashboard.
//
// One process per Pi. Responsibilities:
//   - Serve the built React bundle (`dashboard/dist/`) over HTTP :8787.
//   - Maintain exactly ONE rosbridge client per Pi (ws://localhost:9090).
//     Browsers talk to THIS server, not rosbridge directly — this halves
//     telemetry message load and gives us a single place to enforce schema.
//   - Fan telemetry (joint_states, imu, gas, gait_mode, control_mode,
//     connection status) out to every connected browser over WS `/telemetry`.
//   - Accept HTTP JSON command bodies under `/api/*`, validate them via the
//     shared protocol schema, and publish to rosbridge.
//   - Own the `/cmd_vel` 30 Hz republish loop when Twist is streaming.
//     `command_mux` has a 250 ms freshness timeout; silence = auto-idle.
//   - Handle Disconnect = publish `/release_servos=true` + `/estop=true`
//     atomically.
//   - Handle "rotate by N degrees": publish angular.z=±rotate_rate for
//     `degrees * calibration_factor / rotate_rate` seconds, then zero.
//   - Persist user settings under `~/.argos/` and broadcast them whenever a
//     new client connects (dashboard UI needs initial state).
//
// This file is intentionally the top-of-stack. Lower-level detail lives in
// the sibling modules: `persistence.js`, `ros_topics.js`, `recording.js`.

import path from "node:path";
import { fileURLToPath } from "node:url";
import { createServer } from "node:http";
import express from "express";
import { WebSocketServer } from "ws";

import {
  DEFAULT_ROTATE_SETTINGS,
  LEG_IDS,
  JOINT_NAMES,
  MODE_OPTIONS,
} from "../shared/robot-config.js";
import { validateCommand } from "../shared/protocol.js";
import { euler_from_quaternion } from "./quaternion.js";

import {
  loadServoOverrides,
  saveServoOverrides,
  loadJointLimits,
  saveJointLimits,
  loadStances,
  saveStances,
  upsertStance,
  loadRotateSettings,
  saveRotateSettings,
  loadServoSpeed,
  saveServoSpeed,
  loadServoUpdateRate,
  saveServoUpdateRate,
} from "./persistence.js";
import { TelemetryRecorder, listRecordings } from "./recording.js";
import { RosBridgeClient } from "./ros_topics.js";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const HTTP_PORT = Number(process.env.ARGOS_DASHBOARD_PORT ?? 8787);
const ROSBRIDGE_URL = process.env.ARGOS_ROSBRIDGE_URL ?? "ws://localhost:9090";
const STATIC_DIR = path.resolve(__dirname, "..", "dist");
const TWIST_REPUBLISH_HZ = 30;
const GAIT_PLANNER_NODE = "/gait_planner_node";

// ─── State ────────────────────────────────────────────────────────────────

const state = {
  rotateSettings: { ...DEFAULT_ROTATE_SETTINGS },
  servoOverrides: {},
  jointLimits: {},
  servoSpeed: {},           // per-joint-row deg/s (coxa/femur/tibia)
  servoUpdateRate: { hz: 0 }, // PWM/command update rate; filled in bootstrap
  stances: {},
  mode: "idle",
  // Latest cached telemetry — what we fan out to new WS clients.
  telemetry: {
    connected: { ros: false, lastEvent: null },
    joint_states: null,
    imu: null,
    gas: null,
    gait_mode: null,
    control_mode: null,
  },
  // Twist streaming state.
  twist: { active: false, x: 0, y: 0, wz: 0, expiresAt: 0 },
  // Rotate-by-degrees state (only one at a time).
  rotate: { active: false, endAt: 0, direction: null },
};

const recorder = new TelemetryRecorder();

// ─── ROS client ───────────────────────────────────────────────────────────

const ros = new RosBridgeClient({ url: ROSBRIDGE_URL });

ros.onStatus((status) => {
  state.telemetry.connected = {
    ros: ros.connected,
    lastEvent: status,
  };
  broadcast({ type: "connected", connected: state.telemetry.connected });
});

ros.subscribeJointStates((msg) => {
  state.telemetry.joint_states = msg;
  broadcast({ type: "joint_states", msg });
  appendRecorderSample();
});

ros.subscribeImu((msg) => {
  const [roll, pitch, yaw] = euler_from_quaternion(msg.orientation);
  const enriched = {
    ...msg,
    _euler_rad: { roll, pitch, yaw },
  };
  state.telemetry.imu = enriched;
  broadcast({ type: "imu", msg: enriched });
  appendRecorderSample();
});

ros.subscribeGas((msg) => {
  state.telemetry.gas = msg;
  broadcast({ type: "gas", msg });
  appendRecorderSample();
});

ros.subscribeGaitMode((msg) => {
  state.telemetry.gait_mode = msg;
  broadcast({ type: "gait_mode", msg });
});

ros.subscribeControlMode((msg) => {
  state.telemetry.control_mode = msg;
  broadcast({ type: "control_mode", msg });
});

// ─── HTTP app ─────────────────────────────────────────────────────────────

const app = express();
app.use(express.json({ limit: "1mb" }));

// Static bundle (built by `npm run build` at the dashboard root).
app.use(express.static(STATIC_DIR));

// Basic health/info endpoint — handy for the "connection pill" in the UI.
app.get("/api/health", (_req, res) => {
  res.json({
    ok: true,
    ros_connected: ros.connected,
    mode: state.mode,
    rotate_settings: state.rotateSettings,
    recording: recorder.status(),
  });
});

// ─── Settings: servo overrides ───────────────────────────────────────────
app.get("/api/settings/servo_overrides", (_req, res) => {
  res.json(state.servoOverrides);
});
app.post("/api/settings/servo_overrides", async (req, res) => {
  try {
    state.servoOverrides = await saveServoOverrides(req.body);
    ros.publishServoOverrides(state.servoOverrides);
    broadcast({ type: "settings:servo_overrides", payload: state.servoOverrides });
    res.json(state.servoOverrides);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: joint limits ──────────────────────────────────────────────
app.get("/api/settings/joint_limits", (_req, res) => {
  res.json(state.jointLimits);
});
app.post("/api/settings/joint_limits", async (req, res) => {
  try {
    state.jointLimits = await saveJointLimits(req.body);
    ros.publishJointLimits(state.jointLimits);
    broadcast({ type: "settings:joint_limits", payload: state.jointLimits });
    res.json(state.jointLimits);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: rotate tuneables ──────────────────────────────────────────
app.get("/api/settings/rotate", (_req, res) => {
  res.json(state.rotateSettings);
});
app.post("/api/settings/rotate", async (req, res) => {
  try {
    state.rotateSettings = await saveRotateSettings(req.body);
    broadcast({ type: "settings:rotate", payload: state.rotateSettings });
    res.json(state.rotateSettings);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: per-joint-row servo speed limit (deg/s) ───────────────────
// Dashboard posts {coxa, femur, tibia}. The bridge node turns that into a
// per-joint slew limit applied before /joint_command/raw publishes, so the
// servos can't be commanded to move faster than the linkage tolerates.
app.get("/api/settings/servo_speed", (_req, res) => {
  res.json(state.servoSpeed);
});
app.post("/api/settings/servo_speed", async (req, res) => {
  try {
    state.servoSpeed = await saveServoSpeed(req.body);
    ros.publishServoSpeedLimits(state.servoSpeed);
    broadcast({ type: "settings:servo_speed", payload: state.servoSpeed });
    res.json(state.servoSpeed);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: PWM / joint-command update rate (Hz) ──────────────────────
// Scalar value driving both (a) how fast the Pi republishes /joint_command/raw
// and (b) how often the firmware re-latches the PCA9685 pulse. Bumping this
// smooths motion; dropping it saves CPU. Broadcast as Float32.
app.get("/api/settings/servo_update_rate", (_req, res) => {
  res.json(state.servoUpdateRate);
});
app.post("/api/settings/servo_update_rate", async (req, res) => {
  try {
    state.servoUpdateRate = await saveServoUpdateRate(req.body);
    ros.publishServoUpdateRate(state.servoUpdateRate.hz);
    broadcast({ type: "settings:servo_update_rate", payload: state.servoUpdateRate });
    res.json(state.servoUpdateRate);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Stances ─────────────────────────────────────────────────────────────
app.get("/api/stances", (_req, res) => res.json(state.stances));
app.post("/api/stances/:name", async (req, res) => {
  try {
    const { name } = req.params;
    // If the client provides explicit angles_rad, use them. Otherwise snapshot
    // the latest /joint_states positions. Snapshotting from the latest
    // JointState is the common UI path — "I've posed the dog by hand, save it".
    let anglesRad = req.body?.angles_rad;
    if (!Array.isArray(anglesRad)) {
      const js = state.telemetry.joint_states;
      if (!js || !Array.isArray(js.position) || !Array.isArray(js.name)) {
        return res.status(409).json({ error: "No /joint_states available to snapshot." });
      }
      const indexByName = Object.fromEntries(js.name.map((n, i) => [n, i]));
      anglesRad = JOINT_NAMES.map((n) => Number(js.position[indexByName[n]]));
      if (anglesRad.some((v) => !Number.isFinite(v))) {
        return res.status(409).json({ error: "Latest /joint_states was incomplete." });
      }
    }
    state.stances = await upsertStance(name, anglesRad);
    broadcast({ type: "stances", payload: state.stances });
    res.json(state.stances[name]);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});
app.post("/api/stances/:name/play", (req, res) => {
  try {
    const { name } = req.params;
    validateCommand({ type: "stance_play", name });
    ros.publishStancePlay(name);
    res.json({ ok: true });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Params: gait planner ────────────────────────────────────────────────
app.post("/api/params/gait/:name", async (req, res) => {
  try {
    const { name } = req.params;
    const { value } = req.body ?? {};
    if (!Number.isFinite(Number(value))) {
      return res.status(400).json({ error: "value must be numeric" });
    }
    const result = await ros.setParameter(GAIT_PLANNER_NODE, name, Number(value));
    res.json({ ok: true, result });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// Stabilizer params live on gait_planner_node too (same node, different param).
app.post("/api/params/stabilizer", async (req, res) => {
  try {
    const body = req.body ?? {};
    const allowed = [
      "stabilization_roll_gain",
      "stabilization_pitch_gain",
      "stabilization_max_correction_rad",
      "imu_filter_alpha",
      "use_imu_stabilization",
    ];
    const results = {};
    for (const key of allowed) {
      if (body[key] === undefined) continue;
      results[key] = await ros.setParameter(GAIT_PLANNER_NODE, key, body[key]);
    }
    res.json({ ok: true, results });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Rotate by N degrees ─────────────────────────────────────────────────
// The dashboard posts {direction, degrees}. We spin cmd_vel.angular.z at
// ±rotate_rate_rad_s for (degrees * rotate_calibration_factor) / rate seconds
// and then zero. Default factor 1.02 is calibrated against bench runs; tuning
// it lives in the Settings drawer.
app.post("/api/rotate", (req, res) => {
  try {
    const cmd = validateCommand({
      type: "rotate_degrees",
      direction: req.body?.direction,
      degrees: Number(req.body?.degrees ?? state.rotateSettings.rotate_increment_deg),
    });
    const { rotate_rate_rad_s, rotate_calibration_factor } = state.rotateSettings;
    const sign = cmd.direction === "left" ? +1 : -1;
    const deg = cmd.degrees;
    const durationMs = Math.max(
      50,
      (deg * rotate_calibration_factor * (Math.PI / 180) / rotate_rate_rad_s) * 1000,
    );
    state.rotate = {
      active: true,
      endAt: Date.now() + durationMs,
      direction: cmd.direction,
    };
    state.twist = {
      active: true,
      x: 0,
      y: 0,
      wz: sign * rotate_rate_rad_s,
      expiresAt: Date.now() + durationMs,
    };
    res.json({ ok: true, duration_ms: durationMs });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Generic command dispatch ─────────────────────────────────────────────
// Anything matching the shared protocol schema. Keeps the server open for
// small commands without bolting a dedicated route onto every one.
app.post("/api/command", (req, res) => {
  try {
    const cmd = validateCommand(req.body);
    handleCommand(cmd);
    res.json({ ok: true });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Recording ───────────────────────────────────────────────────────────
app.post("/api/recording/start", async (_req, res) => {
  try {
    const status = await recorder.start();
    broadcast({ type: "recording", status });
    res.json(status);
  } catch (err) {
    res.status(500).json({ error: String(err.message || err) });
  }
});
app.post("/api/recording/stop", async (_req, res) => {
  try {
    const status = await recorder.stop();
    broadcast({ type: "recording", status });
    res.json(status);
  } catch (err) {
    res.status(500).json({ error: String(err.message || err) });
  }
});
app.get("/api/recording/list", async (_req, res) => {
  try {
    res.json({ files: await listRecordings() });
  } catch (err) {
    res.status(500).json({ error: String(err.message || err) });
  }
});

// SPA fallback — the React router handles unknown routes.
app.get("*", (_req, res, next) => {
  if (_req.path.startsWith("/api") || _req.path.startsWith("/telemetry")) return next();
  res.sendFile(path.join(STATIC_DIR, "index.html"), (err) => {
    if (err) next();
  });
});

// ─── Command handlers ─────────────────────────────────────────────────────

function handleCommand(cmd) {
  switch (cmd.type) {
    case "set_mode": {
      if (!MODE_OPTIONS.includes(cmd.mode)) throw new Error(`Unknown mode: ${cmd.mode}`);
      state.mode = cmd.mode;
      ros.publishGaitMode(cmd.mode);
      ros.publishControlMode(cmd.mode.startsWith("direct_") ? "manual" : "auto");
      broadcast({ type: "mode", mode: cmd.mode });
      break;
    }
    case "twist": {
      state.twist = {
        active: true,
        x: cmd.x,
        y: cmd.y,
        wz: cmd.yaw,
        // Twist is "sticky" for 500 ms so a dropped frame in the browser
        // doesn't immediately kill the gait. Client is expected to keep
        // streaming while held.
        expiresAt: Date.now() + 500,
      };
      // Suppress a competing rotate.
      state.rotate.active = false;
      break;
    }
    case "foot_targets": {
      ros.publishFootTargets(cmd.targets);
      break;
    }
    case "joint_angles": {
      // Flat 12-element array matching JOINT_NAMES order (radians).
      ros.publishJointAnglesRad(JOINT_NAMES, cmd.angles);
      break;
    }
    case "servo_angles": {
      // Servo-horn degrees: the ESP32 firmware receives these through the
      // bridge node converting deg→rad. Same topic as joint_angles since the
      // bridge knows to apply the cal table in reverse.
      const rad = cmd.angles_deg.map((d) => (d - 90.0) * (Math.PI / 180.0));
      ros.publishJointAnglesRad(JOINT_NAMES, rad);
      break;
    }
    case "disconnect": {
      // Emergency path. `release_servos=true` unpowers the PCA9685 outputs;
      // `estop=true` freezes command_mux. Both go at once so neither wins a
      // race with a still-in-flight gait publisher.
      ros.publishReleaseServos(true);
      ros.publishEstop(true);
      state.twist.active = false;
      state.rotate.active = false;
      state.mode = "idle";
      broadcast({ type: "disconnect" });
      break;
    }
    case "animation_play":
    case "animation_stop":
    case "recording_start":
    case "recording_stop": {
      // These route through the dedicated endpoints — but when they come via
      // /api/command we forward through the same code paths.
      if (cmd.type === "animation_play") {
        ros.publishStancePlay(cmd.name); // reuse the stance/animation string channel
      }
      break;
    }
    case "stance_play":
    case "stance_save": {
      ros.publishStancePlay(cmd.name);
      break;
    }
    case "stabilizer_set": {
      for (const [key, value] of Object.entries(cmd.params)) {
        ros.setParameter(GAIT_PLANNER_NODE, key, value).catch((err) => {
          console.warn(`[stabilizer_set] ${key} failed: ${err.message}`);
        });
      }
      break;
    }
    case "rotate_degrees": {
      // Routes through /api/rotate in practice, but we accept the same shape
      // via /api/command for scripted clients.
      const { rotate_rate_rad_s, rotate_calibration_factor } = state.rotateSettings;
      const sign = cmd.direction === "left" ? +1 : -1;
      const durationMs = Math.max(
        50,
        (cmd.degrees * rotate_calibration_factor * (Math.PI / 180) / rotate_rate_rad_s) * 1000,
      );
      state.rotate = { active: true, endAt: Date.now() + durationMs, direction: cmd.direction };
      state.twist = {
        active: true,
        x: 0,
        y: 0,
        wz: sign * rotate_rate_rad_s,
        expiresAt: Date.now() + durationMs,
      };
      break;
    }
    case "set_servo_overrides": {
      saveServoOverrides(cmd.overrides).then((clean) => {
        state.servoOverrides = clean;
        ros.publishServoOverrides(clean);
        broadcast({ type: "settings:servo_overrides", payload: clean });
      });
      break;
    }
    case "set_joint_limits": {
      saveJointLimits(cmd.limits).then((clean) => {
        state.jointLimits = clean;
        ros.publishJointLimits(clean);
        broadcast({ type: "settings:joint_limits", payload: clean });
      });
      break;
    }
    default:
      throw new Error(`Command type ${cmd.type} has no server handler`);
  }
}

// ─── 30 Hz Twist republish ───────────────────────────────────────────────
// command_mux has a 250 ms freshness window — silence → idle. Republishing
// the cached twist at 30 Hz keeps the selected source "fresh" for as long
// as the browser streams. When the rotate timer expires we flip to a
// single zero-twist publish so the dog stops cleanly.
setInterval(() => {
  const now = Date.now();
  if (state.rotate.active && now >= state.rotate.endAt) {
    ros.publishTwist({ x: 0, y: 0, wz: 0 });
    state.rotate.active = false;
    state.twist.active = false;
    return;
  }
  if (!state.twist.active) return;
  if (now > state.twist.expiresAt) {
    // Sticky window elapsed — one explicit zero twist to settle the mux.
    ros.publishTwist({ x: 0, y: 0, wz: 0 });
    state.twist.active = false;
    return;
  }
  ros.publishTwist({ x: state.twist.x, y: state.twist.y, wz: state.twist.wz });
}, 1000 / TWIST_REPUBLISH_HZ);

// ─── WebSocket fan-out ───────────────────────────────────────────────────

const httpServer = createServer(app);
const wss = new WebSocketServer({ server: httpServer, path: "/telemetry" });

wss.on("connection", (ws) => {
  // Seed the new client with the latest snapshot of every topic + settings.
  const snapshot = {
    type: "snapshot",
    telemetry: state.telemetry,
    mode: state.mode,
    rotate_settings: state.rotateSettings,
    servo_overrides: state.servoOverrides,
    joint_limits: state.jointLimits,
    servo_speed: state.servoSpeed,
    servo_update_rate: state.servoUpdateRate,
    stances: state.stances,
    leg_ids: LEG_IDS,
    joint_names: JOINT_NAMES,
    mode_options: MODE_OPTIONS,
  };
  safeSend(ws, snapshot);

  ws.on("message", (raw) => {
    // Allow commands over WS too (for scripted debug). Same schema.
    try {
      const msg = JSON.parse(raw.toString());
      const cmd = validateCommand(msg);
      handleCommand(cmd);
    } catch (err) {
      safeSend(ws, { type: "error", error: String(err.message || err) });
    }
  });
});

function broadcast(payload) {
  const str = JSON.stringify(payload);
  for (const client of wss.clients) {
    if (client.readyState === 1 /* OPEN */) {
      try {
        client.send(str);
      } catch {
        /* drop */
      }
    }
  }
}

function safeSend(ws, payload) {
  try {
    ws.send(JSON.stringify(payload));
  } catch {
    /* drop */
  }
}

function appendRecorderSample() {
  if (!recorder.isActive) return;
  const { joint_states: js, imu, gas } = state.telemetry;
  const positions = {};
  if (js?.name && js?.position) {
    for (let i = 0; i < js.name.length && i < js.position.length; i++) {
      positions[js.name[i]] = Number(js.position[i]);
    }
  }
  recorder.append({
    joint_positions_by_name: positions,
    imu_roll: imu?._euler_rad?.roll,
    imu_pitch: imu?._euler_rad?.pitch,
    imu_yaw: imu?._euler_rad?.yaw,
    gas_ppm: gas?.data,
    cmd_lin_x: state.twist.active ? state.twist.x : 0,
    cmd_lin_y: state.twist.active ? state.twist.y : 0,
    cmd_ang_z: state.twist.active ? state.twist.wz : 0,
  });
}

// ─── Startup ─────────────────────────────────────────────────────────────

async function bootstrap() {
  state.servoOverrides = await loadServoOverrides();
  state.jointLimits = await loadJointLimits();
  state.stances = await loadStances();
  state.rotateSettings = await loadRotateSettings();
  state.servoSpeed = await loadServoSpeed();
  state.servoUpdateRate = await loadServoUpdateRate();

  // Republish persisted settings as soon as rosbridge is ready so a fresh
  // Pi boot picks up the same calibration state the UI last saved.
  ros.onStatus((s) => {
    if (s.event === "connected") {
      ros.publishServoOverrides(state.servoOverrides);
      ros.publishJointLimits(state.jointLimits);
      ros.publishServoSpeedLimits(state.servoSpeed);
      ros.publishServoUpdateRate(state.servoUpdateRate.hz);
    }
  });

  httpServer.listen(HTTP_PORT, () => {
    console.log(`[argos-dashboard] HTTP :${HTTP_PORT}`);
    console.log(`[argos-dashboard] rosbridge ${ROSBRIDGE_URL}`);
    console.log(`[argos-dashboard] static ${STATIC_DIR}`);
  });
}

bootstrap().catch((err) => {
  console.error("[argos-dashboard] bootstrap failed:", err);
  process.exit(1);
});

// Graceful shutdown: publish release_servos=true so an accidental SIGTERM
// during a demo doesn't leave the dog straining against a held pose.
function shutdown(signal) {
  console.log(`[argos-dashboard] caught ${signal}, releasing servos`);
  try {
    ros.publishReleaseServos(true);
    ros.publishEstop(true);
  } catch {
    /* ros might already be closed */
  }
  if (recorder.isActive) {
    recorder.stop().catch(() => {});
  }
  setTimeout(() => process.exit(0), 200);
}
process.on("SIGINT", () => shutdown("SIGINT"));
process.on("SIGTERM", () => shutdown("SIGTERM"));
