// Pi-side Node server for the Argos dashboard.
//
// One process per Pi. Responsibilities:
//   - Serve the built React bundle (`dashboard/dist/`) over HTTP :8787.
//   - Talk to the ESP32 over serial (firmware/argos_servo) via SerialBridge.
//   - Run the gait planner + mode state machine via ModeController.
//   - Forward IMU + thermal + MJPEG from the Python sensor sidecar via
//     SensorProxy.
//   - Fan telemetry out to every connected browser over WS `/telemetry`.
//   - Accept HTTP JSON command bodies under `/api/*`, validate via the shared
//     protocol schema, route to the right module.
//   - Persist user settings under `~/.argos/` and replay them on connect.

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
import { setOverrides as setServoCalOverrides } from "../shared/servo_cal.js";
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
import { SerialBridge } from "./serial_bridge.js";
import { ModeController } from "./mode_controller.js";
import { SensorProxy } from "./sensor_proxy.js";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const HTTP_PORT = Number(process.env.ARGOS_DASHBOARD_PORT ?? 8787);
const STATIC_DIR = path.resolve(__dirname, "..", "dist");
const STARTUP_STAND_MS = Math.max(0, Number(process.env.ARGOS_STARTUP_STAND_MS ?? 900));
const STARTUP_CROUCH_MS = Math.max(0, Number(process.env.ARGOS_STARTUP_CROUCH_MS ?? 700));

// ─── State ────────────────────────────────────────────────────────────────

const state = {
  rotateSettings: { ...DEFAULT_ROTATE_SETTINGS },
  servoOverrides: {},
  jointLimits: {},
  servoSpeed: {},
  servoUpdateRate: { hz: 50 },
  stances: {},
  mode: "idle",
  telemetry: {
    connected: { serial: false, sensors: false, lastEvent: null },
    joint_states: null,
    imu: null,
    gas: null,
    thermal: null,
    gait_mode: null,
    control_mode: null,
  },
  // Sticky twist state — last twist held for 500 ms so a dropped frame doesn't
  // freeze the gait. Replaced any time a fresh twist arrives.
  twist: { active: false, x: 0, y: 0, wz: 0, expiresAt: 0 },
  rotate: { active: false, endAt: 0, direction: null },
};

const recorder = new TelemetryRecorder();

// ─── Hardware modules ─────────────────────────────────────────────────────

const serial = new SerialBridge();
const mode = new ModeController({
  serial,
  getStances: async () => state.stances,
});
const sensors = new SensorProxy();

serial.on("open", () => {
  state.telemetry.connected = { ...state.telemetry.connected, serial: true, lastEvent: { event: "serial-open" } };
  broadcast({ type: "connected", connected: state.telemetry.connected });
});
serial.on("close", () => {
  state.telemetry.connected = { ...state.telemetry.connected, serial: false, lastEvent: { event: "serial-close" } };
  broadcast({ type: "connected", connected: state.telemetry.connected });
});
serial.on("error", (err) => {
  console.warn("[serial]", err.message || err);
});

// Joint states from the firmware (actual hardware positions in servo-deg).
serial.on("joint_states", (msg) => {
  // Firmware joint_states are the ground truth for the live robot. Keep the
  // planner's commanded values in separate fields so the UI does not render
  // a pose the hardware has not physically reached.
  const last = state.telemetry.joint_states || {};
  state.telemetry.joint_states = {
    ...last,
    name: msg.name,
    position: msg.position,
    position_servo_deg: msg.position_servo_deg,
  };
  broadcast({ type: "joint_states", msg: state.telemetry.joint_states });
});

serial.on("gas", (msg) => {
  state.telemetry.gas = msg;
  broadcast({ type: "gas", msg });
  appendRecorderSample();
});

// Joint states the planner just commanded (radians, the canonical reading).
mode.on("joint_states", (msg) => {
  const last = state.telemetry.joint_states || {};
  state.telemetry.joint_states = {
    ...last,
    name: msg.name,
    commanded_position: msg.position,
    commanded_position_servo_deg: msg.position_servo_deg,
    position: Array.isArray(last.position) ? last.position : msg.position,
    position_servo_deg: Array.isArray(last.position_servo_deg)
      ? last.position_servo_deg
      : msg.position_servo_deg,
  };
  broadcast({ type: "joint_states", msg: state.telemetry.joint_states });
  appendRecorderSample();
});
mode.on("mode", (m) => {
  state.mode = m;
  state.telemetry.gait_mode = { data: m };
  state.telemetry.control_mode = { data: m.startsWith("direct_") ? "manual" : "auto" };
  broadcast({ type: "mode", mode: m });
  broadcast({ type: "gait_mode", msg: state.telemetry.gait_mode });
  broadcast({ type: "control_mode", msg: state.telemetry.control_mode });
});
mode.on("error", (errMsg) => {
  broadcast({ type: "error", error: String(errMsg) });
});

sensors.on("open", () => {
  state.telemetry.connected = { ...state.telemetry.connected, sensors: true };
  broadcast({ type: "connected", connected: state.telemetry.connected });
});
sensors.on("close", () => {
  state.telemetry.connected = { ...state.telemetry.connected, sensors: false };
  broadcast({ type: "connected", connected: state.telemetry.connected });
});
sensors.on("error", (err) => {
  console.warn("[sensors]", err.message || err);
});
sensors.on("imu", (msg) => {
  // Expected shape from sidecar: {type:"imu", quaternion:[x,y,z,w], accel, gyro, ts}.
  let enriched = msg;
  if (Array.isArray(msg.quaternion) && msg.quaternion.length === 4) {
    const [x, y, z, w] = msg.quaternion;
    const [roll, pitch, yaw] = euler_from_quaternion({ x, y, z, w });
    enriched = { ...msg, orientation: { x, y, z, w }, _euler_rad: { roll, pitch, yaw } };
    mode.setImu({ roll, pitch, yaw });
  }
  state.telemetry.imu = enriched;
  broadcast({ type: "imu", msg: enriched });
  appendRecorderSample();
});
sensors.on("thermal", (msg) => {
  state.telemetry.thermal = msg;
  broadcast({ type: "thermal", msg });
});

// ─── HTTP app ─────────────────────────────────────────────────────────────

const app = express();
app.use(express.json({ limit: "1mb" }));
app.use(express.static(STATIC_DIR));

app.get("/api/health", (_req, res) => {
  res.json({
    ok: true,
    serial_connected: serial.connected,
    sensors_connected: sensors.connected,
    mode: state.mode,
    rotate_settings: state.rotateSettings,
    recording: recorder.status(),
  });
});

// MJPEG proxy: <img src="/api/camera/stream" />
app.get("/api/camera/stream", (req, res) => sensors.proxyCameraStream(req, res));

// Sidecar /health passthrough so the operator can diagnose silent sensors
// (e.g. thermal panel showing "no data") without SSHing into the Pi.
app.get("/api/sensors/health", (req, res) => sensors.proxyHealth(req, res));

// ─── Settings: servo overrides ───────────────────────────────────────────
app.get("/api/settings/servo_overrides", (_req, res) => res.json(state.servoOverrides));
app.post("/api/settings/servo_overrides", async (req, res) => {
  try {
    state.servoOverrides = await saveServoOverrides(req.body);
    setServoCalOverrides(state.servoOverrides);
    broadcast({ type: "settings:servo_overrides", payload: state.servoOverrides });
    res.json(state.servoOverrides);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: joint limits ──────────────────────────────────────────────
app.get("/api/settings/joint_limits", (_req, res) => res.json(state.jointLimits));
app.post("/api/settings/joint_limits", async (req, res) => {
  try {
    state.jointLimits = await saveJointLimits(req.body);
    mode.setJointLimits(state.jointLimits);
    broadcast({ type: "settings:joint_limits", payload: state.jointLimits });
    res.json(state.jointLimits);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: rotate tuneables ──────────────────────────────────────────
app.get("/api/settings/rotate", (_req, res) => res.json(state.rotateSettings));
app.post("/api/settings/rotate", async (req, res) => {
  try {
    state.rotateSettings = await saveRotateSettings(req.body);
    broadcast({ type: "settings:rotate", payload: state.rotateSettings });
    res.json(state.rotateSettings);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: servo speed limit (deg/s, per joint row) ──────────────────
app.get("/api/settings/servo_speed", (_req, res) => res.json(state.servoSpeed));
app.post("/api/settings/servo_speed", async (req, res) => {
  try {
    state.servoSpeed = await saveServoSpeed(req.body);
    mode.setServoSpeedLimit(state.servoSpeed);
    broadcast({ type: "settings:servo_speed", payload: state.servoSpeed });
    res.json(state.servoSpeed);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Settings: servo update rate (Hz) ────────────────────────────────────
app.get("/api/settings/servo_update_rate", (_req, res) => res.json(state.servoUpdateRate));
app.post("/api/settings/servo_update_rate", async (req, res) => {
  try {
    state.servoUpdateRate = await saveServoUpdateRate(req.body);
    mode.setServoUpdateRate(state.servoUpdateRate.hz);
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
    let anglesRad = req.body?.angles_rad;
    if (!Array.isArray(anglesRad)) {
      const js = state.telemetry.joint_states;
      if (!js || !Array.isArray(js.position) || !Array.isArray(js.name)) {
        return res.status(409).json({ error: "No joint_states available to snapshot." });
      }
      const indexByName = Object.fromEntries(js.name.map((n, i) => [n, i]));
      anglesRad = JOINT_NAMES.map((n) => Number(js.position[indexByName[n]]));
      if (anglesRad.some((v) => !Number.isFinite(v))) {
        return res.status(409).json({ error: "Latest joint_states was incomplete." });
      }
    }
    state.stances = await upsertStance(name, anglesRad);
    broadcast({ type: "stances", payload: state.stances });
    res.json(state.stances[name]);
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});
app.post("/api/stances/:name/play", async (req, res) => {
  try {
    const { name } = req.params;
    validateCommand({ type: "stance_play", name });
    await mode.setMode(name); // crouch / stand / extend
    res.json({ ok: true });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Params: gait planner / stabilizer ───────────────────────────────────
app.post("/api/params/gait/:name", async (req, res) => {
  try {
    const { name } = req.params;
    const value = Number(req.body?.value);
    if (!Number.isFinite(value)) return res.status(400).json({ error: "value must be numeric" });
    const patch = mapGaitParam(name, value);
    if (!patch) return res.status(400).json({ error: `Unknown gait param: ${name}` });
    mode.updatePlannerConfig(patch);
    res.json({ ok: true });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

app.post("/api/params/stabilizer", async (req, res) => {
  try {
    mode.updateStabilizerParams(req.body || {});
    res.json({ ok: true });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Rotate by N degrees ─────────────────────────────────────────────────
app.post("/api/rotate", (req, res) => {
  try {
    const cmd = validateCommand({
      type: "rotate_degrees",
      direction: req.body?.direction,
      degrees: Number(req.body?.degrees ?? state.rotateSettings.rotate_increment_deg),
    });
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
    mode.setTwist({ x: 0, y: 0, yaw: state.twist.wz });
    res.json({ ok: true, duration_ms: durationMs });
  } catch (err) {
    res.status(400).json({ error: String(err.message || err) });
  }
});

// ─── Generic command dispatch ─────────────────────────────────────────────
app.post("/api/command", async (req, res) => {
  try {
    const cmd = validateCommand(req.body);
    await handleCommand(cmd);
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

// SPA fallback
app.get("*", (req, res, next) => {
  if (req.path.startsWith("/api") || req.path.startsWith("/telemetry")) return next();
  res.sendFile(path.join(STATIC_DIR, "index.html"), (err) => {
    if (err) next();
  });
});

// ─── Command handlers ─────────────────────────────────────────────────────

async function handleCommand(cmd) {
  switch (cmd.type) {
    case "set_mode":
      await mode.setMode(cmd.mode);
      break;
    case "twist":
      state.twist = {
        active: true,
        x: cmd.x,
        y: cmd.y,
        wz: cmd.yaw,
        expiresAt: Date.now() + 500,
      };
      state.rotate.active = false;
      mode.setTwist({ x: cmd.x, y: cmd.y, yaw: cmd.yaw });
      break;
    case "foot_targets":
      await mode.setFootTargets(cmd.targets);
      break;
    case "joint_angles":
      await mode.setJointAngles(cmd.angles);
      break;
    case "servo_angles":
      await mode.setServoAnglesDeg(cmd.angles_deg);
      break;
    case "raw_servo_angles":
      await mode.setRawServoAnglesDeg(cmd.angles_deg);
      break;
    case "disconnect":
      serial.releaseServos();
      state.twist.active = false;
      state.rotate.active = false;
      await mode.setMode("idle");
      broadcast({ type: "disconnect" });
      break;
    case "stance_play":
    case "stance_save":
      await mode.setMode(cmd.name); // crouch / stand / extend
      break;
    case "stabilizer_set":
      mode.updateStabilizerParams(cmd.params);
      break;
    case "rotate_degrees": {
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
      mode.setTwist({ x: 0, y: 0, yaw: state.twist.wz });
      break;
    }
    case "set_servo_overrides":
      state.servoOverrides = await saveServoOverrides(cmd.overrides);
      setServoCalOverrides(state.servoOverrides);
      broadcast({ type: "settings:servo_overrides", payload: state.servoOverrides });
      break;
    case "set_joint_limits":
      state.jointLimits = await saveJointLimits(cmd.limits);
      mode.setJointLimits(state.jointLimits);
      broadcast({ type: "settings:joint_limits", payload: state.jointLimits });
      break;
    case "animation_play":
    case "animation_stop":
      // Animations are TODO; route through stance mode for now.
      await mode.setMode(cmd.type === "animation_play" ? "animation_playback" : "idle");
      break;
    case "recording_start":
      await recorder.start();
      broadcast({ type: "recording", status: recorder.status() });
      break;
    case "recording_stop":
      await recorder.stop();
      broadcast({ type: "recording", status: recorder.status() });
      break;
    default:
      throw new Error(`Command type ${cmd.type} has no server handler`);
  }
}

function mapGaitParam(name, value) {
  switch (name) {
    case "delta_x_mm":      return { delta_x: value / 1000 };
    case "delta_y_mm":      return { delta_y: value / 1000 };
    case "swing_time_ms":   return { swing_time_ms: value };
    case "rotate_rate_max": return { rotate_rate_max: value };
    case "default_z_ref_mm":return { default_z_ref: value / 1000 };
    default: return null;
  }
}

// ─── Twist decay loop ────────────────────────────────────────────────────
// Without a 30 Hz republish, the planner reads the latest twist anyway, but
// we still need a timer to expire stale twists and end rotate-by-degrees.
setInterval(() => {
  const now = Date.now();
  if (state.rotate.active && now >= state.rotate.endAt) {
    state.rotate.active = false;
    state.twist.active = false;
    mode.setTwist({ x: 0, y: 0, yaw: 0 });
    return;
  }
  if (state.twist.active && now > state.twist.expiresAt) {
    state.twist.active = false;
    mode.setTwist({ x: 0, y: 0, yaw: 0 });
  }
}, 50);

// ─── WebSocket fan-out ───────────────────────────────────────────────────

const httpServer = createServer(app);
const wss = new WebSocketServer({ server: httpServer, path: "/telemetry" });

wss.on("connection", (ws) => {
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

  ws.on("message", async (raw) => {
    try {
      const msg = JSON.parse(raw.toString());
      const cmd = validateCommand(msg);
      await handleCommand(cmd);
    } catch (err) {
      safeSend(ws, { type: "error", error: String(err.message || err) });
    }
  });
});

function broadcast(payload) {
  const str = JSON.stringify(payload);
  for (const client of wss.clients) {
    if (client.readyState === 1) {
      try { client.send(str); } catch { /* drop */ }
    }
  }
}

function safeSend(ws, payload) {
  try { ws.send(JSON.stringify(payload)); } catch { /* drop */ }
}

function appendRecorderSample() {
  if (!recorder.isActive) return;
  const { joint_states: js, imu, gas } = state.telemetry;
  const positions = {};
  if (js?.name && Array.isArray(js?.position)) {
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
  setServoCalOverrides(state.servoOverrides);
  state.jointLimits = await loadJointLimits();
  state.stances = await loadStances();
  state.rotateSettings = await loadRotateSettings();
  state.servoSpeed = await loadServoSpeed();
  state.servoUpdateRate = await loadServoUpdateRate();

  // Reapply persisted motion settings and walk the legs through a deterministic
  // wake-up sequence every time the ESP32 link comes up.
  serial.on("open", async () => {
    if (state.servoUpdateRate?.hz) mode.setServoUpdateRate(state.servoUpdateRate.hz);
    if (Object.keys(state.servoSpeed).length) mode.setServoSpeedLimit(state.servoSpeed);
    if (Object.keys(state.jointLimits).length) mode.setJointLimits(state.jointLimits);
    try {
      await mode.runStartupPoseSequence({
        standMs: STARTUP_STAND_MS,
        crouchMs: STARTUP_CROUCH_MS,
      });
    } catch (err) {
      console.warn("[argos-dashboard] startup pose sequence failed:", err?.message || err);
    }
  });

  serial.connect();
  sensors.connect();
  mode.start();

  httpServer.listen(HTTP_PORT, () => {
    console.log(`[argos-dashboard] HTTP :${HTTP_PORT}`);
    console.log(`[argos-dashboard] serial ${process.env.ARGOS_SERIAL_PORT || "/dev/ttyUSB0"}`);
    console.log(`[argos-dashboard] sensors ${process.env.ARGOS_SIDECAR_HOST || "127.0.0.1"}:${process.env.ARGOS_SIDECAR_PORT || 8788}`);
    console.log(`[argos-dashboard] static ${STATIC_DIR}`);
  });
}

bootstrap().catch((err) => {
  console.error("[argos-dashboard] bootstrap failed:", err);
  process.exit(1);
});

function shutdown(signal) {
  console.log(`[argos-dashboard] caught ${signal}, releasing servos`);
  try {
    serial.releaseServos();
  } catch { /* serial might be closed */ }
  if (recorder.isActive) recorder.stop().catch(() => {});
  mode.stop();
  // Hold the port open for ~200 ms so the release_servos frame queued above
  // has time to flush through the pump (port.drain() + pacingMs). Closing
  // serial synchronously here would race the pump and leave the servos
  // engaged after the process exits.
  setTimeout(() => {
    serial.disconnect();
    sensors.disconnect();
    process.exit(0);
  }, 200);
}
process.on("SIGINT", () => shutdown("SIGINT"));
process.on("SIGTERM", () => shutdown("SIGTERM"));
