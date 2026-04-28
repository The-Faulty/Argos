// Pi-server telemetry socket hook.
//
// One WS per tab to the Pi Node server. The server fans every client the
// cached snapshot of joint_states / imu / gas / mode / settings, then pushes
// deltas. The hook reduces those messages into a flat `data` object the rest
// of the UI consumes.
//
// Shape of what `data` holds (filled progressively as messages arrive):
//   {
//     connected: { ws: bool, serial: bool, sensors: bool, lastEvent: {...} },
//     mode: string,
//     rotate_settings, servo_overrides, stances,
//     joint_states: {name, position, ...} | null,
//     imu: {_euler_rad: {roll,pitch,yaw}, ...} | null,
//     depth: {center, roi, nearest_m, valid_pct, ...} | null,
//     gas: {data: number} | null,
//     gait_mode: {data: string} | null,
//     control_mode: {data: string} | null,
//     leg_ids, joint_names, mode_options,
//     recording: {active, rowCount, ...},
//     lastError: string | null,
//   }
//
// `ws` reflects the browser↔Pi socket; `serial` reflects the ESP32 link;
// `sensors` reflects the Python sidecar (RealSense + MLX90640).
//
// Auto-reconnect with capped backoff so the UI recovers silently if the Pi
// server restarts mid-demo.

import { useEffect, useRef, useState } from "react";

const DEFAULT_URL = (() => {
  if (typeof window === "undefined") return "ws://localhost:8787/telemetry";
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${window.location.host}/telemetry`;
})();

const INITIAL_BACKOFF_MS = 500;
const MAX_BACKOFF_MS = 8_000;

export function useRosbridge({ url = DEFAULT_URL } = {}) {
  const [data, setData] = useState(() => ({
    connected: { ws: false, serial: false, sensors: false, lastEvent: null },
    mode: "idle",
    rotate_settings: null,
    servo_overrides: {},
    servo_speed: null,
    servo_update_rate: null,
    stances: {},
    joint_states: null,
    imu: null,
    depth: null,
    gas: null,
    thermal: null,
    gait_mode: null,
    control_mode: null,
    leg_ids: ["FR", "FL", "RR", "RL"],
    joint_names: [],
    mode_options: [],
    recording: { active: false, rowCount: 0 },
    lastError: null,
  }));

  const wsRef = useRef(null);
  const backoffRef = useRef(INITIAL_BACKOFF_MS);
  const reconnectTimerRef = useRef(null);
  const aliveRef = useRef(true);

  useEffect(() => {
    aliveRef.current = true;

    function connect() {
      if (!aliveRef.current) return;
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.addEventListener("open", () => {
        backoffRef.current = INITIAL_BACKOFF_MS;
        setData((prev) => ({ ...prev, connected: { ...prev.connected, ws: true } }));
      });

      ws.addEventListener("close", () => {
        setData((prev) => ({ ...prev, connected: { ...prev.connected, ws: false } }));
        scheduleReconnect();
      });

      ws.addEventListener("error", () => {
        // `close` fires right after — no separate handling needed, just let
        // the reconnect kick in from the close event.
      });

      ws.addEventListener("message", (event) => {
        try {
          const msg = JSON.parse(event.data);
          setData((prev) => reduce(prev, msg));
        } catch (err) {
          setData((prev) => ({ ...prev, lastError: String(err.message || err) }));
        }
      });
    }

    function scheduleReconnect() {
      if (!aliveRef.current) return;
      if (reconnectTimerRef.current) return;
      const delay = backoffRef.current;
      backoffRef.current = Math.min(MAX_BACKOFF_MS, delay * 2);
      reconnectTimerRef.current = setTimeout(() => {
        reconnectTimerRef.current = null;
        connect();
      }, delay);
    }

    connect();

    return () => {
      aliveRef.current = false;
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      if (wsRef.current) {
        try {
          wsRef.current.close();
        } catch {
          /* already closed */
        }
      }
    };
  }, [url]);

  // Allow components (e.g. the Twist joystick or debug console) to send a
  // command straight down the WS instead of the HTTP API.
  function send(obj) {
    const ws = wsRef.current;
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }

  return { data, send };
}

function reduce(prev, msg) {
  switch (msg.type) {
    case "snapshot":
      return {
        ...prev,
        connected: { ...prev.connected, ...(msg.telemetry?.connected ?? {}) },
        mode: msg.mode ?? prev.mode,
        rotate_settings: msg.rotate_settings ?? prev.rotate_settings,
        servo_overrides: msg.servo_overrides ?? prev.servo_overrides,
        servo_speed: msg.servo_speed ?? prev.servo_speed,
        servo_update_rate: msg.servo_update_rate ?? prev.servo_update_rate,
        stances: msg.stances ?? prev.stances,
        joint_states: msg.telemetry?.joint_states ?? prev.joint_states,
        imu: msg.telemetry?.imu ?? prev.imu,
        depth: msg.telemetry?.depth ?? prev.depth,
        gas: msg.telemetry?.gas ?? prev.gas,
        thermal: msg.telemetry?.thermal ?? prev.thermal,
        gait_mode: msg.telemetry?.gait_mode ?? prev.gait_mode,
        control_mode: msg.telemetry?.control_mode ?? prev.control_mode,
        leg_ids: msg.leg_ids ?? prev.leg_ids,
        joint_names: msg.joint_names ?? prev.joint_names,
        mode_options: msg.mode_options ?? prev.mode_options,
      };
    case "connected":
      return { ...prev, connected: { ...prev.connected, ...(msg.connected ?? {}) } };
    case "joint_states":
      return { ...prev, joint_states: msg.msg };
    case "imu":
      return { ...prev, imu: msg.msg };
    case "depth":
      return { ...prev, depth: msg.msg };
    case "gas":
      return { ...prev, gas: msg.msg };
    case "thermal":
      return { ...prev, thermal: msg.msg };
    case "gait_mode":
      return { ...prev, gait_mode: msg.msg };
    case "control_mode":
      return { ...prev, control_mode: msg.msg };
    case "mode":
      return { ...prev, mode: msg.mode };
    case "settings:rotate":
      return { ...prev, rotate_settings: msg.payload };
    case "settings:servo_overrides":
      return { ...prev, servo_overrides: msg.payload };
    case "settings:servo_speed":
      return { ...prev, servo_speed: msg.payload };
    case "settings:servo_update_rate":
      return { ...prev, servo_update_rate: msg.payload };
    case "stances":
      return { ...prev, stances: msg.payload };
    case "recording":
      return { ...prev, recording: msg.status };
    case "disconnect":
      return { ...prev, mode: "idle" };
    case "error":
      return { ...prev, lastError: msg.error };
    default:
      return prev;
  }
}
