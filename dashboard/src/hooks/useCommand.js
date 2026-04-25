// Thin fetch wrapper around the Pi-side HTTP API.
//
// Every panel posts JSON against a stable base URL (same-origin by default,
// overridable via VITE_ARGOS_SERVER_URL for split-origin dev). Errors are
// captured into hook state so the UI can surface "last command failed"
// without every component repeating its own try/catch.

import { useCallback, useState } from "react";

const BASE_URL =
  (typeof import.meta !== "undefined" && import.meta.env?.VITE_ARGOS_SERVER_URL) || "";

async function jsonRequest(path, { method = "POST", body } = {}) {
  const headers = {};
  let payload;
  if (body !== undefined) {
    headers["Content-Type"] = "application/json";
    payload = JSON.stringify(body);
  }
  const response = await fetch(`${BASE_URL}${path}`, {
    method,
    headers,
    body: payload,
  });
  const text = await response.text();
  const parsed = text ? safeParse(text) : null;
  if (!response.ok) {
    const message = parsed?.error || `HTTP ${response.status}`;
    const err = new Error(message);
    err.status = response.status;
    err.body = parsed;
    throw err;
  }
  return parsed;
}

function safeParse(text) {
  try {
    return JSON.parse(text);
  } catch {
    return { raw: text };
  }
}

export function useCommand() {
  const [lastError, setLastError] = useState(null);
  const [inFlight, setInFlight] = useState(0);

  const wrap = useCallback((fn) => async (...args) => {
    setInFlight((n) => n + 1);
    try {
      const result = await fn(...args);
      setLastError(null);
      return result;
    } catch (err) {
      setLastError(String(err.message || err));
      throw err;
    } finally {
      setInFlight((n) => Math.max(0, n - 1));
    }
  }, []);

  const sendCommand = useCallback(
    wrap((cmd) => jsonRequest("/api/command", { body: cmd })),
    [wrap],
  );

  const setMode = useCallback(
    wrap((mode) => jsonRequest("/api/command", { body: { type: "set_mode", mode } })),
    [wrap],
  );

  const disconnect = useCallback(
    wrap(() => jsonRequest("/api/command", { body: { type: "disconnect" } })),
    [wrap],
  );

  const rotate = useCallback(
    wrap((direction, degrees) =>
      jsonRequest("/api/rotate", { body: { direction, degrees } }),
    ),
    [wrap],
  );

  const sendTwist = useCallback(
    wrap((x, y, yaw) =>
      jsonRequest("/api/command", { body: { type: "twist", x, y, yaw } }),
    ),
    [wrap],
  );

  const sendFootTargets = useCallback(
    wrap((targets) =>
      jsonRequest("/api/command", { body: { type: "foot_targets", targets } }),
    ),
    [wrap],
  );

  const sendJointAngles = useCallback(
    wrap((angles) =>
      jsonRequest("/api/command", { body: { type: "joint_angles", angles } }),
    ),
    [wrap],
  );

  const sendServoAngles = useCallback(
    wrap((angles_deg) =>
      jsonRequest("/api/command", { body: { type: "servo_angles", angles_deg } }),
    ),
    [wrap],
  );

  const saveStance = useCallback(
    wrap((name, angles_rad) =>
      jsonRequest(`/api/stances/${encodeURIComponent(name)}`, {
        body: angles_rad ? { angles_rad } : undefined,
      }),
    ),
    [wrap],
  );

  const playStance = useCallback(
    wrap((name) =>
      jsonRequest(`/api/stances/${encodeURIComponent(name)}/play`, { body: {} }),
    ),
    [wrap],
  );

  const setGaitParam = useCallback(
    wrap((name, value) =>
      jsonRequest(`/api/params/gait/${encodeURIComponent(name)}`, { body: { value } }),
    ),
    [wrap],
  );

  const setStabilizerParams = useCallback(
    wrap((params) => jsonRequest("/api/params/stabilizer", { body: params })),
    [wrap],
  );

  const setServoOverrides = useCallback(
    wrap((overrides) =>
      jsonRequest("/api/settings/servo_overrides", { body: overrides }),
    ),
    [wrap],
  );

  const setJointLimits = useCallback(
    wrap((limits) => jsonRequest("/api/settings/joint_limits", { body: limits })),
    [wrap],
  );

  const setRotateSettings = useCallback(
    wrap((settings) => jsonRequest("/api/settings/rotate", { body: settings })),
    [wrap],
  );

  const setServoSpeedLimits = useCallback(
    wrap((limits) => jsonRequest("/api/settings/servo_speed", { body: limits })),
    [wrap],
  );

  const setServoUpdateRate = useCallback(
    wrap((hz) => jsonRequest("/api/settings/servo_update_rate", { body: { hz } })),
    [wrap],
  );

  const startRecording = useCallback(
    wrap(() => jsonRequest("/api/recording/start", { body: {} })),
    [wrap],
  );
  const stopRecording = useCallback(
    wrap(() => jsonRequest("/api/recording/stop", { body: {} })),
    [wrap],
  );
  const listRecordings = useCallback(
    wrap(() => jsonRequest("/api/recording/list", { method: "GET" })),
    [wrap],
  );

  return {
    lastError,
    inFlight,
    sendCommand,
    setMode,
    disconnect,
    rotate,
    sendTwist,
    sendFootTargets,
    sendJointAngles,
    sendServoAngles,
    saveStance,
    playStance,
    setGaitParam,
    setStabilizerParams,
    setServoOverrides,
    setJointLimits,
    setRotateSettings,
    setServoSpeedLimits,
    setServoUpdateRate,
    startRecording,
    stopRecording,
    listRecordings,
  };
}
