import React, { useEffect, useMemo, useRef, useState } from "react";
import { toCanvasPoint, robotOverviewGeometry } from "../shared/kinematics.js";
import { DEFAULT_DRIVE_COMMAND, DEFAULT_JOINT_LIMITS, DEFAULT_LEG_COMMAND, DEFAULT_SERVO_CHANNEL_MAP, DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC, DEFAULT_SERVO_TRIM_DEG, LEG_DRAWING, LEG_IDS, LEG_LABELS } from "../shared/robot-config.js";

const INPUT_ACTIONS = ["forward", "backward", "strafeLeft", "strafeRight", "rotateLeft", "rotateRight"];
const KEY_TO_ACTION = {
  ArrowUp: "forward",
  KeyW: "forward",
  ArrowDown: "backward",
  KeyS: "backward",
  ArrowLeft: "strafeLeft",
  KeyA: "strafeLeft",
  ArrowRight: "strafeRight",
  KeyD: "strafeRight",
  KeyQ: "rotateLeft",
  KeyE: "rotateRight",
};

function stripTrailingSlash(value) {
  return value.replace(/\/+$/, "");
}

function getBackendHttpBaseUrl() {
  const configured = import.meta.env.VITE_BACKEND_URL?.trim();
  if (configured) {
    return stripTrailingSlash(configured);
  }

  if (import.meta.env.DEV) {
    return `${window.location.protocol}//${window.location.hostname}:8787`;
  }

  return "";
}

function getBackendWsUrl() {
  const backendHttpBaseUrl = getBackendHttpBaseUrl();
  if (backendHttpBaseUrl) {
    return `${backendHttpBaseUrl.replace(/^http/i, "ws")}/telemetry`;
  }

  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/telemetry`;
}

function clone(value) {
  return JSON.parse(JSON.stringify(value));
}

function createLocalState() {
  const legs = {};
  for (const legId of LEG_IDS) {
    legs[legId] = {
      desired: {
        geometry: null,
        foot: { ...DEFAULT_LEG_COMMAND.foot },
        jointAnglesDeg: { ...DEFAULT_LEG_COMMAND.jointAnglesDeg },
        servoAnglesDeg: { ...DEFAULT_LEG_COMMAND.servoAnglesDeg },
      },
      current: {
        geometry: null,
        foot: { ...DEFAULT_LEG_COMMAND.foot },
        jointAnglesDeg: { ...DEFAULT_LEG_COMMAND.jointAnglesDeg },
        servoAnglesDeg: { ...DEFAULT_LEG_COMMAND.servoAnglesDeg },
      },
      status: "idle",
      lastError: "",
      servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
      jointLimits: clone(DEFAULT_JOINT_LIMITS),
      servoSpeedLimitDegPerSec: { ...DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC },
      servoTrimDeg: { ...DEFAULT_SERVO_TRIM_DEG },
    };
  }

  return {
    connected: false,
    esp32Connected: false,
    connectedPort: null,
    mode: "idle",
    motionMode: "idle",
    driveCommand: { ...DEFAULT_DRIVE_COMMAND },
    servosReleased: false,
    activeAnimation: null,
    lastAck: null,
    lastError: null,
    faults: [],
    firmwareMs: 0,
    uptimeMs: 0,
    ports: [],
    robotConfig: null,
    legs,
  };
}

function mergeState(base, patch) {
  return {
    ...base,
    ...patch,
    driveCommand: {
      ...base.driveCommand,
      ...patch.driveCommand,
    },
    legs: {
      ...base.legs,
      ...patch.legs,
    },
  };
}

function segmentPath(a, b) {
  return `M ${a.x} ${a.y} L ${b.x} ${b.y}`;
}

function formatAxis(value) {
  return Number(value ?? 0).toFixed(2);
}

function deriveDriveCommand(activeInputs) {
  const vx = (activeInputs.forward ? 1 : 0) - (activeInputs.backward ? 1 : 0);
  const vy = (activeInputs.strafeRight ? 1 : 0) - (activeInputs.strafeLeft ? 1 : 0);
  const yawRate = (activeInputs.rotateRight ? 1 : 0) - (activeInputs.rotateLeft ? 1 : 0);
  return {
    vx,
    vy,
    yawRate,
    source: "web",
  };
}

function CardStat({ label, value, accent }) {
  return (
    <div className={`status-card ${accent ? `status-card-${accent}` : ""}`}>
      <label>{label}</label>
      <strong>{value}</strong>
    </div>
  );
}

function ConnectionPanel({ robotState, selectedPort, setSelectedPort, connect, disconnect, refreshStatus }) {
  return (
    <section className="panel panel-header">
      <div className="panel-title-row">
        <div>
          <p className="eyebrow">Production Control</p>
          <h1>Argos Robot Dog Driver</h1>
        </div>
        <div className={`connection-pill ${robotState.esp32Connected ? "online" : "offline"}`}>
          {robotState.esp32Connected ? "ESP32-C6 Connected" : "ESP32-C6 Offline"}
        </div>
      </div>
      <div className="toolbar toolbar-spread">
        <label className="inline-field">
          ESP32 Port
          <select value={selectedPort} onChange={(event) => setSelectedPort(event.target.value)}>
            <option value="">Select serial port</option>
            {robotState.ports.map((port) => (
              <option key={port.path} value={port.path}>
                {port.path} {port.manufacturer ? `- ${port.manufacturer}` : ""}
              </option>
            ))}
          </select>
        </label>
        <div className="toolbar">
          <button className="ghost-button" onClick={refreshStatus}>Refresh</button>
          {robotState.esp32Connected ? (
            <button className="ghost-button" onClick={disconnect}>Disconnect</button>
          ) : (
            <button className="accent-button" onClick={connect}>Connect ESP32-C6</button>
          )}
        </div>
      </div>
    </section>
  );
}

function DrivePad({ setAction }) {
  function actionHandlers(action) {
    return {
      onPointerDown: () => setAction(action, true),
      onPointerUp: () => setAction(action, false),
      onPointerLeave: () => setAction(action, false),
      onPointerCancel: () => setAction(action, false),
    };
  }

  return (
    <div className="drive-pad-grid">
      <button className="drive-button" {...actionHandlers("forward")}>Forward</button>
      <button className="drive-button" {...actionHandlers("rotateLeft")}>Rotate Left</button>
      <button className="drive-button" {...actionHandlers("strafeLeft")}>Strafe Left</button>
      <button className="drive-button drive-button-stop" {...actionHandlers("backward")}>Back</button>
      <button className="drive-button" {...actionHandlers("strafeRight")}>Strafe Right</button>
      <button className="drive-button" {...actionHandlers("rotateRight")}>Rotate Right</button>
    </div>
  );
}

function RobotOverview({ robotState, poseSource = "desired" }) {
  const overview = robotOverviewGeometry(robotState, poseSource);
  const width = 700;
  const height = 420;
  const center = { x: width / 2, y: height / 2 };
  const bodyWidth = 280;
  const bodyHeight = 110;

  const toPoint = (point) => ({
    x: center.x + point.x,
    y: center.y + point.y,
  });

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Body Pose</h2>
        <span className="chip">{poseSource === "desired" ? "Pi target pose" : "Latest reported pose"}</span>
      </div>
      <svg className="overview-svg" viewBox={`0 0 ${width} ${height}`}>
        <defs>
          <linearGradient id="body-fill" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#ffd76f" />
            <stop offset="100%" stopColor="#f58f47" />
          </linearGradient>
        </defs>
        <rect x={center.x - bodyWidth / 2} y={center.y - bodyHeight / 2} width={bodyWidth} height={bodyHeight} rx="30" fill="url(#body-fill)" />
        {overview.map((leg) => {
          if (!leg.points || leg.points.length < 3) {
            return null;
          }
          const [hip, knee, foot] = leg.points.map(toPoint);
          return (
            <g key={leg.legId}>
              <path d={segmentPath(hip, knee)} stroke="#1c1c1c" strokeWidth="7" fill="none" strokeLinecap="round" />
              <path d={segmentPath(knee, foot)} stroke="#0078ff" strokeWidth="7" fill="none" strokeLinecap="round" />
              <circle cx={foot.x} cy={foot.y} r="7" fill="#0078ff" />
              <text x={hip.x + 12} y={hip.y - 12} className="overview-label">{LEG_LABELS[leg.legId]}</text>
            </g>
          );
        })}
      </svg>
    </section>
  );
}

function TrimField({ label, value, onChange }) {
  return (
    <label className="inline-field">
      {label}
      <input
        type="number"
        min="-45"
        max="45"
        step="1"
        value={value}
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  );
}

function LegDebugPanel({ legId, legState, source, applyServoTrim, resetServoTrim }) {
  const [draftTrim, setDraftTrim] = useState(() => ({
    hipYaw: legState?.servoTrimDeg?.hipYaw ?? 0,
    thigh: legState?.servoTrimDeg?.thigh ?? 0,
    calf: legState?.servoTrimDeg?.calf ?? 0,
  }));

  useEffect(() => {
    setDraftTrim({
      hipYaw: legState?.servoTrimDeg?.hipYaw ?? 0,
      thigh: legState?.servoTrimDeg?.thigh ?? 0,
      calf: legState?.servoTrimDeg?.calf ?? 0,
    });
  }, [legState?.servoTrimDeg?.hipYaw, legState?.servoTrimDeg?.thigh, legState?.servoTrimDeg?.calf]);

  const geometry = legState?.[source]?.geometry;
  if (!geometry) {
    return (
      <section className="panel">
        <h3>{LEG_LABELS[legId]}</h3>
        <p className="muted-copy">No geometry available yet.</p>
      </section>
    );
  }

  const points = {
    hip: toCanvasPoint(geometry.hip, LEG_DRAWING),
    knee: toCanvasPoint(geometry.knee, LEG_DRAWING),
    foot: toCanvasPoint(geometry.foot, LEG_DRAWING),
    servoPivot: toCanvasPoint(geometry.servoPivot, LEG_DRAWING),
    servoHornEnd: toCanvasPoint(geometry.servoHornEnd, LEG_DRAWING),
  };

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h3>{LEG_LABELS[legId]}</h3>
        <span className="chip">{source}</span>
      </div>
      <svg className="leg-svg" viewBox={`0 0 ${LEG_DRAWING.stageWidth} ${LEG_DRAWING.stageHeight}`}>
        <path d={segmentPath(points.hip, points.knee)} stroke="#141414" strokeWidth="5" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.knee, points.foot)} stroke="#1d8cff" strokeWidth="5" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.servoPivot, points.servoHornEnd)} stroke="#ff5232" strokeWidth="4" fill="none" strokeLinecap="round" />
        {Object.values(points).map((point, index) => (
          <circle key={`${legId}-${index}`} cx={point.x} cy={point.y} r="5" fill="#121212" />
        ))}
      </svg>
      <div className="stats-grid">
        <div>
          <label>Hip yaw</label>
          <strong>{legState[source].jointAnglesDeg.hipYaw.toFixed(1)} deg</strong>
        </div>
        <div>
          <label>Thigh</label>
          <strong>{legState[source].jointAnglesDeg.thigh.toFixed(1)} deg</strong>
        </div>
        <div>
          <label>Calf</label>
          <strong>{legState[source].jointAnglesDeg.calf.toFixed(1)} deg</strong>
        </div>
        <div>
          <label>Servos</label>
          <strong>
            {legState[source].servoAnglesDeg.hipYaw.toFixed(0)} / {legState[source].servoAnglesDeg.thigh.toFixed(0)} / {legState[source].servoAnglesDeg.calf.toFixed(0)}
          </strong>
        </div>
      </div>
      <div className="panel-title-row">
        <h4>Neutral Trim</h4>
        <div className="toolbar">
          <button type="button" className="accent-button" onClick={() => applyServoTrim(legId, draftTrim)}>Apply</button>
          <button
            type="button"
            className="ghost-button"
            onClick={() => {
              setDraftTrim({ hipYaw: 0, thigh: 0, calf: 0 });
              resetServoTrim(legId);
            }}
          >
            Zero
          </button>
        </div>
      </div>
      <div className="toolbar">
        <TrimField
          label="Hip yaw"
          value={draftTrim.hipYaw}
          onChange={(value) => setDraftTrim((current) => ({ ...current, hipYaw: value }))}
        />
        <TrimField
          label="Thigh"
          value={draftTrim.thigh}
          onChange={(value) => setDraftTrim((current) => ({ ...current, thigh: value }))}
        />
        <TrimField
          label="Calf"
          value={draftTrim.calf}
          onChange={(value) => setDraftTrim((current) => ({ ...current, calf: value }))}
        />
      </div>
    </section>
  );
}

export default function App() {
  const [robotState, setRobotState] = useState(createLocalState);
  const [selectedPort, setSelectedPort] = useState("");
  const [tab, setTab] = useState("drive");
  const [poseSource, setPoseSource] = useState("desired");
  const [activeInputs, setActiveInputs] = useState(() => Object.fromEntries(INPUT_ACTIONS.map((action) => [action, false])));
  const telemetrySocketRef = useRef(null);
  const lastDriveSignatureRef = useRef("");

  const driveCommand = useMemo(() => deriveDriveCommand(activeInputs), [activeInputs]);
  const hasMotion = driveCommand.vx !== 0 || driveCommand.vy !== 0 || driveCommand.yawRate !== 0;

  async function postJson(path, body) {
    const response = await fetch(`${getBackendHttpBaseUrl()}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.error || "Request failed");
    }
    return payload;
  }

  function setAction(action, active) {
    setActiveInputs((current) => ({ ...current, [action]: active }));
  }

  function clearActions() {
    setActiveInputs(Object.fromEntries(INPUT_ACTIONS.map((action) => [action, false])));
  }

  async function sendCommand(command) {
    const socket = telemetrySocketRef.current;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: "command", command }));
      return;
    }
    await postJson("/api/command", { command });
  }

  async function fetchStatus() {
    const response = await fetch(`${getBackendHttpBaseUrl()}/api/status`);
    const payload = await response.json();
    setRobotState((current) => mergeState(current, payload));
    if (!selectedPort && payload.connectedPort) {
      setSelectedPort(payload.connectedPort);
    }
  }

  useEffect(() => {
    fetchStatus().catch((error) => setRobotState((current) => ({ ...current, lastError: error.message })));
    const socket = new WebSocket(getBackendWsUrl());
    telemetrySocketRef.current = socket;

    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "status" || message.type === "state" || message.type === "hello_ack") {
          setRobotState((current) => mergeState(current, message.payload));
        }
        if (message.type === "ack") {
          setRobotState((current) => ({ ...current, lastAck: message.message ?? "ack" }));
        }
        if (message.type === "error") {
          setRobotState((current) => ({ ...current, lastError: message.message || "Unknown controller error" }));
        }
      } catch (error) {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      }
    };

    return () => {
      telemetrySocketRef.current = null;
      socket.close();
    };
  }, []);

  useEffect(() => {
    function handleKeyDown(event) {
      const action = KEY_TO_ACTION[event.code];
      if (!action) {
        return;
      }
      if (event.repeat) {
        return;
      }
      event.preventDefault();
      setAction(action, true);
    }

    function handleKeyUp(event) {
      const action = KEY_TO_ACTION[event.code];
      if (!action) {
        return;
      }
      event.preventDefault();
      setAction(action, false);
    }

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    window.addEventListener("blur", clearActions);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      window.removeEventListener("blur", clearActions);
    };
  }, []);

  useEffect(() => {
    const signature = JSON.stringify(driveCommand);
    if (signature === lastDriveSignatureRef.current) {
      return;
    }
    lastDriveSignatureRef.current = signature;

    const command = hasMotion
      ? { type: "set_drive_command", drive: driveCommand }
      : { type: "stop_motion" };

    sendCommand(command).catch((error) => {
      setRobotState((current) => ({ ...current, lastError: error.message }));
    });
  }, [driveCommand, hasMotion]);

  useEffect(() => {
    if (!hasMotion) {
      return undefined;
    }

    const intervalId = window.setInterval(() => {
      sendCommand({ type: "set_drive_command", drive: deriveDriveCommand(activeInputs) }).catch((error) => {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      });
    }, 100);

    return () => {
      window.clearInterval(intervalId);
    };
  }, [activeInputs, hasMotion]);

  function withError(handler) {
    return (...args) => handler(...args).catch((error) => {
      setRobotState((current) => ({ ...current, lastError: error.message }));
    });
  }

  async function connect() {
    if (!selectedPort) {
      throw new Error("Select a serial port before connecting.");
    }
    await postJson("/api/connect", { path: selectedPort });
    await fetchStatus();
  }

  async function disconnect() {
    await postJson("/api/disconnect", {});
    await fetchStatus();
  }

  async function setMotionMode(mode) {
    await sendCommand({ type: "set_motion_mode", mode });
  }

  async function stopMotion() {
    clearActions();
    await sendCommand({ type: "stop_motion" });
  }

  async function panicRelease() {
    clearActions();
    await sendCommand({ type: "panic_release" });
  }

  async function applyServoTrim(legId, trim) {
    setRobotState((current) => ({
      ...current,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          servoTrimDeg: {
            hipYaw: trim.hipYaw ?? DEFAULT_SERVO_TRIM_DEG.hipYaw,
            thigh: trim.thigh ?? DEFAULT_SERVO_TRIM_DEG.thigh,
            calf: trim.calf ?? DEFAULT_SERVO_TRIM_DEG.calf,
          },
        },
      },
    }));

    await sendCommand({
      type: "set_leg_servo_trim",
      legId,
      hipYawOffsetDeg: trim.hipYaw ?? DEFAULT_SERVO_TRIM_DEG.hipYaw,
      thighOffsetDeg: trim.thigh ?? DEFAULT_SERVO_TRIM_DEG.thigh,
      calfOffsetDeg: trim.calf ?? DEFAULT_SERVO_TRIM_DEG.calf,
    });
  }

  async function resetServoTrim(legId) {
    setRobotState((current) => ({
      ...current,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          servoTrimDeg: { ...DEFAULT_SERVO_TRIM_DEG },
        },
      },
    }));

    await sendCommand({
      type: "set_leg_servo_trim",
      legId,
      hipYawOffsetDeg: 0,
      thighOffsetDeg: 0,
      calfOffsetDeg: 0,
    });
  }

  return (
    <main className="app-shell">
      <button className={`panic-button ${robotState.servosReleased ? "panic-button-released" : ""}`} onClick={withError(panicRelease)}>
        {robotState.servosReleased ? "Servos Released" : "PANIC RELEASE"}
      </button>

      <ConnectionPanel
        robotState={robotState}
        selectedPort={selectedPort}
        setSelectedPort={setSelectedPort}
        connect={withError(connect)}
        disconnect={withError(disconnect)}
        refreshStatus={withError(fetchStatus)}
      />

      <div className="toolbar toolbar-spread toolbar-top">
        <div className="toggle-group">
          <button className={tab === "drive" ? "toggle-active" : ""} onClick={() => setTab("drive")}>Drive</button>
          <button className={tab === "debug" ? "toggle-active" : ""} onClick={() => setTab("debug")}>Debug</button>
        </div>
        <div className="toggle-group">
          <button className={poseSource === "desired" ? "toggle-active" : ""} onClick={() => setPoseSource("desired")}>Desired pose</button>
          <button className={poseSource === "current" ? "toggle-active" : ""} onClick={() => setPoseSource("current")}>Current pose</button>
        </div>
      </div>

      {tab === "drive" ? (
        <>
          <section className="drive-layout">
            <section className="panel panel-drive">
              <div className="panel-title-row">
                <div>
                  <p className="eyebrow">Teleop</p>
                  <h2>Movement Control</h2>
                </div>
                <span className="chip">Keyboard: WASD / Arrows / Q / E</span>
              </div>
              <DrivePad setAction={setAction} />
              <div className="toolbar">
                <button className="accent-button" onClick={withError(() => setMotionMode("stand"))}>Stand</button>
                <button className="accent-button accent-button-secondary" onClick={withError(() => setMotionMode("calibration"))}>Calibration</button>
                <button className="ghost-button" onClick={withError(stopMotion)}>Stop</button>
              </div>
            </section>

            <section className="panel">
              <div className="panel-title-row">
                <h2>Robot Status</h2>
                <span className={`connection-pill ${robotState.esp32Connected ? "online" : "offline"}`}>
                  {robotState.connectedPort || "No serial link"}
                </span>
              </div>
              <div className="status-grid">
                <CardStat label="Motion mode" value={robotState.motionMode} accent="mode" />
                <CardStat label="ESP32 link" value={robotState.esp32Connected ? "online" : "offline"} accent={robotState.esp32Connected ? "good" : "bad"} />
                <CardStat label="Forward" value={formatAxis(robotState.driveCommand.vx)} />
                <CardStat label="Strafe" value={formatAxis(robotState.driveCommand.vy)} />
                <CardStat label="Rotate" value={formatAxis(robotState.driveCommand.yawRate)} />
                <CardStat label="Last ack" value={robotState.lastAck || "none"} />
              </div>
              <div className="fault-box">
                <label>Faults</label>
                <p>{robotState.faults.length ? robotState.faults.join(" | ") : "No active faults."}</p>
              </div>
            </section>
          </section>

          <RobotOverview robotState={robotState} poseSource="desired" />
        </>
      ) : (
        <>
          <RobotOverview robotState={robotState} poseSource={poseSource} />
          <section className="details-grid">
            {LEG_IDS.map((legId) => (
              <LegDebugPanel
                key={legId}
                legId={legId}
                legState={robotState.legs[legId]}
                source={poseSource}
                applyServoTrim={withError(applyServoTrim)}
                resetServoTrim={withError(resetServoTrim)}
              />
            ))}
          </section>
        </>
      )}
    </main>
  );
}
