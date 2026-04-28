import React, { useEffect, useMemo, useRef, useState } from "react";
import { convertLegacyLegClip, createUploadFrames, interpolateFullBodyClip, validateClip } from "../shared/animation.js";
import {
  buildLegPoseFromFoot,
  buildLegPoseFromJointAngles,
  buildLegPoseFromServoAngles,
  createNeutralCalibration,
  normalizeJointLimits,
  robotOverviewGeometry,
  toCanvasPoint,
} from "../shared/kinematics.js";
import { createMotionStatePatch } from "../shared/locomotion.js";
import {
  DEFAULT_DRIVE_COMMAND,
  DEFAULT_FULL_BODY_CLIP,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_LEG_COMMAND,
  DEFAULT_SERVO_CHANNEL_MAP,
  DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
  DEFAULT_SERVO_TRIM_DEG,
  DEFAULT_STANCE,
  LEG_DRAWING,
  LEG_GEOMETRY,
  LEG_IDS,
  LEG_LABELS,
  ROBOT_LAYOUT,
  STANCE_HEIGHT_RANGE_MM,
} from "../shared/robot-config.js";

const calibration = createNeutralCalibration();
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
const LIVE_COMMAND_INTERVAL_MS = 25;

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

function numberValue(value, fallback = 0) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function clampChannel(value) {
  return Math.max(0, Math.min(15, Math.round(numberValue(value))));
}

function clampServoAngle(value) {
  return Math.max(0, Math.min(180, numberValue(value, 90)));
}

function clampServoSpeed(value, fallback) {
  return Math.max(1, numberValue(value, fallback));
}

function getLinkConnected(robotState) {
  return Boolean(robotState.esp32Connected ?? robotState.connected);
}

function getMotionMode(robotState) {
  return robotState.motionMode ?? robotState.mode ?? "idle";
}

function formatAxis(value) {
  return numberValue(value).toFixed(2);
}

function formatStanceHeight(value) {
  const height = numberValue(value, DEFAULT_STANCE.height);
  if (height > 0) {
    return `+${height.toFixed(0)} mm`;
  }
  return `${height.toFixed(0)} mm`;
}

function formatPoint(point) {
  if (!point) {
    return "-";
  }
  return `${numberValue(point.x).toFixed(1)}, ${numberValue(point.y).toFixed(1)}`;
}

function formatTriple(values, digits = 1) {
  if (!values) {
    return "-";
  }

  return `${numberValue(values.hipYaw, 0).toFixed(digits)} / ${numberValue(values.thigh, 0).toFixed(digits)} / ${numberValue(values.calf, 0).toFixed(digits)}`;
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

function fromCanvasPoint(point, drawing = LEG_DRAWING) {
  return {
    x: (point.x - drawing.offset.x) / drawing.scale,
    y: -(point.y - drawing.offset.y) / drawing.scale,
  };
}

function toRelativeFoot(absolutePoint) {
  return {
    x: absolutePoint.x - LEG_GEOMETRY.footOriginOffset.x,
    y: absolutePoint.y - LEG_GEOMETRY.footOriginOffset.y,
  };
}

function segmentPath(a, b) {
  return `M ${a.x} ${a.y} L ${b.x} ${b.y}`;
}

function downloadJson(filename, data) {
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

function createDefaultLegPose(jointLimits = DEFAULT_JOINT_LIMITS) {
  return buildLegPoseFromJointAngles(DEFAULT_LEG_COMMAND.jointAnglesDeg, calibration, { jointLimits });
}

function createLocalState() {
  const legs = {};
  for (const legId of LEG_IDS) {
    const defaultPose = createDefaultLegPose(DEFAULT_JOINT_LIMITS);
    legs[legId] = {
      desired: clone(defaultPose),
      current: clone(defaultPose),
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
    stance: { ...DEFAULT_STANCE },
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

function createDraftCommandState() {
  return Object.fromEntries(
    LEG_IDS.map((legId) => [
      legId,
      {
        foot: { ...DEFAULT_LEG_COMMAND.foot },
        jointAnglesDeg: { ...DEFAULT_LEG_COMMAND.jointAnglesDeg },
        servoAnglesDeg: { ...DEFAULT_LEG_COMMAND.servoAnglesDeg },
        servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
        jointLimits: clone(DEFAULT_JOINT_LIMITS),
        servoSpeedLimitDegPerSec: { ...DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC },
      },
    ]),
  );
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

function applyMotionPatchToState(base, { motionMode = "stand", driveCommand = DEFAULT_DRIVE_COMMAND, stance = DEFAULT_STANCE, timeMs = 0 } = {}) {
  const jointLimitsByLeg = Object.fromEntries(
    LEG_IDS.map((legId) => [legId, base.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS]),
  );
  const patch = createMotionStatePatch({
    driveCommand,
    motionMode,
    stance,
    timeMs,
    jointLimitsByLeg,
  });
  const legs = { ...base.legs };

  for (const legId of LEG_IDS) {
    legs[legId] = {
      ...legs[legId],
      ...patch.legs[legId],
    };
  }

  return {
    ...base,
    mode: motionMode === "drive" ? "drive" : "stand",
    motionMode,
    driveCommand: {
      ...base.driveCommand,
      ...patch.driveCommand,
    },
    stance,
    servosReleased: false,
    legs,
  };
}

function syncDraftCommandFromState(currentDraft, patch) {
  const next = clone(currentDraft);
  for (const legId of LEG_IDS) {
    const leg = patch?.legs?.[legId];
    if (!leg) {
      continue;
    }

    if (leg.desired?.foot) {
      next[legId].foot = { ...leg.desired.foot };
    }
    if (leg.desired?.jointAnglesDeg) {
      next[legId].jointAnglesDeg = {
        ...next[legId].jointAnglesDeg,
        ...leg.desired.jointAnglesDeg,
      };
    }
    if (leg.desired?.servoAnglesDeg) {
      next[legId].servoAnglesDeg = {
        ...next[legId].servoAnglesDeg,
        ...leg.desired.servoAnglesDeg,
      };
    }
    if (leg.servoChannelMap) {
      next[legId].servoChannelMap = {
        ...next[legId].servoChannelMap,
        ...leg.servoChannelMap,
      };
    }
    if (leg.jointLimits) {
      next[legId].jointLimits = clone(leg.jointLimits);
    }
    if (leg.servoSpeedLimitDegPerSec) {
      next[legId].servoSpeedLimitDegPerSec = {
        ...next[legId].servoSpeedLimitDegPerSec,
        ...leg.servoSpeedLimitDegPerSec,
      };
    }
  }
  return next;
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
  const connected = getLinkConnected(robotState);
  const activeMode = getMotionMode(robotState);

  return (
    <section className="panel panel-header">
      <div className="panel-title-row">
        <div>
          <p className="eyebrow">Hybrid Dashboard</p>
          <h1>Argos Robot Dog Driver</h1>
          <p className="muted-copy">Main-style workbench on top of the current bridge and telemetry flow.</p>
        </div>
        <div className={`connection-pill ${connected ? "online" : "offline"}`}>
          {connected ? "ESP32 Feather Connected" : "ESP32 Feather Offline"}
        </div>
      </div>
      <div className="hero-meta">
        <span className="chip">Mode: {activeMode}</span>
        <span className="chip">Port: {robotState.connectedPort || "none"}</span>
        <span className="chip">Animation: {robotState.activeAnimation || "none"}</span>
      </div>
      <div className="toolbar toolbar-spread">
        <label className="inline-field">
          ESP32 Feather Port
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
          {connected ? (
            <button className="ghost-button" onClick={disconnect}>Disconnect</button>
          ) : (
            <button className="accent-button" onClick={connect}>Connect ESP32 Feather</button>
          )}
        </div>
      </div>
    </section>
  );
}

function WorkspaceToolbar({ workspace, setWorkspace, poseSource, setPoseSource }) {
  return (
    <section className="panel panel-toolbar">
      <div className="panel-title-row">
        <div>
          <p className="eyebrow">Workspaces</p>
          <h2>Operate, tune, and preview from one console</h2>
        </div>
        <div className="toolbar">
          <div className="toggle-group">
            <button className={workspace === "teleop" ? "toggle-active" : ""} onClick={() => setWorkspace("teleop")}>Teleop</button>
            <button className={workspace === "tuning" ? "toggle-active" : ""} onClick={() => setWorkspace("tuning")}>Leg Tuning</button>
            <button className={workspace === "animations" ? "toggle-active" : ""} onClick={() => setWorkspace("animations")}>Animations</button>
          </div>
          {workspace === "tuning" ? (
            <div className="toggle-group">
              <button className={poseSource === "desired" ? "toggle-active" : ""} onClick={() => setPoseSource("desired")}>Desired pose</button>
              <button className={poseSource === "current" ? "toggle-active" : ""} onClick={() => setPoseSource("current")}>Current pose</button>
            </div>
          ) : null}
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

function RobotOverview({ robotState, poseSource = "desired", selectedLegId, onSelectLeg }) {
  const overview = robotOverviewGeometry(robotState, poseSource);
  const width = 700;
  const height = 420;
  const center = { x: width / 2, y: height / 2 };
  const body = ROBOT_LAYOUT.body;
  const clickable = typeof onSelectLeg === "function";

  const toPoint = (point) => ({
    x: center.x + point.x,
    y: center.y + point.y,
  });

  return (
    <section className="panel panel-hero">
      <div className="panel-title-row">
        <h2>Robot Overview</h2>
        <span className="chip">{poseSource === "desired" ? "Desired pose" : "Estimated current pose"}</span>
      </div>
      <svg className="overview-svg" viewBox={`0 0 ${width} ${height}`}>
        <defs>
          <linearGradient id="body-fill" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#f7d56e" />
            <stop offset="100%" stopColor="#f58549" />
          </linearGradient>
        </defs>
        <rect x={center.x - body.width / 2} y={center.y - body.height / 2} width={body.width} height={body.height} rx="30" fill="url(#body-fill)" />
        {overview.map((leg) => {
          if (!leg.points || leg.points.length < 3) {
            return null;
          }

          const [hip, knee, foot] = leg.points.map(toPoint);
          const selected = leg.legId === selectedLegId;
          return (
            <g
              key={leg.legId}
              className={clickable ? "overview-leg overview-leg-clickable" : "overview-leg"}
              onClick={clickable ? () => onSelectLeg(leg.legId) : undefined}
            >
              <path d={segmentPath(hip, knee)} stroke={selected ? "#111111" : "#383838"} strokeWidth={selected ? "8" : "6"} fill="none" strokeLinecap="round" />
              <path d={segmentPath(knee, foot)} stroke={selected ? "#0078ff" : "#4aa6ff"} strokeWidth={selected ? "8" : "6"} fill="none" strokeLinecap="round" />
              <circle cx={foot.x} cy={foot.y} r={selected ? "9" : "7"} fill={selected ? "#0078ff" : "#4aa6ff"} />
              <text x={hip.x + 12} y={hip.y - 10} className="overview-label">{LEG_LABELS[leg.legId]}</text>
            </g>
          );
        })}
      </svg>
    </section>
  );
}

function LegPoseCard({ legId, title, pose, highlight = false, interactive = false, onLiveFootTarget }) {
  const svgRef = useRef(null);
  const dragStateRef = useRef({ active: false, pointerId: null });

  if (!pose?.geometry) {
    return (
      <section className={`panel ${highlight ? "panel-highlight" : ""}`}>
        <div className="panel-title-row">
          <h2>{title}</h2>
          <span className="chip">{LEG_LABELS[legId]}</span>
        </div>
        <p className="muted-copy">No geometry available yet.</p>
      </section>
    );
  }

  const geometry = pose.geometry;
  const p = (point) => toCanvasPoint(point, LEG_DRAWING);
  const points = {
    hip: p(geometry.hip),
    knee: p(geometry.knee),
    foot: p(geometry.foot),
    servoPivot: p(geometry.servoPivot),
    servoHornEnd: p(geometry.servoHornEnd),
    bellArmA: p(geometry.bellArmA),
    bellArmB: p(geometry.bellArmB),
    calfAttach: p(geometry.calfAttach),
  };

  function pointerToRelativeFoot(event) {
    const svg = svgRef.current;
    if (!svg) {
      return null;
    }

    const rect = svg.getBoundingClientRect();
    const scaleX = LEG_DRAWING.stageWidth / rect.width;
    const scaleY = LEG_DRAWING.stageHeight / rect.height;
    const canvasPoint = {
      x: (event.clientX - rect.left) * scaleX,
      y: (event.clientY - rect.top) * scaleY,
    };
    const worldPoint = fromCanvasPoint(canvasPoint);
    return toRelativeFoot(worldPoint);
  }

  function startDrag(event) {
    if (!interactive || typeof onLiveFootTarget !== "function") {
      return;
    }

    dragStateRef.current = { active: true, pointerId: event.pointerId };
    svgRef.current?.setPointerCapture(event.pointerId);
    const nextFoot = pointerToRelativeFoot(event);
    if (nextFoot) {
      onLiveFootTarget(legId, nextFoot);
    }
  }

  function moveDrag(event) {
    if (!dragStateRef.current.active || !interactive || typeof onLiveFootTarget !== "function") {
      return;
    }

    const nextFoot = pointerToRelativeFoot(event);
    if (nextFoot) {
      onLiveFootTarget(legId, nextFoot);
    }
  }

  function endDrag(event) {
    if (dragStateRef.current.active && dragStateRef.current.pointerId === event.pointerId) {
      dragStateRef.current = { active: false, pointerId: null };
    }
  }

  return (
    <section className={`panel ${highlight ? "panel-highlight" : ""}`}>
      <div className="panel-title-row">
        <div>
          <h2>{title}</h2>
          <p className="muted-copy">{LEG_LABELS[legId]}</p>
        </div>
        <span className="chip">{pose.reachable ? "Reachable" : "Best effort preview"}</span>
      </div>
      <svg
        ref={svgRef}
        className={`leg-svg ${interactive ? "leg-svg-interactive" : ""}`}
        viewBox={`0 0 ${LEG_DRAWING.stageWidth} ${LEG_DRAWING.stageHeight}`}
        onPointerMove={moveDrag}
        onPointerUp={endDrag}
        onPointerCancel={endDrag}
        onPointerLeave={endDrag}
      >
        <path d={segmentPath(points.hip, points.knee)} stroke="#141414" strokeWidth="5" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.knee, points.foot)} stroke="#1d8cff" strokeWidth="5" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.hip, points.bellArmA)} stroke="#21a264" strokeWidth="4" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.hip, points.bellArmB)} stroke="#21a264" strokeWidth="4" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.servoPivot, points.servoHornEnd)} stroke="#ff5232" strokeWidth="4" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.servoHornEnd, points.bellArmA)} stroke="#ef8a17" strokeWidth="4" fill="none" strokeLinecap="round" />
        <path d={segmentPath(points.bellArmB, points.calfAttach)} stroke="#6f4cff" strokeWidth="4" fill="none" strokeLinecap="round" />
        {Object.entries(points).map(([key, point]) => {
          if (key === "foot") {
            return (
              <circle
                key={key}
                cx={point.x}
                cy={point.y}
                r={interactive ? 11 : 7}
                className={interactive ? "draggable-foot" : undefined}
                fill="#1d8cff"
                onPointerDown={startDrag}
              />
            );
          }

          return <circle key={key} cx={point.x} cy={point.y} r="5" fill="#121212" />;
        })}
      </svg>
      {interactive ? <p className="muted-copy">Drag the blue foot target to stream live XY commands for this leg.</p> : null}
      <div className="stats-grid">
        <div>
          <label>Foot</label>
          <strong>{formatPoint(pose.foot)}</strong>
        </div>
        <div>
          <label>Joints</label>
          <strong>{formatTriple(pose.jointAnglesDeg)}</strong>
        </div>
        <div>
          <label>Servos</label>
          <strong>{formatTriple(pose.servoAnglesDeg)}</strong>
        </div>
        <div>
          <label>Foot error</label>
          <strong>{numberValue(pose.footError).toFixed(1)}</strong>
        </div>
      </div>
    </section>
  );
}

function SingleLegSelector({ selectedLegId, setSelectedLegId }) {
  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Single-Leg Focus</h2>
        <span className="chip">{LEG_LABELS[selectedLegId]}</span>
      </div>
      <div className="leg-selector-grid">
        {LEG_IDS.map((legId) => (
          <button key={legId} className={legId === selectedLegId ? "selector-active" : "ghost-button"} onClick={() => setSelectedLegId(legId)}>
            {LEG_LABELS[legId]}
          </button>
        ))}
      </div>
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
        onChange={(event) => onChange(numberValue(event.target.value))}
      />
    </label>
  );
}

function TrimPanel({ legId, legState, applyServoTrim, resetServoTrim }) {
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

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Neutral Trim</h2>
        <span className="chip">{LEG_LABELS[legId]}</span>
      </div>
      <div className="toolbar">
        <TrimField label="Hip yaw" value={draftTrim.hipYaw} onChange={(value) => setDraftTrim((current) => ({ ...current, hipYaw: value }))} />
        <TrimField label="Thigh" value={draftTrim.thigh} onChange={(value) => setDraftTrim((current) => ({ ...current, thigh: value }))} />
        <TrimField label="Calf" value={draftTrim.calf} onChange={(value) => setDraftTrim((current) => ({ ...current, calf: value }))} />
      </div>
      <div className="toolbar">
        <button className="accent-button" onClick={() => applyServoTrim(legId, draftTrim)}>Apply trim</button>
        <button
          className="ghost-button"
          onClick={() => {
            setDraftTrim({ hipYaw: 0, thigh: 0, calf: 0 });
            resetServoTrim(legId);
          }}
        >
          Zero trim
        </button>
      </div>
    </section>
  );
}

function TelemetryPanel({ robotState, selectedLegId }) {
  const leg = robotState.legs[selectedLegId];
  const faults = robotState.faults?.length ? robotState.faults.join(" | ") : "No active faults.";

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Selected Leg Telemetry</h2>
        <span className="chip">{LEG_LABELS[selectedLegId]}</span>
      </div>
      <div className="telemetry-grid">
        <div>
          <label>Connection</label>
          <strong>{getLinkConnected(robotState) ? "connected" : "offline"}</strong>
        </div>
        <div>
          <label>Mode</label>
          <strong>{robotState.mode}</strong>
        </div>
        <div>
          <label>Motion mode</label>
          <strong>{getMotionMode(robotState)}</strong>
        </div>
        <div>
          <label>Last ack</label>
          <strong>{robotState.lastAck ?? "none"}</strong>
        </div>
        <div>
          <label>Active animation</label>
          <strong>{robotState.activeAnimation ?? "none"}</strong>
        </div>
        <div>
          <label>Leg status</label>
          <strong>{leg?.status ?? "idle"}</strong>
        </div>
      </div>
      <div className="telemetry-log">
        <div>
          <h3>Desired foot</h3>
          <p>{formatPoint(leg?.desired?.foot)}</p>
        </div>
        <div>
          <h3>Current foot</h3>
          <p>{formatPoint(leg?.current?.foot)}</p>
        </div>
        <div>
          <h3>Desired joints</h3>
          <p>{formatTriple(leg?.desired?.jointAnglesDeg)}</p>
        </div>
        <div>
          <h3>Current joints</h3>
          <p>{formatTriple(leg?.current?.jointAnglesDeg)}</p>
        </div>
        <div>
          <h3>Desired servos</h3>
          <p>{formatTriple(leg?.desired?.servoAnglesDeg)}</p>
        </div>
        <div>
          <h3>Current servos</h3>
          <p>{formatTriple(leg?.current?.servoAnglesDeg)}</p>
        </div>
        <div>
          <h3>PCA9685 map</h3>
          <p>{formatTriple(leg?.servoChannelMap, 0)}</p>
        </div>
        <div>
          <h3>Joint limits</h3>
          <p>
            hip {numberValue(leg?.jointLimits?.hipYawDeg?.min).toFixed(0)}..{numberValue(leg?.jointLimits?.hipYawDeg?.max).toFixed(0)} |
            thigh {numberValue(leg?.jointLimits?.thighDeg?.min).toFixed(0)}..{numberValue(leg?.jointLimits?.thighDeg?.max).toFixed(0)} |
            calf {numberValue(leg?.jointLimits?.calfDeg?.min).toFixed(0)}..{numberValue(leg?.jointLimits?.calfDeg?.max).toFixed(0)}
          </p>
        </div>
        <div>
          <h3>Servo speed limit</h3>
          <p>{formatTriple(leg?.servoSpeedLimitDegPerSec, 0)}</p>
        </div>
        <div>
          <h3>Faults</h3>
          <p>{faults}</p>
        </div>
        <div>
          <h3>Last error</h3>
          <p>{robotState.lastError || leg?.lastError || "No active errors."}</p>
        </div>
      </div>
    </section>
  );
}

function LegControlPanel({
  selectedLegId,
  draftCommand,
  setDraftCommand,
  robotState,
  sendFoot,
  sendJoint,
  sendServo,
  previewServo,
  streamServo,
  sendChannelMap,
  sendJointLimits,
  sendServoSpeedLimit,
}) {
  const legDraft = draftCommand[selectedLegId];
  const channelMap = robotState.legs[selectedLegId]?.servoChannelMap ?? DEFAULT_SERVO_CHANNEL_MAP[selectedLegId];
  const jointLimits = robotState.legs[selectedLegId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
  const servoSpeedLimit = robotState.legs[selectedLegId]?.servoSpeedLimitDegPerSec ?? DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC;

  function updateSection(section, key, value) {
    setDraftCommand((current) => ({
      ...current,
      [selectedLegId]: {
        ...current[selectedLegId],
        [section]: {
          ...current[selectedLegId][section],
          [key]: value,
        },
      },
    }));
  }

  function updateChannelMap(key, value) {
    updateSection("servoChannelMap", key, clampChannel(value));
  }

  function updateJointLimit(section, key, value) {
    setDraftCommand((current) => ({
      ...current,
      [selectedLegId]: {
        ...current[selectedLegId],
        jointLimits: {
          ...current[selectedLegId].jointLimits,
          [section]: {
            ...current[selectedLegId].jointLimits[section],
            [key]: numberValue(value),
          },
        },
      },
    }));
  }

  function updateServoSpeedLimit(key, value) {
    updateSection("servoSpeedLimitDegPerSec", key, clampServoSpeed(value, DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC[key]));
  }

  function updateServoDraft(key, value) {
    const nextServoAngles = {
      ...legDraft.servoAnglesDeg,
      [key]: clampServoAngle(value),
    };
    setDraftCommand((current) => ({
      ...current,
      [selectedLegId]: {
        ...current[selectedLegId],
        servoAnglesDeg: nextServoAngles,
      },
    }));
    return nextServoAngles;
  }

  function previewServoDraft(key, value) {
    const nextServoAngles = updateServoDraft(key, value);
    previewServo(selectedLegId, nextServoAngles);
  }

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Direct Control</h2>
        <span className="chip">{LEG_LABELS[selectedLegId]}</span>
      </div>
      <div className="control-grid">
        <section className="control-card">
          <h3>Foot XY</h3>
          <label>
            X
            <input type="number" value={legDraft.foot.x} onChange={(event) => updateSection("foot", "x", numberValue(event.target.value))} />
          </label>
          <label>
            Y
            <input type="number" value={legDraft.foot.y} onChange={(event) => updateSection("foot", "y", numberValue(event.target.value))} />
          </label>
          <button onClick={() => sendFoot(selectedLegId, legDraft.foot)}>Send foot target</button>
        </section>

        <section className="control-card">
          <h3>Joint Angles</h3>
          <label>
            Hip yaw deg
            <input type="number" value={legDraft.jointAnglesDeg.hipYaw} onChange={(event) => updateSection("jointAnglesDeg", "hipYaw", numberValue(event.target.value))} />
          </label>
          <label>
            Thigh deg
            <input type="number" value={legDraft.jointAnglesDeg.thigh} onChange={(event) => updateSection("jointAnglesDeg", "thigh", numberValue(event.target.value))} />
          </label>
          <label>
            Calf deg
            <input type="number" value={legDraft.jointAnglesDeg.calf} onChange={(event) => updateSection("jointAnglesDeg", "calf", numberValue(event.target.value))} />
          </label>
          <button onClick={() => sendJoint(selectedLegId, legDraft.jointAnglesDeg)}>Send joint target</button>
        </section>

        <section className="control-card">
          <h3>Servo Angles</h3>
          <label>
            Hip yaw servo
            <input type="number" min="0" max="180" value={legDraft.servoAnglesDeg.hipYaw} onChange={(event) => previewServoDraft("hipYaw", event.target.value)} />
          </label>
          <label>
            Thigh servo
            <input type="number" min="0" max="180" value={legDraft.servoAnglesDeg.thigh} onChange={(event) => previewServoDraft("thigh", event.target.value)} />
          </label>
          <label>
            Calf servo
            <input type="number" min="0" max="180" value={legDraft.servoAnglesDeg.calf} onChange={(event) => previewServoDraft("calf", event.target.value)} />
          </label>
          <div className="range-stack">
            <label>
              Hip yaw live slider
              <input type="range" min="0" max="180" step="1" value={legDraft.servoAnglesDeg.hipYaw} onChange={(event) => streamServo(selectedLegId, updateServoDraft("hipYaw", event.target.value))} />
            </label>
            <label>
              Thigh live slider
              <input type="range" min="0" max="180" step="1" value={legDraft.servoAnglesDeg.thigh} onChange={(event) => streamServo(selectedLegId, updateServoDraft("thigh", event.target.value))} />
            </label>
            <label>
              Calf live slider
              <input type="range" min="0" max="180" step="1" value={legDraft.servoAnglesDeg.calf} onChange={(event) => streamServo(selectedLegId, updateServoDraft("calf", event.target.value))} />
            </label>
          </div>
          <div className="map-readout">Live sliders stream raw `set_leg_servo_angles` commands for the focused leg.</div>
          <button onClick={() => sendServo(selectedLegId, legDraft.servoAnglesDeg)}>Send servo target</button>
        </section>

        <section className="control-card">
          <h3>PCA9685 Channel Map</h3>
          <label>
            Hip yaw channel
            <input type="number" min="0" max="15" value={legDraft.servoChannelMap.hipYaw} onChange={(event) => updateChannelMap("hipYaw", event.target.value)} />
          </label>
          <label>
            Thigh channel
            <input type="number" min="0" max="15" value={legDraft.servoChannelMap.thigh} onChange={(event) => updateChannelMap("thigh", event.target.value)} />
          </label>
          <label>
            Calf channel
            <input type="number" min="0" max="15" value={legDraft.servoChannelMap.calf} onChange={(event) => updateChannelMap("calf", event.target.value)} />
          </label>
          <div className="map-readout">Active map: {formatTriple(channelMap, 0)}</div>
          <button onClick={() => sendChannelMap(selectedLegId, legDraft.servoChannelMap)}>Update servo channels</button>
        </section>

        <section className="control-card">
          <h3>Joint Limits</h3>
          <label>
            Hip yaw min
            <input type="number" value={legDraft.jointLimits.hipYawDeg.min} onChange={(event) => updateJointLimit("hipYawDeg", "min", event.target.value)} />
          </label>
          <label>
            Hip yaw max
            <input type="number" value={legDraft.jointLimits.hipYawDeg.max} onChange={(event) => updateJointLimit("hipYawDeg", "max", event.target.value)} />
          </label>
          <label>
            Thigh min
            <input type="number" value={legDraft.jointLimits.thighDeg.min} onChange={(event) => updateJointLimit("thighDeg", "min", event.target.value)} />
          </label>
          <label>
            Thigh max
            <input type="number" value={legDraft.jointLimits.thighDeg.max} onChange={(event) => updateJointLimit("thighDeg", "max", event.target.value)} />
          </label>
          <label>
            Calf min
            <input type="number" value={legDraft.jointLimits.calfDeg.min} onChange={(event) => updateJointLimit("calfDeg", "min", event.target.value)} />
          </label>
          <label>
            Calf max
            <input type="number" value={legDraft.jointLimits.calfDeg.max} onChange={(event) => updateJointLimit("calfDeg", "max", event.target.value)} />
          </label>
          <div className="map-readout">
            hip {jointLimits.hipYawDeg.min}..{jointLimits.hipYawDeg.max} | thigh {jointLimits.thighDeg.min}..{jointLimits.thighDeg.max} | calf {jointLimits.calfDeg.min}..{jointLimits.calfDeg.max}
          </div>
          <button onClick={() => sendJointLimits(selectedLegId, legDraft.jointLimits)}>Update joint limits</button>
        </section>

        <section className="control-card">
          <h3>Servo Speed Limit</h3>
          <label>
            Hip yaw deg/sec
            <input type="number" min="1" step="1" value={legDraft.servoSpeedLimitDegPerSec.hipYaw} onChange={(event) => updateServoSpeedLimit("hipYaw", event.target.value)} />
          </label>
          <label>
            Thigh deg/sec
            <input type="number" min="1" step="1" value={legDraft.servoSpeedLimitDegPerSec.thigh} onChange={(event) => updateServoSpeedLimit("thigh", event.target.value)} />
          </label>
          <label>
            Calf deg/sec
            <input type="number" min="1" step="1" value={legDraft.servoSpeedLimitDegPerSec.calf} onChange={(event) => updateServoSpeedLimit("calf", event.target.value)} />
          </label>
          <div className="map-readout">Active speed limit: {formatTriple(servoSpeedLimit, 0)}</div>
          <button onClick={() => sendServoSpeedLimit(selectedLegId, legDraft.servoSpeedLimitDegPerSec)}>Update speed limit</button>
        </section>
      </div>
    </section>
  );
}

function StatusPanel({ robotState }) {
  const connected = getLinkConnected(robotState);
  const faultCopy = robotState.faults?.length ? robotState.faults.join(" | ") : "No active faults.";

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Robot Status</h2>
        <span className={`connection-pill ${connected ? "online" : "offline"}`}>{robotState.connectedPort || "No serial link"}</span>
      </div>
      <div className="status-grid">
        <CardStat label="Mode" value={robotState.mode} accent="mode" />
        <CardStat label="Motion mode" value={getMotionMode(robotState)} accent="mode" />
        <CardStat label="ESP32 Feather link" value={connected ? "online" : "offline"} accent={connected ? "good" : "bad"} />
        <CardStat label="Forward" value={formatAxis(robotState.driveCommand?.vx)} />
        <CardStat label="Strafe" value={formatAxis(robotState.driveCommand?.vy)} />
        <CardStat label="Rotate" value={formatAxis(robotState.driveCommand?.yawRate)} />
        <CardStat label="Stance height" value={formatStanceHeight(robotState.stance?.height)} />
        <CardStat label="Active animation" value={robotState.activeAnimation || "none"} />
        <CardStat label="Last ack" value={robotState.lastAck || "none"} />
      </div>
      <div className="fault-box">
        <label>Faults</label>
        <p>{faultCopy}</p>
      </div>
      <div className="fault-box">
        <label>Last error</label>
        <p>{robotState.lastError || "No active errors."}</p>
      </div>
    </section>
  );
}

function StanceHeightControl({ value, setValue }) {
  const heightLabel = value > 0 ? "Taller" : value < 0 ? "Lower" : "Neutral";

  return (
    <section className="stance-control">
      <div className="stance-control-header">
        <div>
          <p className="eyebrow">Stance</p>
          <h3>Height</h3>
        </div>
        <strong>{heightLabel} {formatStanceHeight(value)}</strong>
      </div>
      <input
        aria-label="Stance height"
        type="range"
        min={STANCE_HEIGHT_RANGE_MM.min}
        max={STANCE_HEIGHT_RANGE_MM.max}
        step={STANCE_HEIGHT_RANGE_MM.step}
        value={value}
        onChange={(event) => setValue(numberValue(event.target.value, DEFAULT_STANCE.height))}
      />
      <div className="stance-scale">
        <span>Lower</span>
        <span>Neutral</span>
        <span>Taller</span>
      </div>
    </section>
  );
}

function TeleopWorkspace({ robotState, selectedLegId, setSelectedLegId, setAction, setMotionMode, stopMotion, stanceHeightMm, setStanceHeightMm }) {
  return (
    <>
      <section className="teleop-grid">
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
            <button className="accent-button" onClick={() => setMotionMode("stand")}>Stand</button>
            <button className="accent-button accent-button-secondary" onClick={() => setMotionMode("calibration")}>Calibration</button>
            <button className="ghost-button" onClick={() => setMotionMode("idle")}>Idle</button>
            <button className="ghost-button" onClick={stopMotion}>Stop</button>
          </div>
          <StanceHeightControl value={stanceHeightMm} setValue={setStanceHeightMm} />
        </section>
        <StatusPanel robotState={robotState} />
      </section>
      <section className="single-focus-grid">
        <RobotOverview robotState={robotState} poseSource="desired" selectedLegId={selectedLegId} onSelectLeg={setSelectedLegId} />
        <LegPoseCard legId={selectedLegId} title={`${LEG_LABELS[selectedLegId]} target`} pose={robotState.legs[selectedLegId]?.desired} highlight />
      </section>
    </>
  );
}

function LegTuningWorkspace({
  robotState,
  poseSource,
  selectedLegId,
  setSelectedLegId,
  draftCommand,
  setDraftCommand,
  queueLiveFootTarget,
  sendFoot,
  sendJoint,
  sendServo,
  previewServo,
  queueLiveServoTarget,
  sendChannelMap,
  sendJointLimits,
  sendServoSpeedLimit,
  applyServoTrim,
  resetServoTrim,
}) {
  return (
    <>
      <SingleLegSelector selectedLegId={selectedLegId} setSelectedLegId={setSelectedLegId} />
      <section className="single-focus-grid">
        <RobotOverview robotState={robotState} poseSource={poseSource} selectedLegId={selectedLegId} onSelectLeg={setSelectedLegId} />
        <LegPoseCard
          legId={selectedLegId}
          title={`${LEG_LABELS[selectedLegId]} ${poseSource}`}
          pose={robotState.legs[selectedLegId]?.[poseSource]}
          highlight
          interactive={poseSource === "desired"}
          onLiveFootTarget={queueLiveFootTarget}
        />
      </section>
      <LegControlPanel
        selectedLegId={selectedLegId}
        draftCommand={draftCommand}
        setDraftCommand={setDraftCommand}
        robotState={robotState}
        sendFoot={sendFoot}
        sendJoint={sendJoint}
        sendServo={sendServo}
        previewServo={previewServo}
        streamServo={queueLiveServoTarget}
        sendChannelMap={sendChannelMap}
        sendJointLimits={sendJointLimits}
        sendServoSpeedLimit={sendServoSpeedLimit}
      />
      <section className="support-grid">
        <TrimPanel
          legId={selectedLegId}
          legState={robotState.legs[selectedLegId]}
          applyServoTrim={applyServoTrim}
          resetServoTrim={resetServoTrim}
        />
        <TelemetryPanel robotState={robotState} selectedLegId={selectedLegId} />
      </section>
    </>
  );
}

function AnimationPanel({
  clip,
  setClip,
  previewTime,
  setPreviewTime,
  previewEnabled,
  setPreviewEnabled,
  selectedLegId,
  importPlacement,
  setImportPlacement,
  uploadClip,
  playClip,
  stopClip,
}) {
  const fileInputRef = useRef(null);
  const preview = useMemo(() => interpolateFullBodyClip(clip, previewTime), [clip, previewTime]);

  async function loadFile(event) {
    const file = event.target.files?.[0];
    if (!file) {
      return;
    }

    const text = await file.text();
    const parsed = JSON.parse(text);
    const converted =
      parsed.version === 2 && parsed.tracks
        ? validateClip(parsed)
        : convertLegacyLegClip(
            parsed,
            importPlacement.mode === "selected_leg"
              ? { mode: "selected_leg", legId: selectedLegId }
              : { mode: "mirrored_pair", pair: importPlacement.pair },
          );

    setClip(converted);
    setPreviewTime(0);
    event.target.value = "";
  }

  return (
    <section className="panel">
      <div className="panel-title-row">
        <h2>Animation</h2>
        <span className="chip">{clip.name}</span>
      </div>
      <div className="control-grid">
        <section className="control-card">
          <h3>Clip</h3>
          <label>
            Name
            <input value={clip.name} onChange={(event) => setClip((current) => ({ ...current, name: event.target.value }))} />
          </label>
          <label>
            Duration
            <input
              type="number"
              step="0.1"
              min="0.1"
              value={clip.duration}
              onChange={(event) => setClip((current) => validateClip({ ...current, duration: numberValue(event.target.value, current.duration) }))}
            />
          </label>
          <div className="toolbar">
            <button onClick={() => fileInputRef.current?.click()}>Load clip</button>
            <button onClick={() => downloadJson(`${clip.name || "robot-dog-clip"}.json`, clip)}>Save clip</button>
          </div>
          <input ref={fileInputRef} type="file" accept="application/json" className="hidden-input" onChange={loadFile} />
        </section>

        <section className="control-card">
          <h3>Legacy import mapping</h3>
          <label>
            Placement
            <select value={importPlacement.mode} onChange={(event) => setImportPlacement((current) => ({ ...current, mode: event.target.value }))}>
              <option value="selected_leg">Selected leg</option>
              <option value="mirrored_pair">Mirrored pair</option>
            </select>
          </label>
          {importPlacement.mode === "mirrored_pair" ? (
            <label>
              Pair
              <select value={importPlacement.pair} onChange={(event) => setImportPlacement((current) => ({ ...current, pair: event.target.value }))}>
                <option value="front">Front pair</option>
                <option value="rear">Rear pair</option>
                <option value="left">Left pair</option>
                <option value="right">Right pair</option>
              </select>
            </label>
          ) : (
            <p className="muted-copy">Legacy clips will map onto {LEG_LABELS[selectedLegId]}.</p>
          )}
        </section>

        <section className="control-card">
          <h3>Playback</h3>
          <label>
            Preview time
            <input type="range" min="0" max={clip.duration} step="0.01" value={previewTime} onChange={(event) => setPreviewTime(numberValue(event.target.value))} />
          </label>
          <div className="preview-readout">
            {LEG_IDS.map((legId) => (
              <div key={legId}>
                <strong>{LEG_LABELS[legId]}</strong>
                <span>{preview[legId].x.toFixed(1)}, {preview[legId].y.toFixed(1)}</span>
              </div>
            ))}
          </div>
          <div className="toolbar">
            <button onClick={uploadClip}>Upload to firmware</button>
            <button onClick={() => setPreviewEnabled((current) => !current)}>{previewEnabled ? "Use live desired view" : "Preview clip in viewport"}</button>
            <button className="accent-button" onClick={playClip}>Play on robot</button>
            <button className="ghost-button" onClick={stopClip}>Stop</button>
          </div>
        </section>
      </div>
    </section>
  );
}

function AnimationWorkspace({
  robotState,
  previewRobotState,
  clip,
  setClip,
  previewTime,
  setPreviewTime,
  previewEnabled,
  setPreviewEnabled,
  selectedLegId,
  setSelectedLegId,
  importPlacement,
  setImportPlacement,
  uploadClip,
  playClip,
  stopClip,
}) {
  return (
    <>
      <SingleLegSelector selectedLegId={selectedLegId} setSelectedLegId={setSelectedLegId} />
      <section className="single-focus-grid">
        <RobotOverview robotState={previewRobotState} poseSource="desired" selectedLegId={selectedLegId} onSelectLeg={setSelectedLegId} />
        <LegPoseCard
          legId={selectedLegId}
          title={previewEnabled ? `${LEG_LABELS[selectedLegId]} preview` : `${LEG_LABELS[selectedLegId]} desired`}
          pose={previewRobotState.legs[selectedLegId]?.desired}
          highlight
        />
      </section>
      <AnimationPanel
        clip={clip}
        setClip={setClip}
        previewTime={previewTime}
        setPreviewTime={setPreviewTime}
        previewEnabled={previewEnabled}
        setPreviewEnabled={setPreviewEnabled}
        selectedLegId={selectedLegId}
        importPlacement={importPlacement}
        setImportPlacement={setImportPlacement}
        uploadClip={uploadClip}
        playClip={playClip}
        stopClip={stopClip}
      />
      <StatusPanel robotState={robotState} />
    </>
  );
}

export default function App() {
  const [robotState, setRobotState] = useState(createLocalState);
  const [draftCommand, setDraftCommand] = useState(createDraftCommandState);
  const [workspace, setWorkspace] = useState("teleop");
  const [poseSource, setPoseSource] = useState("desired");
  const [selectedLegId, setSelectedLegId] = useState("front_left");
  const [selectedPort, setSelectedPort] = useState("");
  const [clip, setClip] = useState(DEFAULT_FULL_BODY_CLIP);
  const [previewTime, setPreviewTime] = useState(0);
  const [previewEnabled, setPreviewEnabled] = useState(false);
  const [importPlacement, setImportPlacement] = useState({ mode: "selected_leg", pair: "front" });
  const [stanceHeightMm, setStanceHeightMm] = useState(DEFAULT_STANCE.height);
  const [activeInputs, setActiveInputs] = useState(() => Object.fromEntries(INPUT_ACTIONS.map((action) => [action, false])));

  const telemetrySocketRef = useRef(null);
  const lastDriveSignatureRef = useRef("");
  const didInitializeStanceRef = useRef(false);
  const stanceSendTimerRef = useRef(null);
  const desiredPoseRef = useRef({});
  const jointLimitsRef = useRef({});
  const liveFootQueuedRef = useRef(null);
  const liveFootTimerRef = useRef(null);
  const lastLiveFootSentAtRef = useRef(0);
  const liveServoQueuedRef = useRef(null);
  const liveServoTimerRef = useRef(null);
  const lastLiveServoSentAtRef = useRef(0);

  const driveCommand = useMemo(() => deriveDriveCommand(activeInputs), [activeInputs]);
  const stance = useMemo(() => ({ ...DEFAULT_STANCE, height: stanceHeightMm }), [stanceHeightMm]);
  const hasMotion = driveCommand.vx !== 0 || driveCommand.vy !== 0 || driveCommand.yawRate !== 0;

  useEffect(() => {
    const desiredByLeg = {};
    const limitsByLeg = {};
    for (const legId of LEG_IDS) {
      desiredByLeg[legId] = robotState.legs[legId]?.desired;
      limitsByLeg[legId] = robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    }
    desiredPoseRef.current = desiredByLeg;
    jointLimitsRef.current = limitsByLeg;
  }, [robotState]);

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

  async function sendCommand(command) {
    const socket = telemetrySocketRef.current;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: "command", command }));
      return;
    }

    await postJson("/api/command", { command });
  }

  function clearLiveFootTimer() {
    if (liveFootTimerRef.current) {
      window.clearTimeout(liveFootTimerRef.current);
      liveFootTimerRef.current = null;
    }
  }

  function clearLiveServoTimer() {
    if (liveServoTimerRef.current) {
      window.clearTimeout(liveServoTimerRef.current);
      liveServoTimerRef.current = null;
    }
  }

  function clearStanceSendTimer() {
    if (stanceSendTimerRef.current) {
      window.clearTimeout(stanceSendTimerRef.current);
      stanceSendTimerRef.current = null;
    }
  }

  function clearLiveQueues() {
    clearLiveFootTimer();
    clearLiveServoTimer();
    liveFootQueuedRef.current = null;
    liveServoQueuedRef.current = null;
  }

  function setAction(action, active) {
    setActiveInputs((current) => ({ ...current, [action]: active }));
  }

  function clearActions() {
    setActiveInputs(Object.fromEntries(INPUT_ACTIONS.map((action) => [action, false])));
  }

  function applyDesiredFootLocally(legId, foot) {
    const currentDesired = desiredPoseRef.current[legId] ?? robotState.legs[legId]?.desired;
    const jointLimits = jointLimitsRef.current[legId] ?? robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    const pose = buildLegPoseFromFoot(foot, calibration, {
      startThetaThigh: currentDesired?.geometry?.thetaThigh,
      startThetaServo: currentDesired?.geometry?.thetaServo,
      jointLimits,
      hipYawDeg: currentDesired?.jointAnglesDeg?.hipYaw ?? 0,
    });
    const nextPose = pose.reachable ? pose : currentDesired ?? pose;

    desiredPoseRef.current[legId] = nextPose;
    setRobotState((current) => ({
      ...current,
      mode: "direct_foot_xy",
      servosReleased: false,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          desired: nextPose,
        },
      },
    }));
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        foot: { ...nextPose.foot },
        jointAnglesDeg: { ...nextPose.jointAnglesDeg },
        servoAnglesDeg: { ...nextPose.servoAnglesDeg },
      },
    }));

    return nextPose;
  }

  function applyDesiredJointLocally(legId, jointAnglesDeg) {
    const currentDesired = desiredPoseRef.current[legId] ?? robotState.legs[legId]?.desired;
    const jointLimits = jointLimitsRef.current[legId] ?? robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    const pose = buildLegPoseFromJointAngles(
      {
        hipYaw: numberValue(jointAnglesDeg.hipYaw, currentDesired?.jointAnglesDeg?.hipYaw ?? 0),
        thigh: numberValue(jointAnglesDeg.thigh, currentDesired?.jointAnglesDeg?.thigh ?? DEFAULT_LEG_COMMAND.jointAnglesDeg.thigh),
        calf: numberValue(jointAnglesDeg.calf, currentDesired?.jointAnglesDeg?.calf ?? DEFAULT_LEG_COMMAND.jointAnglesDeg.calf),
      },
      calibration,
      {
        startThetaServo: currentDesired?.geometry?.thetaServo,
        jointLimits,
      },
    );

    desiredPoseRef.current[legId] = pose;
    setRobotState((current) => ({
      ...current,
      mode: "direct_joint_angles",
      servosReleased: false,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          desired: pose,
        },
      },
    }));
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        foot: { ...pose.foot },
        jointAnglesDeg: { ...pose.jointAnglesDeg },
        servoAnglesDeg: { ...pose.servoAnglesDeg },
      },
    }));

    return pose;
  }

  function applyDesiredServoLocally(legId, servoAnglesDeg) {
    const currentDesired = desiredPoseRef.current[legId] ?? robotState.legs[legId]?.desired;
    const jointLimits = jointLimitsRef.current[legId] ?? robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    const nextServoAngles = {
      hipYaw: clampServoAngle(servoAnglesDeg.hipYaw ?? currentDesired?.servoAnglesDeg?.hipYaw ?? 90),
      thigh: clampServoAngle(servoAnglesDeg.thigh ?? currentDesired?.servoAnglesDeg?.thigh ?? 90),
      calf: clampServoAngle(servoAnglesDeg.calf ?? currentDesired?.servoAnglesDeg?.calf ?? 90),
    };
    const pose = buildLegPoseFromServoAngles(nextServoAngles, calibration, { jointLimits });

    desiredPoseRef.current[legId] = pose;
    setRobotState((current) => ({
      ...current,
      mode: "direct_servo_angles",
      servosReleased: false,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          desired: pose,
        },
      },
    }));
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        foot: { ...pose.foot },
        jointAnglesDeg: { ...pose.jointAnglesDeg },
        servoAnglesDeg: { ...nextServoAngles },
      },
    }));

    return nextServoAngles;
  }

  function buildServoCommandFromPose(legId, pose) {
    return {
      type: "set_leg_servo_angles",
      legId,
      hipYawServoDeg: pose.servoAnglesDeg.hipYaw,
      thighServoDeg: pose.servoAnglesDeg.thigh,
      calfServoDeg: pose.servoAnglesDeg.calf,
    };
  }

  async function flushQueuedFootCommand() {
    if (!liveFootQueuedRef.current) {
      return;
    }

    const queued = liveFootQueuedRef.current;
    liveFootQueuedRef.current = null;
    lastLiveFootSentAtRef.current = performance.now();

    try {
      await sendCommand(buildServoCommandFromPose(queued.legId, queued.pose));
    } catch (error) {
      setRobotState((current) => ({ ...current, lastError: error.message }));
    }

    if (liveFootQueuedRef.current) {
      scheduleQueuedFootCommand();
    }
  }

  function scheduleQueuedFootCommand() {
    if (!liveFootQueuedRef.current) {
      return;
    }

    const elapsed = performance.now() - lastLiveFootSentAtRef.current;
    if (elapsed >= LIVE_COMMAND_INTERVAL_MS) {
      clearLiveFootTimer();
      flushQueuedFootCommand();
      return;
    }

    if (liveFootTimerRef.current) {
      return;
    }

    liveFootTimerRef.current = window.setTimeout(() => {
      liveFootTimerRef.current = null;
      flushQueuedFootCommand();
    }, Math.max(0, LIVE_COMMAND_INTERVAL_MS - elapsed));
  }

  async function flushQueuedServoCommand() {
    if (!liveServoQueuedRef.current) {
      return;
    }

    const queued = liveServoQueuedRef.current;
    liveServoQueuedRef.current = null;
    lastLiveServoSentAtRef.current = performance.now();

    try {
      await sendCommand({
        type: "set_leg_servo_angles",
        legId: queued.legId,
        hipYawServoDeg: queued.servoAnglesDeg.hipYaw,
        thighServoDeg: queued.servoAnglesDeg.thigh,
        calfServoDeg: queued.servoAnglesDeg.calf,
      });
    } catch (error) {
      setRobotState((current) => ({ ...current, lastError: error.message }));
    }

    if (liveServoQueuedRef.current) {
      scheduleQueuedServoCommand();
    }
  }

  function scheduleQueuedServoCommand() {
    if (!liveServoQueuedRef.current) {
      return;
    }

    const elapsed = performance.now() - lastLiveServoSentAtRef.current;
    if (elapsed >= LIVE_COMMAND_INTERVAL_MS) {
      clearLiveServoTimer();
      flushQueuedServoCommand();
      return;
    }

    if (liveServoTimerRef.current) {
      return;
    }

    liveServoTimerRef.current = window.setTimeout(() => {
      liveServoTimerRef.current = null;
      flushQueuedServoCommand();
    }, Math.max(0, LIVE_COMMAND_INTERVAL_MS - elapsed));
  }

  function queueLiveFootTarget(legId, foot) {
    const pose = applyDesiredFootLocally(legId, foot);
    liveFootQueuedRef.current = { legId, pose };
    scheduleQueuedFootCommand();
  }

  function queueLiveServoTarget(legId, servoAnglesDeg) {
    const nextServoAngles = applyDesiredServoLocally(legId, servoAnglesDeg);
    liveServoQueuedRef.current = { legId, servoAnglesDeg: nextServoAngles };
    scheduleQueuedServoCommand();
  }

  async function fetchStatus() {
    const response = await fetch(`${getBackendHttpBaseUrl()}/api/status`);
    const payload = await response.json();
    setRobotState((current) => mergeState(current, payload));
    setDraftCommand((current) => syncDraftCommandFromState(current, payload));
    if (Number.isFinite(payload.stance?.height)) {
      setStanceHeightMm(payload.stance.height);
    }
    setSelectedPort((current) => current || payload.connectedPort || "");
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
          setDraftCommand((current) => syncDraftCommandFromState(current, message.payload));
          if (Number.isFinite(message.payload?.stance?.height)) {
            setStanceHeightMm(message.payload.stance.height);
          }
          setSelectedPort((current) => current || message.payload?.connectedPort || "");
        }
        if (message.type === "ack") {
          setRobotState((current) => ({ ...current, lastAck: message.message ?? message.seq ?? "ack" }));
        }
        if (message.type === "error") {
          setRobotState((current) => ({ ...current, lastError: message.message || "Unknown controller error" }));
        }
        if (message.type === "animation_progress") {
          setRobotState((current) => ({ ...current, activeAnimation: message.name || current.activeAnimation }));
        }
      } catch (error) {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      }
    };

    return () => {
      telemetrySocketRef.current = null;
      clearLiveQueues();
      clearStanceSendTimer();
      socket.close();
    };
  }, []);

  useEffect(() => {
    function handleKeyDown(event) {
      const action = KEY_TO_ACTION[event.code];
      if (!action || event.repeat) {
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
    if (!didInitializeStanceRef.current) {
      didInitializeStanceRef.current = true;
      return undefined;
    }

    setRobotState((current) => applyMotionPatchToState(current, {
      motionMode: hasMotion ? "drive" : "stand",
      driveCommand: hasMotion ? driveCommand : DEFAULT_DRIVE_COMMAND,
      stance,
      timeMs: performance.now(),
    }));

    clearStanceSendTimer();
    stanceSendTimerRef.current = window.setTimeout(() => {
      stanceSendTimerRef.current = null;
      sendCommand({ type: "set_stance", stance }).catch((error) => {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      });
    }, LIVE_COMMAND_INTERVAL_MS);

    return clearStanceSendTimer;
  }, [stanceHeightMm]);

  useEffect(() => {
    const signature = JSON.stringify(driveCommand);
    if (signature === lastDriveSignatureRef.current) {
      return;
    }

    lastDriveSignatureRef.current = signature;
    const command = hasMotion ? { type: "set_drive_command", drive: driveCommand, stance } : { type: "stop_motion" };
    sendCommand(command).catch((error) => {
      setRobotState((current) => ({ ...current, lastError: error.message }));
    });
  }, [driveCommand, hasMotion]);

  useEffect(() => {
    if (!hasMotion) {
      return undefined;
    }

    const intervalId = window.setInterval(() => {
      sendCommand({ type: "set_drive_command", drive: deriveDriveCommand(activeInputs), stance }).catch((error) => {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      });
    }, 100);

    return () => window.clearInterval(intervalId);
  }, [activeInputs, hasMotion, stanceHeightMm]);

  async function connect() {
    if (!selectedPort) {
      throw new Error("Select a serial port before connecting.");
    }

    await postJson("/api/connect", { path: selectedPort, baudRate: 921600 });
    await fetchStatus();
  }

  async function disconnect() {
    clearLiveQueues();
    await postJson("/api/disconnect", {});
    await fetchStatus();
  }

  async function sendFoot(legId, foot) {
    const pose = applyDesiredFootLocally(legId, foot);
    await sendCommand(buildServoCommandFromPose(legId, pose));
  }

  async function sendJoint(legId, jointAnglesDeg) {
    const pose = applyDesiredJointLocally(legId, jointAnglesDeg);
    await sendCommand(buildServoCommandFromPose(legId, pose));
  }

  async function sendServo(legId, servoAnglesDeg) {
    const nextServoAngles = applyDesiredServoLocally(legId, servoAnglesDeg);
    await sendCommand({
      type: "set_leg_servo_angles",
      legId,
      hipYawServoDeg: nextServoAngles.hipYaw,
      thighServoDeg: nextServoAngles.thigh,
      calfServoDeg: nextServoAngles.calf,
    });
  }

  async function sendChannelMap(legId, servoChannelMap) {
    const nextMap = {
      hipYaw: clampChannel(servoChannelMap.hipYaw),
      thigh: clampChannel(servoChannelMap.thigh),
      calf: clampChannel(servoChannelMap.calf),
    };

    setRobotState((current) => ({
      ...current,
      servosReleased: false,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          servoChannelMap: nextMap,
        },
      },
    }));
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        servoChannelMap: nextMap,
      },
    }));

    await sendCommand({
      type: "set_leg_servo_channel_map",
      legId,
      hipYawChannel: nextMap.hipYaw,
      thighChannel: nextMap.thigh,
      calfChannel: nextMap.calf,
    });
  }

  async function sendJointLimits(legId, jointLimits) {
    const normalized = normalizeJointLimits(jointLimits);
    const currentDesired = desiredPoseRef.current[legId] ?? robotState.legs[legId]?.desired;
    const nextDesired = buildLegPoseFromFoot(currentDesired?.foot ?? DEFAULT_LEG_COMMAND.foot, calibration, {
      startThetaThigh: currentDesired?.geometry?.thetaThigh,
      startThetaServo: currentDesired?.geometry?.thetaServo,
      jointLimits: normalized,
      hipYawDeg: currentDesired?.jointAnglesDeg?.hipYaw ?? 0,
    });

    jointLimitsRef.current[legId] = normalized;
    desiredPoseRef.current[legId] = nextDesired;
    setRobotState((current) => ({
      ...current,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          jointLimits: normalized,
          desired: nextDesired,
        },
      },
    }));
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        jointLimits: clone(normalized),
        foot: { ...nextDesired.foot },
        jointAnglesDeg: { ...nextDesired.jointAnglesDeg },
        servoAnglesDeg: { ...nextDesired.servoAnglesDeg },
      },
    }));

    await sendCommand({
      type: "set_leg_joint_limits",
      legId,
      hipYawMinDeg: normalized.hipYawDeg.min,
      hipYawMaxDeg: normalized.hipYawDeg.max,
      thighMinDeg: normalized.thighDeg.min,
      thighMaxDeg: normalized.thighDeg.max,
      calfMinDeg: normalized.calfDeg.min,
      calfMaxDeg: normalized.calfDeg.max,
    });
  }

  async function sendServoSpeedLimit(legId, servoSpeedLimitDegPerSec) {
    const normalized = {
      hipYaw: clampServoSpeed(servoSpeedLimitDegPerSec.hipYaw, DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC.hipYaw),
      thigh: clampServoSpeed(servoSpeedLimitDegPerSec.thigh, DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC.thigh),
      calf: clampServoSpeed(servoSpeedLimitDegPerSec.calf, DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC.calf),
    };

    setRobotState((current) => ({
      ...current,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          servoSpeedLimitDegPerSec: normalized,
        },
      },
    }));
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        servoSpeedLimitDegPerSec: normalized,
      },
    }));

    await sendCommand({
      type: "set_leg_servo_speed_limit",
      legId,
      hipYawDegPerSec: normalized.hipYaw,
      thighDegPerSec: normalized.thigh,
      calfDegPerSec: normalized.calf,
    });
  }

  async function applyServoTrim(legId, trim) {
    const nextTrim = {
      hipYaw: numberValue(trim.hipYaw, DEFAULT_SERVO_TRIM_DEG.hipYaw),
      thigh: numberValue(trim.thigh, DEFAULT_SERVO_TRIM_DEG.thigh),
      calf: numberValue(trim.calf, DEFAULT_SERVO_TRIM_DEG.calf),
    };

    setRobotState((current) => ({
      ...current,
      legs: {
        ...current.legs,
        [legId]: {
          ...current.legs[legId],
          servoTrimDeg: nextTrim,
        },
      },
    }));

    await sendCommand({
      type: "set_leg_servo_trim",
      legId,
      hipYawOffsetDeg: nextTrim.hipYaw,
      thighOffsetDeg: nextTrim.thigh,
      calfOffsetDeg: nextTrim.calf,
    });
  }

  async function resetServoTrim(legId) {
    await applyServoTrim(legId, DEFAULT_SERVO_TRIM_DEG);
  }

  async function setMotionMode(mode) {
    if (mode === "stand" || mode === "drive") {
      setRobotState((current) => applyMotionPatchToState(current, {
        motionMode: mode,
        driveCommand: mode === "drive" ? driveCommand : DEFAULT_DRIVE_COMMAND,
        stance,
        timeMs: performance.now(),
      }));
    } else {
      setRobotState((current) => ({
        ...current,
        motionMode: mode,
        mode: mode === "idle" ? "idle" : current.mode,
        servosReleased: false,
      }));
    }
    await sendCommand({ type: "set_motion_mode", mode, stance });
  }

  async function stopMotion() {
    clearActions();
    setRobotState((current) => ({
      ...current,
      motionMode: "idle",
      driveCommand: { ...DEFAULT_DRIVE_COMMAND },
    }));
    await sendCommand({ type: "stop_motion" });
  }

  async function panicRelease() {
    clearActions();
    clearLiveQueues();
    setRobotState((current) => ({
      ...current,
      servosReleased: true,
      motionMode: "idle",
      activeAnimation: null,
      driveCommand: { ...DEFAULT_DRIVE_COMMAND },
    }));
    await sendCommand({ type: "panic_release" });
  }

  async function uploadClip() {
    const validated = validateClip(clip);
    await postJson("/api/animations", { clip: validated, frames: createUploadFrames(validated) });
    setRobotState((current) => ({ ...current, activeAnimation: validated.name }));
  }

  async function playClip() {
    const validated = validateClip(clip);
    await postJson(`/api/animations/${encodeURIComponent(validated.name)}/play`, {});
    setRobotState((current) => ({
      ...current,
      mode: "animation_playback",
      activeAnimation: validated.name,
      servosReleased: false,
    }));
  }

  async function stopClip() {
    const validated = validateClip(clip);
    await postJson(`/api/animations/${encodeURIComponent(validated.name)}/stop`, {});
    setRobotState((current) => ({
      ...current,
      mode: "idle",
    }));
  }

  const previewRobotState = useMemo(() => {
    if (!previewEnabled) {
      return robotState;
    }

    const previewFrame = interpolateFullBodyClip(clip, previewTime);
    const next = clone(robotState);
    for (const legId of LEG_IDS) {
      next.legs[legId].desired = buildLegPoseFromFoot(previewFrame[legId], calibration, {
        startThetaThigh: next.legs[legId].desired?.geometry?.thetaThigh,
        startThetaServo: next.legs[legId].desired?.geometry?.thetaServo,
        jointLimits: next.legs[legId].jointLimits,
        hipYawDeg: next.legs[legId].desired?.jointAnglesDeg?.hipYaw ?? 0,
      });
    }
    return next;
  }, [clip, previewEnabled, previewTime, robotState]);

  function withError(handler) {
    return (...args) => handler(...args).catch((error) => {
      setRobotState((current) => ({ ...current, lastError: error.message }));
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

      <WorkspaceToolbar workspace={workspace} setWorkspace={setWorkspace} poseSource={poseSource} setPoseSource={setPoseSource} />

      {workspace === "teleop" ? (
        <TeleopWorkspace
          robotState={robotState}
          selectedLegId={selectedLegId}
          setSelectedLegId={setSelectedLegId}
          setAction={setAction}
          setMotionMode={withError(setMotionMode)}
          stopMotion={withError(stopMotion)}
          stanceHeightMm={stanceHeightMm}
          setStanceHeightMm={setStanceHeightMm}
        />
      ) : null}

      {workspace === "tuning" ? (
        <LegTuningWorkspace
          robotState={robotState}
          poseSource={poseSource}
          selectedLegId={selectedLegId}
          setSelectedLegId={setSelectedLegId}
          draftCommand={draftCommand}
          setDraftCommand={setDraftCommand}
          queueLiveFootTarget={queueLiveFootTarget}
          sendFoot={withError(sendFoot)}
          sendJoint={withError(sendJoint)}
          sendServo={withError(sendServo)}
          previewServo={applyDesiredServoLocally}
          queueLiveServoTarget={queueLiveServoTarget}
          sendChannelMap={withError(sendChannelMap)}
          sendJointLimits={withError(sendJointLimits)}
          sendServoSpeedLimit={withError(sendServoSpeedLimit)}
          applyServoTrim={withError(applyServoTrim)}
          resetServoTrim={withError(resetServoTrim)}
        />
      ) : null}

      {workspace === "animations" ? (
        <AnimationWorkspace
          robotState={robotState}
          previewRobotState={previewRobotState}
          clip={clip}
          setClip={setClip}
          previewTime={previewTime}
          setPreviewTime={setPreviewTime}
          previewEnabled={previewEnabled}
          setPreviewEnabled={setPreviewEnabled}
          selectedLegId={selectedLegId}
          setSelectedLegId={setSelectedLegId}
          importPlacement={importPlacement}
          setImportPlacement={setImportPlacement}
          uploadClip={withError(uploadClip)}
          playClip={withError(playClip)}
          stopClip={withError(stopClip)}
        />
      ) : null}
    </main>
  );
}
