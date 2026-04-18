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
import {
  DEFAULT_FULL_BODY_CLIP,
  DEFAULT_JOINT_LIMITS,
  DEFAULT_LEG_COMMAND,
  DEFAULT_SERVO_CHANNEL_MAP,
  LEG_DRAWING,
  LEG_GEOMETRY,
  LEG_IDS,
  LEG_LABELS,
  ROBOT_LAYOUT,
} from "../shared/robot-config.js";

const calibration = createNeutralCalibration();

function clone(value) {
  return JSON.parse(JSON.stringify(value));
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

function clampChannel(value) {
  const next = Number(value);
  if (!Number.isFinite(next)) {
    return 0;
  }
  return Math.max(0, Math.min(15, Math.round(next)));
}

function numberValue(value, fallback = 0) {
  const next = Number(value);
  return Number.isFinite(next) ? next : fallback;
}

function createLocalState() {
  const legs = {};
  for (const legId of LEG_IDS) {
    const desired = buildLegPoseFromFoot(DEFAULT_LEG_COMMAND.foot, calibration);
    legs[legId] = {
      desired,
      current: desired,
      status: "idle",
      lastError: null,
      servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
      jointLimits: clone(DEFAULT_JOINT_LIMITS),
    };
  }

  return {
    connected: false,
    mode: "idle",
    servosReleased: false,
    activeAnimation: null,
    lastAck: null,
    lastError: null,
    firmwareMs: 0,
    ports: [],
    legs,
  };
}

function mergeState(base, patch) {
  return {
    ...base,
    ...patch,
    legs: {
      ...base.legs,
      ...patch.legs,
    },
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

function LegDetail({ legId, legState, title, highlight = false, interactive = false, onLiveFootTarget }) {
  const geometry = legState?.geometry;
  const svgRef = useRef(null);
  const dragStateRef = useRef({ active: false, pointerId: null });

  if (!geometry) {
    return <div className="empty-card">No pose</div>;
  }

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
    if (!interactive) {
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
    if (!dragStateRef.current.active || !interactive) {
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
    <div className={`panel ${highlight ? "panel-highlight" : ""}`}>
      <div className="panel-title-row">
        <h3>{title}</h3>
        <span className="chip">
          foot {legState.foot.x.toFixed(1)}, {legState.foot.y.toFixed(1)}
        </span>
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
      {interactive ? <p className="muted-copy">Drag the blue foot target to stream live XY commands to the selected leg.</p> : null}
      <div className="stats-grid">
        <div>
          <label>Thigh joint</label>
          <strong>{legState.jointAnglesDeg.thigh.toFixed(1)} deg</strong>
        </div>
        <div>
          <label>Calf joint</label>
          <strong>{legState.jointAnglesDeg.calf.toFixed(1)} deg</strong>
        </div>
        <div>
          <label>Thigh servo</label>
          <strong>{legState.servoAnglesDeg.thigh.toFixed(1)} deg</strong>
        </div>
        <div>
          <label>Calf servo</label>
          <strong>{legState.servoAnglesDeg.calf.toFixed(1)} deg</strong>
        </div>
      </div>
    </div>
  );
}

function RobotOverview({ robotState, poseSource, selectedLegId, onSelectLeg }) {
  const overview = robotOverviewGeometry(robotState, poseSource);
  const width = 700;
  const height = 420;
  const center = { x: width / 2, y: height / 2 };
  const body = ROBOT_LAYOUT.body;

  const toPoint = (point) => ({
    x: center.x + point.x,
    y: center.y + point.y,
  });

  return (
    <div className="panel panel-hero">
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
          const [hip, knee, foot] = leg.points.map(toPoint);
          const selected = leg.legId === selectedLegId;
          return (
            <g key={leg.legId} onClick={() => onSelectLeg(leg.legId)} style={{ cursor: "pointer" }}>
              <path d={segmentPath(hip, knee)} stroke={selected ? "#111111" : "#383838"} strokeWidth={selected ? "8" : "6"} fill="none" strokeLinecap="round" />
              <path d={segmentPath(knee, foot)} stroke={selected ? "#0078ff" : "#4aa6ff"} strokeWidth={selected ? "8" : "6"} fill="none" strokeLinecap="round" />
              <circle cx={foot.x} cy={foot.y} r={selected ? "9" : "7"} fill={selected ? "#0078ff" : "#4aa6ff"} />
              <text x={hip.x + 12} y={hip.y - 10} className="overview-label">
                {LEG_LABELS[leg.legId]}
              </text>
            </g>
          );
        })}
      </svg>
    </div>
  );
}

function SingleLegSelector({ selectedLegId, setSelectedLegId }) {
  return (
    <div className="panel">
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
    </div>
  );
}

function ControlPanel({
  selectedLegId,
  draftCommand,
  setDraftCommand,
  robotState,
  sendFoot,
  sendJoint,
  sendServo,
  sendChannelMap,
  sendJointLimits,
  runBuiltin,
  stopMotion,
}) {
  const legDraft = draftCommand[selectedLegId];
  const channelMap = robotState.legs[selectedLegId]?.servoChannelMap ?? DEFAULT_SERVO_CHANNEL_MAP[selectedLegId];
  const jointLimits = robotState.legs[selectedLegId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;

  function updateSection(section, key, value) {
    setDraftCommand((current) => ({
      ...current,
      [selectedLegId]: {
        ...current[selectedLegId],
        [section]: {
          ...current[selectedLegId][section],
          [key]: numberValue(value),
        },
      },
    }));
  }

  function updateChannelMap(key, value) {
    setDraftCommand((current) => ({
      ...current,
      [selectedLegId]: {
        ...current[selectedLegId],
        servoChannelMap: {
          ...current[selectedLegId].servoChannelMap,
          [key]: clampChannel(value),
        },
      },
    }));
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

  return (
    <div className="panel">
      <div className="panel-title-row">
        <h2>Direct Control</h2>
        <span className="chip">{LEG_LABELS[selectedLegId]}</span>
      </div>

      <div className="control-grid">
        <section className="control-card">
          <h3>Foot XY</h3>
          <label>
            X
            <input type="number" value={legDraft.foot.x} onChange={(event) => updateSection("foot", "x", event.target.value)} />
          </label>
          <label>
            Y
            <input type="number" value={legDraft.foot.y} onChange={(event) => updateSection("foot", "y", event.target.value)} />
          </label>
          <button onClick={() => sendFoot(selectedLegId, legDraft.foot)}>Send foot target</button>
        </section>

        <section className="control-card">
          <h3>Joint Angles</h3>
          <label>
            Thigh deg
            <input type="number" value={legDraft.jointAnglesDeg.thigh} onChange={(event) => updateSection("jointAnglesDeg", "thigh", event.target.value)} />
          </label>
          <label>
            Calf deg
            <input type="number" value={legDraft.jointAnglesDeg.calf} onChange={(event) => updateSection("jointAnglesDeg", "calf", event.target.value)} />
          </label>
          <button onClick={() => sendJoint(selectedLegId, legDraft.jointAnglesDeg)}>Send joint target</button>
        </section>

        <section className="control-card">
          <h3>Servo Angles</h3>
          <label>
            Thigh servo
            <input type="number" value={legDraft.servoAnglesDeg.thigh} onChange={(event) => updateSection("servoAnglesDeg", "thigh", event.target.value)} />
          </label>
          <label>
            Calf servo
            <input type="number" value={legDraft.servoAnglesDeg.calf} onChange={(event) => updateSection("servoAnglesDeg", "calf", event.target.value)} />
          </label>
          <button onClick={() => sendServo(selectedLegId, legDraft.servoAnglesDeg)}>Send servo target</button>
        </section>

        <section className="control-card">
          <h3>PCA9685 Channel Map</h3>
          <label>
            Thigh channel
            <input type="number" min="0" max="15" value={legDraft.servoChannelMap.thigh} onChange={(event) => updateChannelMap("thigh", event.target.value)} />
          </label>
          <label>
            Calf channel
            <input type="number" min="0" max="15" value={legDraft.servoChannelMap.calf} onChange={(event) => updateChannelMap("calf", event.target.value)} />
          </label>
          <div className="map-readout">
            Active map: thigh {channelMap.thigh}, calf {channelMap.calf}
          </div>
          <button onClick={() => sendChannelMap(selectedLegId, legDraft.servoChannelMap)}>Update servo channels</button>
        </section>

        <section className="control-card">
          <h3>Joint Limits</h3>
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
            Active: thigh {jointLimits.thighDeg.min}..{jointLimits.thighDeg.max}, calf {jointLimits.calfDeg.min}..{jointLimits.calfDeg.max}
          </div>
          <button onClick={() => sendJointLimits(selectedLegId, legDraft.jointLimits)}>Update joint limits</button>
        </section>
      </div>

      <div className="toolbar">
        <button className="accent-button" onClick={() => runBuiltin("walk")}>
          Built-in walk
        </button>
        <button className="accent-button accent-button-secondary" onClick={() => runBuiltin("crouch")}>
          Built-in crouch
        </button>
        <button className="ghost-button" onClick={stopMotion}>
          Stop motion
        </button>
      </div>
    </div>
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
    <div className="panel">
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
                <span>
                  {preview[legId].x.toFixed(1)}, {preview[legId].y.toFixed(1)}
                </span>
              </div>
            ))}
          </div>
          <div className="toolbar">
            <button onClick={uploadClip}>Upload to firmware</button>
            <button onClick={() => setPreviewEnabled((current) => !current)}>{previewEnabled ? "Use live desired view" : "Preview clip in viewport"}</button>
            <button className="accent-button" onClick={playClip}>
              Play on robot
            </button>
            <button className="ghost-button" onClick={stopClip}>
              Stop
            </button>
          </div>
        </section>
      </div>
    </div>
  );
}

function ConnectionPanel({ robotState, selectedPort, setSelectedPort, connect, disconnect, refreshStatus }) {
  return (
    <div className="panel panel-header">
      <div className="panel-title-row">
        <div>
          <p className="eyebrow">Argos debug dashboard</p>
          <h1>Robot dog motion console</h1>
        </div>
        <div className={`connection-pill ${robotState.connected ? "online" : "offline"}`}>{robotState.connected ? "Connected" : "Disconnected"}</div>
      </div>
      <div className="toolbar toolbar-spread">
        <label className="inline-field">
          Port
          <select value={selectedPort} onChange={(event) => setSelectedPort(event.target.value)}>
            <option value="">Select serial port</option>
            {robotState.ports.map((port) => (
              <option key={port.path} value={port.path}>
                {port.path}
              </option>
            ))}
          </select>
        </label>
        <div className="toolbar">
          <button onClick={refreshStatus}>Refresh ports</button>
          {robotState.connected ? (
            <button className="ghost-button" onClick={disconnect}>
              Disconnect
            </button>
          ) : (
            <button className="accent-button" onClick={connect}>
              Connect
            </button>
          )}
        </div>
      </div>
    </div>
  );
}

function TelemetryPanel({ robotState, selectedLegId }) {
  const leg = robotState.legs[selectedLegId];
  return (
    <div className="panel">
      <div className="panel-title-row">
        <h2>Telemetry</h2>
        <span className="chip">{robotState.mode}</span>
      </div>
      <div className="telemetry-grid">
        <div>
          <label>Firmware clock</label>
          <strong>{robotState.firmwareMs} ms</strong>
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
          <label>Servo drive</label>
          <strong>{robotState.servosReleased ? "released" : "holding"}</strong>
        </div>
        <div>
          <label>Selected leg status</label>
          <strong>{leg?.status ?? "idle"}</strong>
        </div>
      </div>
      <div className="telemetry-log">
        <div>
          <h3>Desired servo</h3>
          <p>
            thigh {leg?.desired.servoAnglesDeg.thigh.toFixed(1)} deg, calf {leg?.desired.servoAnglesDeg.calf.toFixed(1)} deg
          </p>
        </div>
        <div>
          <h3>Current servo</h3>
          <p>
            thigh {leg?.current.servoAnglesDeg.thigh.toFixed(1)} deg, calf {leg?.current.servoAnglesDeg.calf.toFixed(1)} deg
          </p>
        </div>
        <div>
          <h3>PCA9685 map</h3>
          <p>
            thigh ch {leg?.servoChannelMap?.thigh ?? "-"}, calf ch {leg?.servoChannelMap?.calf ?? "-"}
          </p>
        </div>
        <div>
          <h3>Joint limits</h3>
          <p>
            thigh {leg?.jointLimits?.thighDeg?.min ?? "-"}..{leg?.jointLimits?.thighDeg?.max ?? "-"} deg, calf {leg?.jointLimits?.calfDeg?.min ?? "-"}..
            {leg?.jointLimits?.calfDeg?.max ?? "-"} deg
          </p>
        </div>
        <div>
          <h3>Errors</h3>
          <p>{robotState.lastError || leg?.lastError || "No active errors."}</p>
        </div>
      </div>
    </div>
  );
}

export default function App() {
  const [robotState, setRobotState] = useState(createLocalState);
  const [viewMode, setViewMode] = useState("full");
  const [poseSource, setPoseSource] = useState("desired");
  const [selectedLegId, setSelectedLegId] = useState("front_left");
  const [selectedPort, setSelectedPort] = useState("");
  const [clip, setClip] = useState(DEFAULT_FULL_BODY_CLIP);
  const [previewTime, setPreviewTime] = useState(0);
  const [previewEnabled, setPreviewEnabled] = useState(false);
  const [importPlacement, setImportPlacement] = useState({ mode: "selected_leg", pair: "front" });
  const [draftCommand, setDraftCommand] = useState(() =>
    Object.fromEntries(
      LEG_IDS.map((legId) => [
        legId,
        {
          ...clone(DEFAULT_LEG_COMMAND),
          servoChannelMap: { ...DEFAULT_SERVO_CHANNEL_MAP[legId] },
          jointLimits: clone(DEFAULT_JOINT_LIMITS),
        },
      ]),
    ),
  );
  const dragQueuedCommandRef = useRef(null);
  const dragSendInFlightRef = useRef(false);
  const desiredPoseRef = useRef({});
  const jointLimitsRef = useRef({});

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

  async function fetchStatus() {
    const response = await fetch("/api/status");
    const payload = await response.json();
    setRobotState((current) => mergeState(current, payload));
    setDraftCommand((current) => {
      const next = clone(current);
      for (const legId of LEG_IDS) {
        next[legId].servoChannelMap = payload.legs?.[legId]?.servoChannelMap ?? next[legId].servoChannelMap;
        next[legId].jointLimits = payload.legs?.[legId]?.jointLimits ?? next[legId].jointLimits;
      }
      return next;
    });
    if (!selectedPort && payload.connectedPort) {
      setSelectedPort(payload.connectedPort);
    }
  }

  useEffect(() => {
    fetchStatus().catch((error) => setRobotState((current) => ({ ...current, lastError: error.message })));
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const socket = new WebSocket(`${protocol}//${window.location.host}/telemetry`);

    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "status" || message.type === "state" || message.type === "hello_ack") {
          setRobotState((current) => mergeState(current, message.payload));
          setDraftCommand((current) => {
            const next = clone(current);
            for (const legId of LEG_IDS) {
              next[legId].servoChannelMap = message.payload.legs?.[legId]?.servoChannelMap ?? next[legId].servoChannelMap;
              next[legId].jointLimits = message.payload.legs?.[legId]?.jointLimits ?? next[legId].jointLimits;
            }
            return next;
          });
        }
        if (message.type === "ack") {
          setRobotState((current) => ({ ...current, lastAck: message.message ?? message.seq ?? "ack" }));
        }
        if (message.type === "error") {
          setRobotState((current) => ({ ...current, lastError: message.message || "Unknown firmware error" }));
        }
        if (message.type === "animation_progress") {
          setRobotState((current) => ({ ...current, activeAnimation: message.name || current.activeAnimation }));
        }
      } catch (error) {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      }
    };

    return () => socket.close();
  }, []);

  async function postJson(path, body) {
    const response = await fetch(path, {
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
    await postJson("/api/command", { command });
  }

  function applyDesiredFootLocally(legId, foot) {
    const currentPose = desiredPoseRef.current[legId] ?? robotState.legs[legId]?.desired;
    const jointLimits = jointLimitsRef.current[legId] ?? robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    const pose = buildLegPoseFromFoot(foot, calibration, {
      startThetaThigh: currentPose?.geometry?.thetaThigh,
      startThetaServo: currentPose?.geometry?.thetaServo,
      jointLimits,
    });
    const nextPose = pose.reachable ? pose : currentPose;
    desiredPoseRef.current[legId] = nextPose;
    setRobotState((current) =>
      mergeState(current, {
        mode: "direct_foot_xy",
        servosReleased: false,
        legs: {
          [legId]: {
            ...current.legs[legId],
            desired: nextPose,
          },
        },
      }),
    );
    setDraftCommand((current) => ({
      ...current,
      [legId]: {
        ...current[legId],
        foot: { ...(nextPose?.foot ?? foot) },
      },
    }));
    return nextPose ?? pose;
  }

  function flushQueuedDragFootCommand() {
    if (dragSendInFlightRef.current || !dragQueuedCommandRef.current) {
      return;
    }

    const queued = dragQueuedCommandRef.current;
    dragQueuedCommandRef.current = null;
    dragSendInFlightRef.current = true;

    sendCommand({ type: "set_leg_foot_xy", legId: queued.legId, x: queued.foot.x, y: queued.foot.y })
      .catch((error) => {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      })
      .finally(() => {
        dragSendInFlightRef.current = false;
        if (dragQueuedCommandRef.current) {
          flushQueuedDragFootCommand();
        }
      });
  }

  function queueLiveFootTarget(legId, foot) {
    const pose = applyDesiredFootLocally(legId, foot);
    dragQueuedCommandRef.current = { legId, foot: pose?.foot ?? foot };
    flushQueuedDragFootCommand();
  }

  async function connect() {
    if (!selectedPort) {
      setRobotState((current) => ({ ...current, lastError: "Select a serial port before connecting." }));
      return;
    }
    await postJson("/api/connect", { path: selectedPort, baudRate: 115200 });
    await fetchStatus();
  }

  async function disconnect() {
    await postJson("/api/disconnect", {});
    await fetchStatus();
  }

  async function sendFoot(legId, foot) {
    const pose = applyDesiredFootLocally(legId, foot);
    const nextFoot = pose?.foot ?? foot;
    await sendCommand({ type: "set_leg_foot_xy", legId, x: nextFoot.x, y: nextFoot.y });
  }

  async function sendJoint(legId, jointAnglesDeg) {
    const jointLimits = robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    const pose = buildLegPoseFromJointAngles(jointAnglesDeg, calibration, {
      startThetaServo: robotState.legs[legId]?.desired?.geometry?.thetaServo,
      jointLimits,
    });
    desiredPoseRef.current[legId] = pose;
    setRobotState((current) =>
      mergeState(current, {
        mode: "direct_joint_angles",
        servosReleased: false,
        legs: {
          [legId]: {
            ...current.legs[legId],
            desired: pose,
          },
        },
      }),
    );
    await sendCommand({ type: "set_leg_joint_angles", legId, thighDeg: jointAnglesDeg.thigh, calfDeg: jointAnglesDeg.calf });
  }

  async function sendServo(legId, servoAnglesDeg) {
    const jointLimits = robotState.legs[legId]?.jointLimits ?? DEFAULT_JOINT_LIMITS;
    const pose = buildLegPoseFromServoAngles(servoAnglesDeg, calibration, { jointLimits });
    desiredPoseRef.current[legId] = pose;
    setRobotState((current) =>
      mergeState(current, {
        mode: "direct_servo_angles",
        servosReleased: false,
        legs: {
          [legId]: {
            ...current.legs[legId],
            desired: pose,
          },
        },
      }),
    );
    await sendCommand({ type: "set_leg_servo_angles", legId, thighServoDeg: servoAnglesDeg.thigh, calfServoDeg: servoAnglesDeg.calf });
  }

  async function sendChannelMap(legId, servoChannelMap) {
    const nextMap = {
      thigh: clampChannel(servoChannelMap.thigh),
      calf: clampChannel(servoChannelMap.calf),
    };
    setRobotState((current) =>
      mergeState(current, {
        legs: {
          [legId]: {
            ...current.legs[legId],
            servoChannelMap: nextMap,
          },
        },
      }),
    );
    await sendCommand({
      type: "set_leg_servo_channel_map",
      legId,
      thighChannel: nextMap.thigh,
      calfChannel: nextMap.calf,
    });
  }

  async function sendJointLimits(legId, jointLimits) {
    const normalized = normalizeJointLimits(jointLimits);
    jointLimitsRef.current[legId] = normalized;
    const nextDesired = buildLegPoseFromFoot(robotState.legs[legId].desired.foot, calibration, {
      startThetaThigh: robotState.legs[legId].desired.geometry?.thetaThigh,
      startThetaServo: robotState.legs[legId].desired.geometry?.thetaServo,
      jointLimits: normalized,
    });
    desiredPoseRef.current[legId] = nextDesired;
    setRobotState((current) =>
      mergeState(current, {
        legs: {
          [legId]: {
            ...current.legs[legId],
            jointLimits: normalized,
            desired: nextDesired,
          },
        },
      }),
    );
    await sendCommand({
      type: "set_leg_joint_limits",
      legId,
      thighMinDeg: normalized.thighDeg.min,
      thighMaxDeg: normalized.thighDeg.max,
      calfMinDeg: normalized.calfDeg.min,
      calfMaxDeg: normalized.calfDeg.max,
    });
  }

  async function runBuiltin(name) {
    await sendCommand({ type: "run_builtin", name });
    setRobotState((current) => ({ ...current, mode: name === "walk" ? "builtin_walk" : "builtin_crouch", servosReleased: false }));
  }

  async function stopMotion() {
    await sendCommand({ type: "set_mode", mode: "idle" });
    setRobotState((current) => ({ ...current, mode: "idle" }));
  }

  async function releaseServos() {
    await sendCommand({ type: "release_servos" });
    setRobotState((current) => ({
      ...current,
      mode: "idle",
      servosReleased: true,
      activeAnimation: null,
      lastAck: "servos released",
    }));
  }

  async function uploadClip() {
    const validated = validateClip(clip);
    await postJson("/api/animations", { clip: validated, frames: createUploadFrames(validated) });
    setRobotState((current) => ({ ...current, activeAnimation: validated.name }));
  }

  async function playClip() {
    const validated = validateClip(clip);
    await postJson(`/api/animations/${encodeURIComponent(validated.name)}/play`, {});
    setRobotState((current) => ({ ...current, mode: "animation_playback", activeAnimation: validated.name, servosReleased: false }));
  }

  async function stopClip() {
    const validated = validateClip(clip);
    await postJson(`/api/animations/${encodeURIComponent(validated.name)}/stop`, {});
    setRobotState((current) => ({ ...current, mode: "idle" }));
  }

  const displayedLegs = useMemo(() => (viewMode === "single" ? [selectedLegId] : LEG_IDS), [selectedLegId, viewMode]);

  const previewRobotState = useMemo(() => {
    if (poseSource === "current" || !previewEnabled) {
      return robotState;
    }

    const previewFrame = interpolateFullBodyClip(clip, previewTime);
    const next = clone(robotState);
    for (const legId of LEG_IDS) {
      next.legs[legId].desired = buildLegPoseFromFoot(previewFrame[legId], calibration, {
        startThetaThigh: next.legs[legId].desired?.geometry?.thetaThigh,
        startThetaServo: next.legs[legId].desired?.geometry?.thetaServo,
        jointLimits: next.legs[legId].jointLimits,
      });
    }
    return next;
  }, [clip, poseSource, previewTime, robotState, previewEnabled]);

  function withError(handler) {
    return (...args) =>
      handler(...args).catch((error) => {
        setRobotState((current) => ({ ...current, lastError: error.message }));
      });
  }

  return (
    <main className="app-shell">
      <button className={`panic-button ${robotState.servosReleased ? "panic-button-released" : ""}`} onClick={withError(releaseServos)}>
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
          <button className={viewMode === "full" ? "toggle-active" : ""} onClick={() => setViewMode("full")}>
            Full robot
          </button>
          <button className={viewMode === "single" ? "toggle-active" : ""} onClick={() => setViewMode("single")}>
            Single leg
          </button>
        </div>
        <div className="toggle-group">
          <button className={poseSource === "desired" ? "toggle-active" : ""} onClick={() => setPoseSource("desired")}>
            Desired pose
          </button>
          <button className={poseSource === "current" ? "toggle-active" : ""} onClick={() => setPoseSource("current")}>
            Current pose
          </button>
        </div>
        <label className="inline-field">
          Focus leg
          <select value={selectedLegId} onChange={(event) => setSelectedLegId(event.target.value)}>
            {LEG_IDS.map((legId) => (
              <option key={legId} value={legId}>
                {LEG_LABELS[legId]}
              </option>
            ))}
          </select>
        </label>
      </div>

      {viewMode === "single" ? <SingleLegSelector selectedLegId={selectedLegId} setSelectedLegId={setSelectedLegId} /> : null}

      {viewMode === "single" ? (
        <section className="single-focus-grid">
          <RobotOverview robotState={previewRobotState} poseSource={poseSource} selectedLegId={selectedLegId} onSelectLeg={setSelectedLegId} />
          <LegDetail
            legId={selectedLegId}
            title={`${LEG_LABELS[selectedLegId]} ${poseSource}`}
            legState={previewRobotState.legs[selectedLegId][poseSource]}
            highlight
            interactive={poseSource === "desired"}
            onLiveFootTarget={queueLiveFootTarget}
          />
        </section>
      ) : (
        <>
          <RobotOverview robotState={previewRobotState} poseSource={poseSource} selectedLegId={selectedLegId} onSelectLeg={setSelectedLegId} />
          <section className="details-grid">
            {displayedLegs.map((legId) => (
              <LegDetail
                key={legId}
                legId={legId}
                title={`${LEG_LABELS[legId]} ${poseSource}`}
                legState={previewRobotState.legs[legId][poseSource]}
                highlight={legId === selectedLegId}
                interactive={false}
                onLiveFootTarget={queueLiveFootTarget}
              />
            ))}
          </section>
        </>
      )}

      <ControlPanel
        selectedLegId={selectedLegId}
        draftCommand={draftCommand}
        setDraftCommand={setDraftCommand}
        robotState={robotState}
        sendFoot={withError(sendFoot)}
        sendJoint={withError(sendJoint)}
        sendServo={withError(sendServo)}
        sendChannelMap={withError(sendChannelMap)}
        sendJointLimits={withError(sendJointLimits)}
        runBuiltin={withError(runBuiltin)}
        stopMotion={withError(stopMotion)}
      />

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
        uploadClip={withError(uploadClip)}
        playClip={withError(playClip)}
        stopClip={withError(stopClip)}
      />

      <TelemetryPanel robotState={robotState} selectedLegId={selectedLegId} />
    </main>
  );
}
