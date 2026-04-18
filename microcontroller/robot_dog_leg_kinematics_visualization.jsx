import React, { useEffect, useMemo, useRef, useState } from "react";

const L_THIGH = 127;
const L_CALF = 127;
const HORN = 20;
const LINK_SHORT = 30;
const BELL = 40;
const LINK_LONG = 150;
const SERVO_PIVOT = { x: -20, y: -22 };
const HIP_PIVOT = { x: 0, y: 0 };
const CALF_ATTACH_OFFSET = 30;
const SCALE = 2;
const OFFSET = { x: 300, y: 220 };
const STAGE_WIDTH = 700;
const STAGE_HEIGHT = 650;
const DEFAULT_DURATION = 2;
const DEFAULT_FOOT = { x: 40, y: -140 };
const DEFAULT_WALK_KEYFRAMES = [
  { id: "walk-0", time: 0, foot: { x: 30, y: -140 } },
  { id: "walk-1", time: 0.5, foot: { x: 48, y: -132 } },
  { id: "walk-2", time: 1, foot: { x: 56, y: -140 } },
  { id: "walk-3", time: 1.5, foot: { x: 40, y: -148 } },
  { id: "walk-4", time: 2, foot: { x: 30, y: -140 } },
];

function circleIntersections(c0, r0, c1, r1) {
  const dx = c1.x - c0.x;
  const dy = c1.y - c0.y;
  const d = Math.hypot(dx, dy);

  if (d === 0 || d > r0 + r1 || d < Math.abs(r0 - r1)) {
    return [];
  }

  const a = (r0 * r0 - r1 * r1 + d * d) / (2 * d);
  const hSquared = r0 * r0 - a * a;

  if (hSquared < 0) {
    return [];
  }

  const h = Math.sqrt(hSquared);
  const xm = c0.x + (a * dx) / d;
  const ym = c0.y + (a * dy) / d;
  const rx = (-dy * h) / d;
  const ry = (dx * h) / d;

  return [
    { x: xm + rx, y: ym + ry },
    { x: xm - rx, y: ym - ry },
  ];
}

function chooseBellPoint(points) {
  if (points.length === 0) {
    return null;
  }

  return points.reduce((best, point) => {
    if (best === null) {
      return point;
    }

    return point.y > best.y ? point : best;
  }, null);
}

function chooseCalfAttachPoint(points, knee) {
  if (points.length === 0) {
    return null;
  }

  return points.reduce((best, point) => {
    const score = (knee.y - point.y) * 3 + Math.abs(knee.x - point.x);

    if (best === null) {
      return { point, score };
    }

    return score < best.score ? { point, score } : best;
  }, null)?.point ?? null;
}

function clampAngle(angle) {
  const twoPi = Math.PI * 2;
  let value = angle % twoPi;

  if (value <= -Math.PI) {
    value += twoPi;
  }

  if (value > Math.PI) {
    value -= twoPi;
  }

  return value;
}

function solveGeometry(thetaThigh, thetaServo) {
  const hip = HIP_PIVOT;
  const servoPivot = SERVO_PIVOT;

  const knee = {
    x: hip.x + L_THIGH * Math.cos(thetaThigh),
    y: hip.y + L_THIGH * Math.sin(thetaThigh),
  };

  const servoHornEnd = {
    x: servoPivot.x + HORN * Math.cos(thetaServo),
    y: servoPivot.y + HORN * Math.sin(thetaServo),
  };

  const bellCandidates = circleIntersections(hip, BELL, servoHornEnd, LINK_SHORT);
  const bellArmA = chooseBellPoint(bellCandidates) ?? {
    x: hip.x + BELL * Math.cos(thetaServo),
    y: hip.y + BELL * Math.sin(thetaServo),
  };

  const bellAngle = Math.atan2(bellArmA.y - hip.y, bellArmA.x - hip.x);
  const bellArmB = {
    x: hip.x + BELL * Math.cos(bellAngle - Math.PI / 2),
    y: hip.y + BELL * Math.sin(bellAngle - Math.PI / 2),
  };

  const calfAttachCandidates = circleIntersections(bellArmB, LINK_LONG, knee, CALF_ATTACH_OFFSET);
  const calfAttach = chooseCalfAttachPoint(calfAttachCandidates, knee) ?? {
    x: knee.x - CALF_ATTACH_OFFSET,
    y: knee.y,
  };

  const thetaCalf = Math.atan2(knee.y - calfAttach.y, knee.x - calfAttach.x);
  const foot = {
    x: knee.x + L_CALF * Math.cos(thetaCalf),
    y: knee.y + L_CALF * Math.sin(thetaCalf),
  };

  return {
    hip,
    servoPivot,
    knee,
    servoHornEnd,
    bellArmA,
    bellArmB,
    calfAttach,
    foot,
    thetaCalf,
  };
}

function solveAnglesForFoot(target, startThigh, startServo) {
  let bestThigh = startThigh;
  let bestServo = startServo;
  let bestGeometry = solveGeometry(bestThigh, bestServo);
  let bestError = Math.hypot(bestGeometry.foot.x - target.x, bestGeometry.foot.y - target.y);
  let stepThigh = 0.2;
  let stepServo = 0.2;

  for (let i = 0; i < 28; i += 1) {
    let improved = false;
    const candidates = [
      [bestThigh + stepThigh, bestServo],
      [bestThigh - stepThigh, bestServo],
      [bestThigh, bestServo + stepServo],
      [bestThigh, bestServo - stepServo],
      [bestThigh + stepThigh, bestServo + stepServo],
      [bestThigh + stepThigh, bestServo - stepServo],
      [bestThigh - stepThigh, bestServo + stepServo],
      [bestThigh - stepThigh, bestServo - stepServo],
    ];

    for (const [candidateThigh, candidateServo] of candidates) {
      const thigh = clampAngle(candidateThigh);
      const servo = clampAngle(candidateServo);
      const geometry = solveGeometry(thigh, servo);
      const error = Math.hypot(geometry.foot.x - target.x, geometry.foot.y - target.y);

      if (error < bestError) {
        bestError = error;
        bestThigh = thigh;
        bestServo = servo;
        bestGeometry = geometry;
        improved = true;
      }
    }

    if (!improved) {
      stepThigh *= 0.5;
      stepServo *= 0.5;
    }
  }

  return {
    thetaThigh: bestThigh,
    thetaServo: bestServo,
    geometry: bestGeometry,
  };
}

function interpolateFoot(keyframes, time) {
  if (keyframes.length === 0) {
    return null;
  }

  const ordered = [...keyframes].sort((a, b) => a.time - b.time);

  if (time <= ordered[0].time) {
    return ordered[0].foot;
  }

  if (time >= ordered[ordered.length - 1].time) {
    return ordered[ordered.length - 1].foot;
  }

  for (let i = 0; i < ordered.length - 1; i += 1) {
    const a = ordered[i];
    const b = ordered[i + 1];

    if (time >= a.time && time <= b.time) {
      const span = b.time - a.time || 1;
      const t = (time - a.time) / span;
      return {
        x: a.foot.x + (b.foot.x - a.foot.x) * t,
        y: a.foot.y + (b.foot.y - a.foot.y) * t,
      };
    }
  }

  return ordered[ordered.length - 1].foot;
}

function downloadTextFile(filename, content) {
  const blob = new Blob([content], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

export default function RobotLegViz() {
  const init = solveAnglesForFoot(DEFAULT_FOOT, -Math.PI / 4, Math.PI / 6);
  const [thetaThigh, setThetaThigh] = useState(init.thetaThigh);
  const [thetaServo, setThetaServo] = useState(init.thetaServo);
  const [dragTarget, setDragTarget] = useState(null);
  const [keyframes, setKeyframes] = useState(DEFAULT_WALK_KEYFRAMES);
  const [playhead, setPlayhead] = useState(0);
  const [duration, setDuration] = useState(DEFAULT_DURATION);
  const [isPlaying, setIsPlaying] = useState(false);
  const [timelineDragId, setTimelineDragId] = useState(null);
  const [animationName, setAnimationName] = useState("robot-dog-step");
  const svgRef = useRef(null);
  const fileInputRef = useRef(null);
  const timelineRef = useRef(null);
  const lastFrameTimeRef = useRef(null);

  function toCanvasPoint(point) {
    return {
      x: point.x * SCALE + OFFSET.x,
      y: -point.y * SCALE + OFFSET.y,
    };
  }

  function fromCanvasPoint(x, y) {
    return {
      x: (x - OFFSET.x) / SCALE,
      y: -(y - OFFSET.y) / SCALE,
    };
  }

  function getPointerPoint(event) {
    const svg = svgRef.current;

    if (!svg) {
      return null;
    }

    const rect = svg.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top,
    };
  }

  function Segment({ a, b, color, width = 3 }) {
    const start = toCanvasPoint(a);
    const end = toCanvasPoint(b);
    return <line x1={start.x} y1={start.y} x2={end.x} y2={end.y} stroke={color} strokeWidth={width} strokeLinecap="round" />;
  }

  function Joint({ point, radius = 5, fill = "black", draggable = false, onPointerDown }) {
    const p = toCanvasPoint(point);
    return <circle cx={p.x} cy={p.y} r={radius} fill={fill} style={draggable ? { cursor: "grab" } : undefined} onMouseDown={onPointerDown} />;
  }

  function Label({ point, text }) {
    const p = toCanvasPoint(point);
    return (
      <text x={p.x + 8} y={p.y - 8} fontSize="12" fill="#222">
        {text}
      </text>
    );
  }

  const geometry = useMemo(() => solveGeometry(thetaThigh, thetaServo), [thetaServo, thetaThigh]);

  useEffect(() => {
    if (!isPlaying) {
      lastFrameTimeRef.current = null;
      return;
    }

    let frameId;

    function step(timestamp) {
      if (lastFrameTimeRef.current == null) {
        lastFrameTimeRef.current = timestamp;
      }

      const delta = (timestamp - lastFrameTimeRef.current) / 1000;
      lastFrameTimeRef.current = timestamp;

      setPlayhead((current) => {
        const next = current + delta;
        return next > duration ? 0 : next;
      });

      frameId = window.requestAnimationFrame(step);
    }

    frameId = window.requestAnimationFrame(step);

    return () => window.cancelAnimationFrame(frameId);
  }, [duration, isPlaying]);

  useEffect(() => {
    if (!isPlaying || keyframes.length === 0) {
      return;
    }

    const targetFoot = interpolateFoot(keyframes, playhead);

    if (!targetFoot) {
      return;
    }

    const solution = solveAnglesForFoot(targetFoot, thetaThigh, thetaServo);
    setThetaThigh(solution.thetaThigh);
    setThetaServo(solution.thetaServo);
  }, [isPlaying, keyframes, playhead, thetaServo, thetaThigh]);

  function startDrag(target, event) {
    event.stopPropagation();
    setIsPlaying(false);
    setDragTarget(target);
  }

  function handlePointerMove(event) {
    if (!dragTarget) {
      return;
    }

    const pointer = getPointerPoint(event);

    if (!pointer) {
      return;
    }

    const world = fromCanvasPoint(pointer.x, pointer.y);

    if (dragTarget === "knee") {
      const angle = Math.atan2(world.y - HIP_PIVOT.y, world.x - HIP_PIVOT.x);
      setThetaThigh(clampAngle(angle));
    }

    if (dragTarget === "servoHornEnd") {
      const angle = Math.atan2(world.y - SERVO_PIVOT.y, world.x - SERVO_PIVOT.x);
      setThetaServo(clampAngle(angle));
    }

    if (dragTarget === "foot") {
      const solution = solveAnglesForFoot(world, thetaThigh, thetaServo);
      setThetaThigh(solution.thetaThigh);
      setThetaServo(solution.thetaServo);
    }
  }

  function endDrag() {
    setDragTarget(null);
  }

  function startTimelineDrag(id, event) {
    event.preventDefault();
    event.stopPropagation();
    setIsPlaying(false);
    setTimelineDragId(id);
  }

  function updateTimelineDrag(clientX) {
    if (!timelineDragId || !timelineRef.current) {
      return;
    }

    const rect = timelineRef.current.getBoundingClientRect();
    const ratio = rect.width > 0 ? (clientX - rect.left) / rect.width : 0;
    let nextTime = Math.max(0, Math.min(duration, ratio * duration));
    nextTime = Math.round(nextTime / 0.05) * 0.05;
    updateKeyframeTime(timelineDragId, nextTime);
    setPlayhead(nextTime);
  }

  function endTimelineDrag() {
    setTimelineDragId(null);
  }

  function addKeyframe() {
    const frame = {
      id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
      time: Number(playhead.toFixed(3)),
      foot: {
        x: Number(geometry.foot.x.toFixed(3)),
        y: Number(geometry.foot.y.toFixed(3)),
      },
    };

    setKeyframes((current) => {
      const filtered = current.filter((item) => Math.abs(item.time - frame.time) > 0.001);
      return [...filtered, frame].sort((a, b) => a.time - b.time);
    });
  }

  function removeKeyframe(id) {
    setKeyframes((current) => current.filter((item) => item.id !== id));
  }

  function updateKeyframeTime(id, timeValue) {
    const nextTime = Math.max(0, Math.min(duration, Number(timeValue) || 0));
    setKeyframes((current) => current.map((item) => (item.id === id ? { ...item, time: nextTime } : item)).sort((a, b) => a.time - b.time));
  }

  function jumpToTime(nextTime) {
    const clamped = Math.max(0, Math.min(duration, nextTime));
    setPlayhead(clamped);

    if (keyframes.length > 0) {
      const targetFoot = interpolateFoot(keyframes, clamped);
      if (targetFoot) {
        const solution = solveAnglesForFoot(targetFoot, thetaThigh, thetaServo);
        setThetaThigh(solution.thetaThigh);
        setThetaServo(solution.thetaServo);
      }
    }
  }

  function saveAnimation() {
    const payload = {
      version: 1,
      name: animationName,
      duration,
      keyframes: keyframes.map((item) => ({
        time: item.time,
        foot: item.foot,
      })),
    };

    downloadTextFile(`${animationName || "robot-dog-animation"}.json`, JSON.stringify(payload, null, 2));
  }

  function loadAnimationFromFile(event) {
    const file = event.target.files?.[0];

    if (!file) {
      return;
    }

    const reader = new FileReader();
    reader.onload = () => {
      try {
        const parsed = JSON.parse(String(reader.result || "{}"));
        const nextDuration = Math.max(0.1, Number(parsed.duration) || DEFAULT_DURATION);
        const nextKeyframes = Array.isArray(parsed.keyframes)
          ? parsed.keyframes
              .filter((item) => item && typeof item.time === "number" && item.foot && typeof item.foot.x === "number" && typeof item.foot.y === "number")
              .map((item, index) => ({
                id: `loaded-${index}-${Math.random().toString(36).slice(2, 8)}`,
                time: Math.max(0, Math.min(nextDuration, item.time)),
                foot: {
                  x: item.foot.x,
                  y: item.foot.y,
                },
              }))
              .sort((a, b) => a.time - b.time)
          : [];

        setAnimationName(typeof parsed.name === "string" && parsed.name ? parsed.name : "loaded-animation");
        setDuration(nextDuration);
        setKeyframes(nextKeyframes);
        setPlayhead(0);
        setIsPlaying(false);

        if (nextKeyframes.length > 0) {
          const solution = solveAnglesForFoot(nextKeyframes[0].foot, thetaThigh, thetaServo);
          setThetaThigh(solution.thetaThigh);
          setThetaServo(solution.thetaServo);
        }
      } catch (error) {
        console.error(error);
      }
    };

    reader.readAsText(file);
    event.target.value = "";
  }

  const timelineMarkers = keyframes.map((item) => ({
    ...item,
    left: duration > 0 ? (item.time / duration) * 100 : 0,
  }));

  useEffect(() => {
    if (!timelineDragId) {
      return undefined;
    }

    function handleWindowMove(event) {
      updateTimelineDrag(event.clientX);
    }

    function handleWindowUp() {
      endTimelineDrag();
    }

    window.addEventListener("mousemove", handleWindowMove);
    window.addEventListener("mouseup", handleWindowUp);

    return () => {
      window.removeEventListener("mousemove", handleWindowMove);
      window.removeEventListener("mouseup", handleWindowUp);
    };
  }, [timelineDragId, duration]);

  return (
    <div className="w-full h-full bg-white select-none p-4">
      <div className="grid grid-cols-1 lg:grid-cols-[720px_1fr] gap-4 items-start">
        <div className="border rounded-2xl shadow-sm p-3 bg-white">
          <svg
            ref={svgRef}
            width={STAGE_WIDTH}
            height={STAGE_HEIGHT}
            viewBox={`0 0 ${STAGE_WIDTH} ${STAGE_HEIGHT}`}
            onMouseMove={handlePointerMove}
            onMouseUp={endDrag}
            onMouseLeave={endDrag}
          >
            <Segment a={geometry.hip} b={geometry.knee} color="black" width={4} />
            <Segment a={geometry.knee} b={geometry.foot} color="#2563eb" width={4} />
            <Segment a={geometry.hip} b={geometry.bellArmA} color="#16a34a" />
            <Segment a={geometry.hip} b={geometry.bellArmB} color="#16a34a" />
            <Segment a={geometry.servoPivot} b={geometry.servoHornEnd} color="#dc2626" />
            <Segment a={geometry.servoHornEnd} b={geometry.bellArmA} color="#ea580c" />
            <Segment a={geometry.bellArmB} b={geometry.calfAttach} color="#7c3aed" />
            <Joint point={geometry.hip} />
            <Joint point={geometry.knee} fill="#111827" draggable onPointerDown={(event) => startDrag("knee", event)} />
            <Joint point={geometry.foot} fill="#1d4ed8" draggable onPointerDown={(event) => startDrag("foot", event)} />
            <Joint point={geometry.servoPivot} />
            <Joint point={geometry.servoHornEnd} fill="#b91c1c" draggable onPointerDown={(event) => startDrag("servoHornEnd", event)} />
            <Joint point={geometry.bellArmA} />
            <Joint point={geometry.bellArmB} />
            <Joint point={geometry.calfAttach} />
            <Label point={geometry.hip} text="Hip" />
            <Label point={geometry.knee} text="Knee" />
            <Label point={geometry.foot} text="Foot" />
            <Label point={geometry.servoPivot} text="Calf Servo" />
            <Label point={geometry.servoHornEnd} text="Horn" />
            <Label point={geometry.bellArmA} text="Bell A" />
            <Label point={geometry.bellArmB} text="Bell B" />
            <Label point={geometry.calfAttach} text="Calf Link" />
          </svg>
        </div>

        <div className="border rounded-2xl shadow-sm p-4 bg-white space-y-4">
          <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
            <label className="text-sm">
              <div className="mb-1 text-gray-700">Animation name</div>
              <input className="w-full border rounded-lg px-3 py-2" value={animationName} onChange={(e) => setAnimationName(e.target.value)} />
            </label>
            <label className="text-sm">
              <div className="mb-1 text-gray-700">Duration (seconds)</div>
              <input
                className="w-full border rounded-lg px-3 py-2"
                type="number"
                min="0.1"
                step="0.1"
                value={duration}
                onChange={(e) => {
                  const next = Math.max(0.1, Number(e.target.value) || DEFAULT_DURATION);
                  setDuration(next);
                  setPlayhead((current) => Math.min(current, next));
                }}
              />
            </label>
          </div>

          <div className="flex flex-wrap gap-2">
            <button className="px-3 py-2 rounded-xl bg-black text-white" onClick={() => setIsPlaying((value) => !value)}>
              {isPlaying ? "Pause" : "Play"}
            </button>
            <button className="px-3 py-2 rounded-xl border" onClick={() => jumpToTime(0)}>
              Stop
            </button>
            <button className="px-3 py-2 rounded-xl border" onClick={addKeyframe}>
              Add keyframe at playhead
            </button>
            <button className="px-3 py-2 rounded-xl border" onClick={saveAnimation}>
              Save animation
            </button>
            <button className="px-3 py-2 rounded-xl border" onClick={() => fileInputRef.current?.click()}>
              Load animation
            </button>
            <input ref={fileInputRef} type="file" accept="application/json" className="hidden" onChange={loadAnimationFromFile} />
          </div>

          <div>
            <div className="mb-1 text-sm text-gray-700">Playhead: {playhead.toFixed(2)}s</div>
            <input
              className="w-full"
              type="range"
              min="0"
              max={duration}
              step="0.01"
              value={playhead}
              onChange={(e) => {
                setIsPlaying(false);
                jumpToTime(Number(e.target.value));
              }}
            />
          </div>

          <div>
            <div className="mb-2 text-sm text-gray-700">Timeline</div>
            <div ref={timelineRef} className="relative h-12 border rounded-xl bg-gray-50 overflow-hidden">
              <div className="absolute top-0 bottom-0 w-0.5 bg-red-500" style={{ left: `${duration > 0 ? (playhead / duration) * 100 : 0}%` }} />
              {timelineMarkers.map((item) => (
                <button
                  key={item.id}
                  className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 w-3 h-3 rounded-full bg-blue-600 border-2 border-white shadow cursor-ew-resize"
                  style={{ left: `${item.left}%` }}
                  onMouseDown={(event) => startTimelineDrag(item.id, event)}
                  onClick={() => {
                    if (timelineDragId) {
                      return;
                    }
                    setIsPlaying(false);
                    jumpToTime(item.time);
                  }}
                  title={`${item.time.toFixed(2)}s`}
                />
              ))}
            </div>
          </div>

          <div className="space-y-2 max-h-72 overflow-auto">
            {keyframes.length === 0 ? (
              <div className="text-sm text-gray-500">No keyframes yet.</div>
            ) : (
              keyframes.map((item) => (
                <div key={item.id} className="border rounded-xl p-3 grid grid-cols-[1fr_auto] gap-3 items-center">
                  <div className="grid grid-cols-3 gap-2 text-sm items-end">
                    <label>
                      <div className="mb-1 text-gray-700">Time</div>
                      <input
                        className="w-full border rounded-lg px-2 py-1"
                        type="number"
                        min="0"
                        max={duration}
                        step="0.01"
                        value={item.time}
                        onChange={(e) => updateKeyframeTime(item.id, e.target.value)}
                      />
                    </label>
                    <div>
                      <div className="mb-1 text-gray-700">Foot X</div>
                      <div className="border rounded-lg px-2 py-1 bg-gray-50">{item.foot.x.toFixed(2)}</div>
                    </div>
                    <div>
                      <div className="mb-1 text-gray-700">Foot Y</div>
                      <div className="border rounded-lg px-2 py-1 bg-gray-50">{item.foot.y.toFixed(2)}</div>
                    </div>
                  </div>
                  <div className="flex gap-2">
                    <button
                      className="px-3 py-2 rounded-xl border"
                      onClick={() => {
                        setIsPlaying(false);
                        jumpToTime(item.time);
                      }}
                    >
                      Go
                    </button>
                    <button className="px-3 py-2 rounded-xl border" onClick={() => removeKeyframe(item.id)}>
                      Delete
                    </button>
                  </div>
                </div>
              ))
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
