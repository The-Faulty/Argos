// Walking controls: stance-spread + cycle-time sliders, XY twist joystick,
// Rotate L/R buttons.
//
// The XY joystick uses an effect-driven 10 Hz heartbeat: while the pointer is
// captured, a setInterval re-emits the last twist value. Without the
// heartbeat, only pointermove fires events — a held-still deflected stick
// would expire on the server (500 ms watchdog) and the robot would stop
// mid-walk. The heartbeat is bound to a `pointerActive` state so React's
// useEffect lifecycle handles start/stop cleanly across re-renders, strict-
// mode double-mounts, and edge cases.
//
// Rotate L/R posts to /api/rotate and lets the server handle the timed spin
// (so a browser hiccup mid-rotation doesn't leave the robot spinning).

import { useEffect, useRef, useState } from "react";
import { GAIT_TUNABLE_PARAMS, JOYSTICK_SCALE } from "../../shared/robot-config.js";

const JOYSTICK_SIZE = 200;
const JOYSTICK_RADIUS = JOYSTICK_SIZE / 2 - 16;

export default function WalkingPanel({
  onTwist,
  onZeroTwist,
  onRotate,
  onGaitParam,
  rotateIncrement,
  stepLength,
  swingTimeMs,
  mode,
}) {
  // Joystick is enabled in any auto/idle pose — App.jsx auto-switches to trot
  // on first non-zero input. Direct modes (foot/joint/servo) keep it disabled
  // because flipping out of them would clobber the operator's manual pose.
  const twistCapable =
    mode === "trot" ||
    mode === "crawl" ||
    mode === "stand" ||
    mode === "crouch" ||
    mode === "extend" ||
    mode === "idle";

  const [stickPos, setStickPos] = useState({ dx: 0, dy: 0 });
  const [pointerActive, setPointerActive] = useState(false);
  const [step, setStep] = useState(stepLength ?? GAIT_TUNABLE_PARAMS.delta_x_mm.default);
  const [swing, setSwing] = useState(swingTimeMs ?? GAIT_TUNABLE_PARAMS.swing_time_ms.default);
  const svgRef = useRef(null);

  // Refs the heartbeat reads. Both are kept current via the dispatch path
  // (lastTwistRef from stickDispatch, onTwistRef from a ref-mirror effect)
  // so the heartbeat callback always sees the latest closure regardless of
  // when it last re-rendered.
  const lastTwistRef = useRef({ x: 0, y: 0, yaw: 0 });
  const onTwistRef = useRef(onTwist);
  useEffect(() => { onTwistRef.current = onTwist; }, [onTwist]);

  // Heartbeat. Bound to pointerActive so React handles the start/stop —
  // previously this was an imperative startHeartbeat/stopHeartbeat pair
  // gated on refs, which made it harder to reason about when the interval
  // would actually run. The empty-deps [pointerActive] re-runs ONLY when
  // capture state flips, so a parent re-render mid-drag doesn't cycle the
  // heartbeat and lose ticks.
  useEffect(() => {
    if (!pointerActive) return;
    const id = setInterval(() => {
      const t = lastTwistRef.current;
      const fn = onTwistRef.current;
      if (typeof fn === "function") fn(t.x, t.y, t.yaw);
    }, 100);
    return () => clearInterval(id);
  }, [pointerActive]);

  function handleStep(next) {
    setStep(next);
    onGaitParam?.("delta_x_mm", next);
  }
  function handleSwing(next) {
    setSwing(next);
    onGaitParam?.("swing_time_ms", next);
  }

  function stickDispatch(clientX, clientY) {
    const rect = svgRef.current.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const mag = Math.hypot(dx, dy);
    if (mag > JOYSTICK_RADIUS) {
      dx = (dx / mag) * JOYSTICK_RADIUS;
      dy = (dy / mag) * JOYSTICK_RADIUS;
    }
    setStickPos({ dx, dy });
    // Map: up is forward (positive x), left is +y (body left). Joystick is
    // normalized to [-1, 1]; apply a radial deadzone, then scale to the
    // planner's SI units. Without scaling, max deflection (1.0) was being
    // treated by the planner as 1 m/s and clamped to 0.6 — so any push past
    // ~half-stick was indistinguishable from full-tilt.
    const nxRaw = -dy / JOYSTICK_RADIUS;
    const nyRaw = -dx / JOYSTICK_RADIUS;
    const r = Math.hypot(nxRaw, nyRaw);
    const k = r < JOYSTICK_SCALE.DEADZONE
      ? 0
      : (r - JOYSTICK_SCALE.DEADZONE) / (1 - JOYSTICK_SCALE.DEADZONE) / r;
    const linX = nxRaw * k * JOYSTICK_SCALE.MAX_LIN_VEL;
    const linY = nyRaw * k * JOYSTICK_SCALE.MAX_LIN_VEL;
    lastTwistRef.current = { x: linX, y: linY, yaw: 0 };
    onTwist?.(linX, linY, 0);
  }

  function stickEnd(event) {
    setPointerActive(false);
    setStickPos({ dx: 0, dy: 0 });
    lastTwistRef.current = { x: 0, y: 0, yaw: 0 };
    onZeroTwist?.();
    try {
      svgRef.current?.releasePointerCapture(event.pointerId);
    } catch {
      /* already released */
    }
  }

  return (
    <section className="walking-panel">
      <header>
        <h3>Walking</h3>
        <span className="muted">mode: {mode}</span>
      </header>

      <div className="walking-panel__row">
        <label className="slider">
          <span>Stance spread, front/back ({GAIT_TUNABLE_PARAMS.delta_x_mm.units})</span>
          <input
            type="range"
            min={GAIT_TUNABLE_PARAMS.delta_x_mm.min}
            max={GAIT_TUNABLE_PARAMS.delta_x_mm.max}
            value={step}
            onChange={(e) => handleStep(Number(e.target.value))}
            disabled={!twistCapable}
          />
          <span className="slider__value">{step}</span>
        </label>
      </div>

      <div className="walking-panel__row">
        {/* Cycle-time slider: scales the planner's stance + swing timing
            in lockstep. Larger = slower, more deliberate gait. The other
            "speed" sliders the user might try (servo-speed limits in the
            settings drawer) only constrain INDIVIDUAL servo angular
            velocity — at default 180°/s the per-tick gait deltas are
            already well below the cap, so those sliders don't visibly
            slow walking. This one does. */}
        <label className="slider">
          <span>
            Step duration ({GAIT_TUNABLE_PARAMS.swing_time_ms.units}) — bigger = slower gait
          </span>
          <input
            type="range"
            min={GAIT_TUNABLE_PARAMS.swing_time_ms.min}
            max={GAIT_TUNABLE_PARAMS.swing_time_ms.max}
            step={5}
            value={swing}
            onChange={(e) => handleSwing(Number(e.target.value))}
            disabled={!twistCapable}
          />
          <span className="slider__value">{swing}</span>
        </label>
      </div>

      <div className="walking-panel__row walking-panel__row--split">
        <div className="walking-panel__joy">
          <svg
            ref={svgRef}
            width={JOYSTICK_SIZE}
            height={JOYSTICK_SIZE}
            viewBox={`0 0 ${JOYSTICK_SIZE} ${JOYSTICK_SIZE}`}
            style={{ touchAction: "none", cursor: twistCapable ? "grab" : "not-allowed" }}
            onPointerDown={(e) => {
              if (!twistCapable) return;
              svgRef.current.setPointerCapture(e.pointerId);
              stickDispatch(e.clientX, e.clientY);
              setPointerActive(true);
            }}
            onPointerMove={(e) => {
              // Gate on the pointerActive state so stale moves after release
              // don't dispatch. Reading from state here (not a ref) is fine
              // because pointermove only fires while the pointer is captured,
              // and capture follows pointerActive within one render.
              if (pointerActive) stickDispatch(e.clientX, e.clientY);
            }}
            onPointerUp={stickEnd}
            onPointerCancel={stickEnd}
            aria-label="Twist joystick"
          >
            <circle
              cx={JOYSTICK_SIZE / 2} cy={JOYSTICK_SIZE / 2}
              r={JOYSTICK_RADIUS + 10}
              fill="rgba(17,17,17,0.06)"
              stroke="#111"
            />
            <circle
              cx={JOYSTICK_SIZE / 2 + stickPos.dx} cy={JOYSTICK_SIZE / 2 + stickPos.dy}
              r={20}
              fill={pointerActive ? "#2563eb" : "#111"}
            />
          </svg>
        </div>

        <div className="walking-panel__rotate">
          <div className="walking-panel__rotate-row">
            <button type="button" className="rotate-btn" onClick={() => onRotate?.("left", rotateIncrement)}>
              ↺ Rotate L
            </button>
            <button type="button" className="rotate-btn" onClick={() => onRotate?.("right", rotateIncrement)}>
              ↻ Rotate R
            </button>
          </div>
          <p className="muted">
            per click: {rotateIncrement}°
          </p>
        </div>
      </div>
    </section>
  );
}
