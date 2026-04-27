// Walking controls: step-length slider, XY twist joystick, Rotate L/R buttons.
//
// XY joystick streams /cmd_vel continuously while the pointer is down (30 Hz
// is handled server-side; we just send one command per visible change). On
// release, we send a zero-twist to settle the command_mux.
//
// Rotate L/R posts to /api/rotate and lets the server handle the timed spin
// (so a browser hiccup mid-rotation doesn't leave the robot spinning).

import { useRef, useState } from "react";
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

  const [stick, setStick] = useState({ dx: 0, dy: 0, active: false });
  const [step, setStep] = useState(stepLength ?? GAIT_TUNABLE_PARAMS.delta_x_mm.default);
  const svgRef = useRef(null);

  function handleStep(next) {
    setStep(next);
    onGaitParam?.("delta_x_mm", next);
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
    setStick({ dx, dy, active: true });
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
    onTwist?.(linX, linY, 0);
  }

  function stickEnd(event) {
    setStick({ dx: 0, dy: 0, active: false });
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
            }}
            onPointerMove={(e) => stick.active && stickDispatch(e.clientX, e.clientY)}
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
              cx={JOYSTICK_SIZE / 2 + stick.dx} cy={JOYSTICK_SIZE / 2 + stick.dy}
              r={20}
              fill={stick.active ? "#2563eb" : "#111"}
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
