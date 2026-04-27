// Walking controls: stance-spread + cycle-time sliders, arrow-button D-pad,
// Rotate L/R buttons.
//
// The arrow buttons are hold-to-walk: pointerdown sets the direction,
// pointerup/cancel clears it. Multiple buttons can be held at once for
// diagonals (forward + left → forward-left), normalized so diagonal speed
// matches cardinal speed. Keyboard ArrowUp/Down/Left/Right and W/A/S/D
// mirror the buttons (gated off when an input is focused).
//
// While any direction is held, an effect-driven 100 ms heartbeat re-emits
// the last twist value. Without the heartbeat, only the initial keydown /
// pointerdown would fire — a held-still arrow would expire on the server
// (500 ms watchdog) and the robot would stop mid-walk.
//
// Rotate L/R posts to /api/rotate and lets the server handle the timed
// spin (so a browser hiccup mid-rotation doesn't leave the robot spinning).

import { useEffect, useRef, useState } from "react";
import { GAIT_TUNABLE_PARAMS, JOYSTICK_SCALE } from "../../shared/robot-config.js";

const KEY_TO_DIR = {
  ArrowUp: "up", ArrowDown: "down", ArrowLeft: "left", ArrowRight: "right",
  w: "up", a: "left", s: "down", d: "right",
  W: "up", A: "left", S: "down", D: "right",
};

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
  // Arrow buttons are enabled in any auto/idle pose — App.jsx auto-switches
  // to trot on first non-zero input. Direct modes (foot/joint/servo) keep
  // them disabled because flipping out of them would clobber the operator's
  // manual pose.
  const twistCapable =
    mode === "trot" ||
    mode === "crawl" ||
    mode === "stand" ||
    mode === "crouch" ||
    mode === "extend" ||
    mode === "idle";

  const [active, setActive] = useState({ up: false, down: false, left: false, right: false });
  const [walking, setWalking] = useState(false);
  const [step, setStep] = useState(stepLength ?? GAIT_TUNABLE_PARAMS.delta_x_mm.default);
  const [swing, setSwing] = useState(swingTimeMs ?? GAIT_TUNABLE_PARAMS.swing_time_ms.default);

  useEffect(() => {
    if (Number.isFinite(stepLength)) setStep(stepLength);
  }, [stepLength]);

  useEffect(() => {
    if (Number.isFinite(swingTimeMs)) setSwing(swingTimeMs);
  }, [swingTimeMs]);

  // Refs the heartbeat reads. activeRef shadows `active` so setDir can
  // compute the next twist synchronously without waiting on a re-render —
  // a held arrow that fires keydown twice within one render cycle would
  // otherwise see stale state. onTwistRef/onZeroTwistRef mirror the props
  // so the heartbeat callback always sees the latest closure.
  const activeRef = useRef(active);
  const lastTwistRef = useRef({ x: 0, y: 0, yaw: 0 });
  const onTwistRef = useRef(onTwist);
  const onZeroTwistRef = useRef(onZeroTwist);
  useEffect(() => { onTwistRef.current = onTwist; }, [onTwist]);
  useEffect(() => { onZeroTwistRef.current = onZeroTwist; }, [onZeroTwist]);

  // Heartbeat. Bound to `walking` so React handles start/stop — empty deps
  // [walking] re-runs ONLY when at least one direction transitions on/off,
  // so a parent re-render mid-walk doesn't cycle the heartbeat and lose ticks.
  useEffect(() => {
    if (!walking) return;
    const id = setInterval(() => {
      const t = lastTwistRef.current;
      const fn = onTwistRef.current;
      if (typeof fn === "function") fn(t.x, t.y, t.yaw);
    }, 100);
    return () => clearInterval(id);
  }, [walking]);

  // Keyboard listeners. Skip when an input/textarea/contenteditable is
  // focused so typing in the SettingsDrawer doesn't drive the robot. Both
  // Arrow keys and WASD are bound; preventDefault stops Arrow keys from
  // scrolling the page.
  useEffect(() => {
    function onKeyDown(e) {
      if (e.repeat) return;
      const dir = KEY_TO_DIR[e.key];
      if (!dir) return;
      const el = document.activeElement;
      const tag = el?.tagName;
      if (tag === "INPUT" || tag === "TEXTAREA" || el?.isContentEditable) return;
      e.preventDefault();
      if (twistCapable) setDir(dir, true);
    }
    function onKeyUp(e) {
      const dir = KEY_TO_DIR[e.key];
      if (!dir) return;
      e.preventDefault();
      setDir(dir, false);
    }
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [twistCapable]);

  // If we lose twistCapable mid-walk (e.g. user switched to direct mode),
  // release everything so the planner doesn't hold a stale twist.
  useEffect(() => {
    if (!twistCapable && walking) clearAll();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [twistCapable]);

  function handleStep(next) {
    setStep(next);
    onGaitParam?.("delta_x_mm", next);
  }
  function handleSwing(next) {
    setSwing(next);
    onGaitParam?.("swing_time_ms", next);
  }

  function twistFromActive(a) {
    // Up=forward (+x), left=+y in body frame — same convention the joystick
    // used. Diagonals are normalized so two-key holds aren't √2× faster than
    // single-key holds.
    const x = (a.up ? 1 : 0) - (a.down ? 1 : 0);
    const y = (a.left ? 1 : 0) - (a.right ? 1 : 0);
    if (x === 0 && y === 0) return { x: 0, y: 0 };
    const mag = Math.hypot(x, y);
    return {
      x: (x / mag) * JOYSTICK_SCALE.MAX_LIN_VEL,
      y: (y / mag) * JOYSTICK_SCALE.MAX_LIN_VEL,
    };
  }

  function setDir(dir, on) {
    if (activeRef.current[dir] === on) return;
    const next = { ...activeRef.current, [dir]: on };
    activeRef.current = next;
    setActive(next);

    const t = twistFromActive(next);
    lastTwistRef.current = { x: t.x, y: t.y, yaw: 0 };
    const anyActive = next.up || next.down || next.left || next.right;
    if (anyActive) {
      onTwistRef.current?.(t.x, t.y, 0);
      setWalking(true);
    } else {
      onZeroTwistRef.current?.();
      setWalking(false);
    }
  }

  function clearAll() {
    activeRef.current = { up: false, down: false, left: false, right: false };
    setActive(activeRef.current);
    lastTwistRef.current = { x: 0, y: 0, yaw: 0 };
    onZeroTwistRef.current?.();
    setWalking(false);
  }

  function arrowProps(dir, label, glyph) {
    return {
      type: "button",
      className: `arrow-btn${active[dir] ? " arrow-btn--active" : ""}`,
      disabled: !twistCapable,
      onPointerDown: (e) => {
        if (!twistCapable) return;
        try { e.currentTarget.setPointerCapture(e.pointerId); } catch { /* */ }
        setDir(dir, true);
      },
      onPointerUp: () => setDir(dir, false),
      onPointerCancel: () => setDir(dir, false),
      onContextMenu: (e) => e.preventDefault(),
      "aria-label": label,
      "aria-pressed": active[dir],
      children: glyph,
    };
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
        <div className="walking-panel__arrows">
          <div className="walking-panel__arrows-row">
            <button {...arrowProps("up", "Walk forward", "↑")} />
          </div>
          <div className="walking-panel__arrows-row">
            <button {...arrowProps("left", "Strafe left", "←")} />
            <button {...arrowProps("down", "Walk backward", "↓")} />
            <button {...arrowProps("right", "Strafe right", "→")} />
          </div>
          <p className="muted">Hold ↑ ↓ ← → or W A S D</p>
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
