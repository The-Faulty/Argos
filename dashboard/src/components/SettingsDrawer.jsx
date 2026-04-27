// Slide-out settings drawer.
//
// Six sections:
//   1. Per-joint servo inversion toggles (12 switches) → /api/settings/servo_overrides
//   2. Per-joint min/max number inputs (12 × 2) → /api/settings/joint_limits
//   3. Servo speed limits (per-row deg/s) + PWM update rate (Hz) → /api/settings/servo_speed, /api/settings/servo_update_rate
//   4. Rotate tuneables (increment, rate, calibration factor) → /api/settings/rotate
//   5. Stabilizer params (roll gain, pitch gain, max correction, filter alpha, on/off) → /api/params/stabilizer
//   6. Body-height slider → /api/params/gait/default_z_ref_mm
//
// All edits debounce before POSTing so a slider drag doesn't spam the Pi server.

import { useEffect, useMemo, useRef, useState } from "react";
import {
  COUPLED_BOT_MAX_SAMPLES_DEG,
  COUPLED_BOT_MIN_SAMPLES_DEG,
  COUPLED_TOP_SAMPLES_DEG,
  DEFAULT_JOINT_LIMITS_RAD,
  DEFAULT_ROTATE_SETTINGS,
  DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC,
  DEFAULT_SERVO_UPDATE_RATE_HZ,
  GAIT_TUNABLE_PARAMS,
  JOINT_NAMES,
  JOINT_ROWS,
  LEG_IDS,
  ROTATE_SETTINGS_BOUNDS,
  SERVO_SPEED_LIMIT_BOUNDS_DEG_PER_SEC,
  SERVO_CENTER_DEG,
  SERVO_UPDATE_RATE_BOUNDS_HZ,
  STABILIZER_PARAM_BOUNDS,
} from "../../shared/robot-config.js";

const RAD2DEG = 180.0 / Math.PI;

export default function SettingsDrawer({
  open,
  onClose,
  servoOverrides,
  jointLimits,
  rotateSettings,
  servoSpeed,
  servoUpdateRate,
  onSaveServoOverrides,
  onSaveJointLimits,
  onSaveRotateSettings,
  onSaveStabilizerParams,
  onSaveGaitParam,
  onSaveServoSpeed,
  onSaveServoUpdateRate,
}) {
  return (
    <aside className={`settings-drawer${open ? " is-open" : ""}`} aria-hidden={!open}>
      <header className="settings-drawer__header">
        <h2>Settings</h2>
        <button type="button" onClick={onClose} aria-label="Close">✕</button>
      </header>
      <div className="settings-drawer__scroll">
        <ServoOverridesSection value={servoOverrides} onSave={onSaveServoOverrides} />
        <JointLimitsSection value={jointLimits} onSave={onSaveJointLimits} />
        <ServoRateSection
          speed={servoSpeed}
          updateRate={servoUpdateRate}
          onSaveSpeed={onSaveServoSpeed}
          onSaveUpdateRate={onSaveServoUpdateRate}
        />
        <RotateSettingsSection value={rotateSettings} onSave={onSaveRotateSettings} />
        <StabilizerSection onSave={onSaveStabilizerParams} />
        <BodyHeightSection onSave={(v) => onSaveGaitParam?.("default_z_ref_mm", v)} />
      </div>
    </aside>
  );
}

// ─── Section: per-joint invert toggles ──────────────────────────────────

function ServoOverridesSection({ value, onSave }) {
  const [draft, setDraft] = useState(value || {});
  useEffect(() => setDraft(value || {}), [value]);

  function toggle(jointName) {
    const cur = draft[jointName] || {};
    const next = { ...draft };
    const newInvert = !cur.invert;
    if (!newInvert && !cur.offset_deg) {
      delete next[jointName];
    } else {
      next[jointName] = { ...cur, invert: newInvert, offset_deg: cur.offset_deg || 0 };
    }
    setDraft(next);
    onSave?.(next);
  }

  function setOffset(jointName, offset_deg) {
    const cur = draft[jointName] || {};
    const next = { ...draft };
    if (!offset_deg && !cur.invert) {
      delete next[jointName];
    } else {
      next[jointName] = { ...cur, offset_deg };
    }
    setDraft(next);
    onSave?.(next);
  }

  return (
    <Section title="Servo overrides">
      <p className="muted">Flip direction + fine-tune offset_deg per joint. Firmware defaults apply when a row is blank.</p>
      <table className="settings-table">
        <thead>
          <tr><th>Joint</th><th>Invert</th><th>Offset (°)</th></tr>
        </thead>
        <tbody>
          {JOINT_NAMES.map((n) => {
            const cur = draft[n] || {};
            return (
              <tr key={n}>
                <th scope="row">{n}</th>
                <td>
                  <input type="checkbox" checked={!!cur.invert} onChange={() => toggle(n)} />
                </td>
                <td>
                  <input
                    type="number" step="0.5" value={cur.offset_deg || 0}
                    onChange={(e) => setOffset(n, Number(e.target.value) || 0)}
                    style={{ width: 70 }}
                  />
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </Section>
  );
}

// ─── Section: per-joint min/max limits ──────────────────────────────────

function JointLimitsSection({ value, onSave }) {
  const initial = useMemo(() => computeInitial(value), [value]);
  const [draft, setDraft] = useState(initial);
  useEffect(() => setDraft(initial), [initial]);

  function setLimit(jointName, kind, deg) {
    const cur = draft[jointName] || { ...initial[jointName] };
    const next = { ...draft, [jointName]: { ...cur, [kind]: deg } };
    setDraft(next);
    // Drop entries matching defaults so the persisted file stays small.
    const outgoing = {};
    for (const [n, v] of Object.entries(next)) {
      const d = defaultDegFor(n);
      if (v.min_deg === d.min_deg && v.max_deg === d.max_deg) continue;
      outgoing[n] = v;
    }
    onSave?.(outgoing);
  }

  function resetAll() {
    const clean = {};
    setDraft(computeInitial(clean));
    onSave?.(clean);
  }

  return (
    <Section title="Joint limits (per leg)">
      <p className="muted">Measured defaults in degrees; enter decimals for tight tuning.</p>
      <button type="button" className="ghost-btn" onClick={resetAll}>
        Reset limits to measured defaults
      </button>
      <div className="settings-grid-legs">
        {LEG_IDS.map((leg) => (
          <div key={leg}>
            <h4>{leg}</h4>
            <table className="settings-table">
              <thead><tr><th>Joint</th><th>min°</th><th>max°</th></tr></thead>
              <tbody>
                {JOINT_ROWS.map((row) => {
                  const jointName = `${leg}_${row}_joint`;
                  const cur = draft[jointName] || defaultDegFor(jointName);
                  return (
                    <tr key={row}>
                      <th scope="row">{row}</th>
                      <td><input type="number" step="1" value={cur.min_deg}
                                 onChange={(e) => setLimit(jointName, "min_deg", Number(e.target.value))}
                                 style={{ width: 70 }} /></td>
                      <td><input type="number" step="1" value={cur.max_deg}
                                 onChange={(e) => setLimit(jointName, "max_deg", Number(e.target.value))}
                                 style={{ width: 70 }} /></td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
        ))}
      </div>
      <h4>Coupled femur/tibia envelope</h4>
      <table className="settings-table">
        <thead><tr><th>Femur°</th><th>Tibia min°</th><th>Tibia max°</th></tr></thead>
        <tbody>
          {COUPLED_TOP_SAMPLES_DEG.map((topServoDeg, i) => (
            <tr key={topServoDeg}>
              <th scope="row">{fmtDeg(topServoDeg - SERVO_CENTER_DEG)}</th>
              <td>{fmtDeg(COUPLED_BOT_MIN_SAMPLES_DEG[i] - SERVO_CENTER_DEG)}</td>
              <td>{fmtDeg(COUPLED_BOT_MAX_SAMPLES_DEG[i] - SERVO_CENTER_DEG)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </Section>
  );
}

function defaultDegFor(jointName) {
  const row = jointName.split("_")[1];
  const [lo, hi] = DEFAULT_JOINT_LIMITS_RAD[row];
  return { min_deg: lo * RAD2DEG, max_deg: hi * RAD2DEG };
}

function computeInitial(value) {
  const out = {};
  for (const n of JOINT_NAMES) {
    out[n] = value?.[n] ? { ...value[n] } : defaultDegFor(n);
  }
  return out;
}

function fmtDeg(value) {
  return Number(value).toFixed(0);
}

// ─── Section: rotate tuneables ──────────────────────────────────────────

function RotateSettingsSection({ value, onSave }) {
  const [draft, setDraft] = useState(value || DEFAULT_ROTATE_SETTINGS);
  useEffect(() => setDraft(value || DEFAULT_ROTATE_SETTINGS), [value]);

  function setField(key, v) {
    const next = { ...draft, [key]: v };
    setDraft(next);
    onSave?.(next);
  }

  const b = ROTATE_SETTINGS_BOUNDS;

  return (
    <Section title="Rotate buttons">
      <p className="muted">
        Click-to-rotate by degrees. `rate_rad_s` is the angular velocity the
        Pi commands; `calibration_factor` scales the spin duration to
        compensate for drive-train slippage.
      </p>
      <div className="settings-row">
        <label>
          <span>Increment per click (°)</span>
          <input type="number" step="1"
            min={b.rotate_increment_deg.min} max={b.rotate_increment_deg.max}
            value={draft.rotate_increment_deg ?? DEFAULT_ROTATE_SETTINGS.rotate_increment_deg}
            onChange={(e) => setField("rotate_increment_deg", Number(e.target.value))} />
        </label>
        <label>
          <span>Rate (rad/s)</span>
          <input type="number" step="0.05"
            min={b.rotate_rate_rad_s.min} max={b.rotate_rate_rad_s.max}
            value={draft.rotate_rate_rad_s ?? DEFAULT_ROTATE_SETTINGS.rotate_rate_rad_s}
            onChange={(e) => setField("rotate_rate_rad_s", Number(e.target.value))} />
        </label>
        <label>
          <span>Calibration factor</span>
          <input type="number" step="0.01"
            min={b.rotate_calibration_factor.min} max={b.rotate_calibration_factor.max}
            value={draft.rotate_calibration_factor ?? DEFAULT_ROTATE_SETTINGS.rotate_calibration_factor}
            onChange={(e) => setField("rotate_calibration_factor", Number(e.target.value))} />
        </label>
      </div>
    </Section>
  );
}

// ─── Section: stabilizer ────────────────────────────────────────────────

function StabilizerSection({ onSave }) {
  const b = STABILIZER_PARAM_BOUNDS;
  const [draft, setDraft] = useState(() => ({
    stabilization_roll_gain: b.stabilization_roll_gain.default,
    stabilization_pitch_gain: b.stabilization_pitch_gain.default,
    stabilization_max_correction_rad: b.stabilization_max_correction_rad.default,
    imu_filter_alpha: b.imu_filter_alpha.default,
    use_imu_stabilization: true,
  }));
  const timer = useRef(null);

  function setField(key, v) {
    const next = { ...draft, [key]: v };
    setDraft(next);
    // Debounce POST so dragging a slider doesn't flood rosbridge.
    if (timer.current) clearTimeout(timer.current);
    timer.current = setTimeout(() => onSave?.({ [key]: v }), 150);
  }

  return (
    <Section title="Stabilizer (IMU)">
      <label className="checkbox">
        <input type="checkbox" checked={draft.use_imu_stabilization}
               onChange={(e) => setField("use_imu_stabilization", e.target.checked)} />
        <span>Active IMU stabilization</span>
      </label>
      <RangeField label="Roll gain" bounds={b.stabilization_roll_gain}
                  value={draft.stabilization_roll_gain} step={0.01}
                  onChange={(v) => setField("stabilization_roll_gain", v)} />
      <RangeField label="Pitch gain" bounds={b.stabilization_pitch_gain}
                  value={draft.stabilization_pitch_gain} step={0.01}
                  onChange={(v) => setField("stabilization_pitch_gain", v)} />
      <RangeField label="Max correction (rad)" bounds={b.stabilization_max_correction_rad}
                  value={draft.stabilization_max_correction_rad} step={0.005}
                  onChange={(v) => setField("stabilization_max_correction_rad", v)} />
      <RangeField label="IMU filter α" bounds={b.imu_filter_alpha}
                  value={draft.imu_filter_alpha} step={0.01}
                  onChange={(v) => setField("imu_filter_alpha", v)} />
    </Section>
  );
}

// ─── Section: servo speed + PWM update rate ─────────────────────────────
//
// Two knobs that together shape how the servos execute commands:
//   - Per-joint-row speed cap (deg/s): stops the bridge from commanding
//     travel faster than the bell-crank tolerates. Coxa is light → fine at
//     180 °/s; tibia carries the calf + IMU + wiring so it ships at 300 °/s.
//   - PWM / command update rate (Hz): 50 Hz is standard hobby-servo native;
//     bumping to 100+ smooths motion if the firmware's PCA9685 can keep up.
// Both POST in real time (no debounce) — these values change rarely and the
// server persists them, so a stray mid-drag write is harmless.
function ServoRateSection({ speed, updateRate, onSaveSpeed, onSaveUpdateRate }) {
  const speedBounds = SERVO_SPEED_LIMIT_BOUNDS_DEG_PER_SEC;
  const rateBounds = SERVO_UPDATE_RATE_BOUNDS_HZ;

  const [speedDraft, setSpeedDraft] = useState(
    speed && typeof speed === "object" && Object.keys(speed).length > 0
      ? { ...DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC, ...speed }
      : { ...DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC },
  );
  const [hzDraft, setHzDraft] = useState(
    Number.isFinite(updateRate?.hz) && updateRate.hz > 0
      ? updateRate.hz
      : DEFAULT_SERVO_UPDATE_RATE_HZ,
  );

  // Pull in server-side updates whenever the persisted values change. The
  // empty-object sentinel from useRosbridge means "not yet loaded from Pi",
  // so we don't stomp the draft with defaults in that case.
  useEffect(() => {
    if (speed && typeof speed === "object" && Object.keys(speed).length > 0) {
      setSpeedDraft((prev) => ({ ...prev, ...speed }));
    }
  }, [speed]);
  useEffect(() => {
    if (Number.isFinite(updateRate?.hz) && updateRate.hz > 0) {
      setHzDraft(updateRate.hz);
    }
  }, [updateRate]);

  function setSpeedFor(row, v) {
    const next = { ...speedDraft, [row]: v };
    setSpeedDraft(next);
    onSaveSpeed?.(next);
  }

  function setHz(v) {
    setHzDraft(v);
    onSaveUpdateRate?.(v);
  }

  return (
    <Section title="Servo speed + PWM rate">
      <p className="muted">
        Per-joint-row slew cap in deg/s (applied in the bridge before
        /joint_command/raw). Bottom slider sets how often the Pi re-publishes
        commands and the firmware refreshes the PCA9685 PWM.
      </p>
      <div className="settings-row">
        {JOINT_ROWS.map((row) => (
          <label key={row} className="slider">
            <span>{row} speed (°/s)</span>
            <input
              type="range"
              min={speedBounds.min}
              max={speedBounds.max}
              step="10"
              value={speedDraft[row] ?? DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC[row]}
              onChange={(e) => setSpeedFor(row, Number(e.target.value))}
            />
            <span className="slider__value">
              {Math.round(speedDraft[row] ?? DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC[row])} °/s
            </span>
          </label>
        ))}
      </div>
      <label className="slider">
        <span>PWM / command update rate (Hz)</span>
        <input
          type="range"
          min={rateBounds.min}
          max={rateBounds.max}
          step="5"
          value={hzDraft}
          onChange={(e) => setHz(Number(e.target.value))}
        />
        <span className="slider__value">{Math.round(hzDraft)} Hz</span>
      </label>
    </Section>
  );
}

// ─── Section: body height ───────────────────────────────────────────────

function BodyHeightSection({ onSave }) {
  const cfg = GAIT_TUNABLE_PARAMS.default_z_ref_mm;
  const [value, setValue] = useState(cfg.default);
  const timer = useRef(null);
  function setField(v) {
    setValue(v);
    if (timer.current) clearTimeout(timer.current);
    timer.current = setTimeout(() => onSave?.(v), 150);
  }
  return (
    <Section title="Body height">
      <label className="slider">
        <span>default_z_ref_mm</span>
        <input type="range" min={cfg.min} max={cfg.max} step="1"
               value={value} onChange={(e) => setField(Number(e.target.value))} />
        <span className="slider__value">{value} mm</span>
      </label>
    </Section>
  );
}

// ─── Generic helpers ────────────────────────────────────────────────────

function Section({ title, children }) {
  return (
    <section className="settings-section">
      <h3>{title}</h3>
      {children}
    </section>
  );
}

function RangeField({ label, bounds, value, step, onChange }) {
  return (
    <label className="slider">
      <span>{label}</span>
      <input type="range" min={bounds.min} max={bounds.max} step={step}
             value={value} onChange={(e) => onChange(Number(e.target.value))} />
      <span className="slider__value">{Number(value).toFixed(3)}</span>
    </label>
  );
}
