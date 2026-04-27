// Slide-out settings drawer.
//
// Four sections:
//   1. Per-joint servo inversion toggles (12 switches) → /api/settings/servo_overrides
//   2. Rotate tuneables (increment, rate, calibration factor) → /api/settings/rotate
//   3. Stabilizer params (roll gain, pitch gain, max correction, filter alpha, on/off) → /api/params/stabilizer
//   4. Body-height slider → /api/params/gait/default_z_ref_mm
//
// All edits debounce before POSTing so a slider drag doesn't spam the Pi server.

import { useEffect, useRef, useState } from "react";
import {
  DEFAULT_ROTATE_SETTINGS,
  GAIT_TUNABLE_PARAMS,
  JOINT_NAMES,
  ROTATE_SETTINGS_BOUNDS,
  STABILIZER_PARAM_BOUNDS,
} from "../../shared/robot-config.js";

export default function SettingsDrawer({
  open,
  onClose,
  servoOverrides,
  rotateSettings,
  onSaveServoOverrides,
  onSaveRotateSettings,
  onSaveStabilizerParams,
  onSaveGaitParam,
}) {
  return (
    <aside className={`settings-drawer${open ? " is-open" : ""}`} aria-hidden={!open}>
      <header className="settings-drawer__header">
        <h2>Settings</h2>
        <button type="button" onClick={onClose} aria-label="Close">✕</button>
      </header>
      <div className="settings-drawer__scroll">
        <ServoOverridesSection value={servoOverrides} onSave={onSaveServoOverrides} />
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
