// Gas sensor panel.
//
// ESP32 publishes MQ-131 ozone telemetry. `gas.data` remains the primary
// O3 ppb value for back-compat; richer firmware also includes ppm, mg/m3,
// ug/m3, and raw ADC counts. We show the live PPB value + a user-set alert
// threshold. When the value exceeds the threshold we flip the panel into a
// red-warning state AND play a short audio beep (if the user has clicked
// the page at least once — browsers block autoplay otherwise).

import { useEffect, useRef, useState } from "react";

// O3 PPB threshold. EPA NAAQS 8-h average is 70 ppb; 100 ppb is a
// reasonable "noticeable" alert threshold for a workshop bot.
const DEFAULT_THRESHOLD = 100;
const STORAGE_KEY = "argos.gasThreshold";

export default function GasPanel({ gas }) {
  const [threshold, setThreshold] = useState(() => readThreshold());
  const [audioUnlocked, setAudioUnlocked] = useState(false);
  const lastAlertAtRef = useRef(0);

  useEffect(() => {
    try { window.localStorage.setItem(STORAGE_KEY, String(threshold)); } catch {}
  }, [threshold]);

  const value = Number(gas?.data);
  const ppm = Number(gas?.ppm);
  const mgM3 = Number(gas?.mg_m3);
  const ugM3 = Number(gas?.ug_m3);
  const raw = Number(gas?.raw);
  const isAlerting = Number.isFinite(value) && value >= threshold;

  useEffect(() => {
    if (!isAlerting || !audioUnlocked) return;
    const now = Date.now();
    if (now - lastAlertAtRef.current < 2000) return;
    lastAlertAtRef.current = now;
    try {
      const ctx = new (window.AudioContext || window.webkitAudioContext)();
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = "square";
      osc.frequency.value = 880;
      gain.gain.value = 0.05;
      osc.connect(gain).connect(ctx.destination);
      osc.start();
      osc.stop(ctx.currentTime + 0.12);
    } catch { /* ignore */ }
  }, [isAlerting, audioUnlocked]);

  return (
    <section className={`gas-panel${isAlerting ? " is-alert" : ""}`}
             onPointerDown={() => setAudioUnlocked(true)}>
      <header>
        <h3>Ozone</h3>
        <span className="muted">MQ-131 (ppb O₃)</span>
      </header>
      <div className="gas-panel__value">
        {Number.isFinite(value) ? value.toFixed(1) : "—"}
        <span className="gas-panel__unit"> ppb</span>
      </div>
      <dl className="gas-panel__metrics">
        <div>
          <dt>ppm</dt>
          <dd>{fmt(ppm, 4)}</dd>
        </div>
        <div>
          <dt>mg/m3</dt>
          <dd>{fmt(mgM3, 4)}</dd>
        </div>
        <div>
          <dt>ug/m3</dt>
          <dd>{fmt(ugM3, 1)}</dd>
        </div>
        <div>
          <dt>raw</dt>
          <dd>{Number.isFinite(raw) ? raw.toFixed(0) : "--"}</dd>
        </div>
      </dl>

      <label className="slider">
        <span>Alert threshold (ppb)</span>
        <input
          type="range"
          min={0} max={1000} step={5}
          value={threshold}
          onChange={(e) => setThreshold(Number(e.target.value))}
        />
        <span className="slider__value">{threshold}</span>
      </label>

      {isAlerting && <p className="gas-panel__alert">⚠ O₃ threshold exceeded</p>}
    </section>
  );
}

function fmt(value, digits) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function readThreshold() {
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    const n = raw === null ? DEFAULT_THRESHOLD : Number(raw);
    return Number.isFinite(n) ? n : DEFAULT_THRESHOLD;
  } catch {
    return DEFAULT_THRESHOLD;
  }
}
