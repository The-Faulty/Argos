// Gas sensor panel.
//
// ESP32 publishes a Float32 on /gas. We show the live value + a user-set
// alert threshold. When the value exceeds the threshold we flip the panel
// into a red-warning state AND play a short audio beep (if the user has
// clicked the page at least once — browsers block autoplay otherwise).

import { useEffect, useRef, useState } from "react";

const DEFAULT_THRESHOLD = 500;
const STORAGE_KEY = "argos.gasThreshold";

export default function GasPanel({ gas }) {
  const [threshold, setThreshold] = useState(() => readThreshold());
  const [audioUnlocked, setAudioUnlocked] = useState(false);
  const lastAlertAtRef = useRef(0);

  useEffect(() => {
    try { window.localStorage.setItem(STORAGE_KEY, String(threshold)); } catch {}
  }, [threshold]);

  const value = Number(gas?.data);
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
        <h3>Gas</h3>
        <span className="muted">/gas (ppm)</span>
      </header>
      <div className="gas-panel__value">
        {Number.isFinite(value) ? value.toFixed(0) : "—"}
      </div>

      <label className="slider">
        <span>Alert threshold</span>
        <input
          type="range"
          min={0} max={2000} step={10}
          value={threshold}
          onChange={(e) => setThreshold(Number(e.target.value))}
        />
        <span className="slider__value">{threshold}</span>
      </label>

      {isAlerting && <p className="gas-panel__alert">⚠ Gas threshold exceeded</p>}
    </section>
  );
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
