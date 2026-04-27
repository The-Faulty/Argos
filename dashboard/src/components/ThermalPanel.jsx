// MLX90640 thermal grid (24 × 32 px) rendered into a canvas.
//
// The Python sidecar pushes one frame every ~125 ms over the same WS the
// Pi server fans out to the browser. Each frame is `rows × cols` Celsius
// values; we colour-map them with a simple "iron"-ish gradient between the
// frame's own min/max so the gradient stays useful at any ambient temp.

import { useEffect, useMemo, useRef, useState } from "react";

const SCALE = 8; // px per cell — 32×8 = 256, 24×8 = 192
const HEALTH_POLL_MS = 3000;

export default function ThermalPanel({ thermal }) {
  const canvasRef = useRef(null);
  const [health, setHealth] = useState(null);

  const normalized = useMemo(() => normalizeThermalFrame(thermal), [thermal]);

  useEffect(() => {
    let cancelled = false;
    let timer = null;

    async function pollHealth() {
      try {
        const res = await fetch("/api/sensors/health", { cache: "no-store" });
        const next = await res.json();
        if (!cancelled) setHealth(next);
      } catch (err) {
        if (!cancelled) {
          setHealth({
            ok: false,
            errors: { thermal: String(err.message || err) },
          });
        }
      } finally {
        if (!cancelled) timer = setTimeout(pollHealth, HEALTH_POLL_MS);
      }
    }

    pollHealth();
    return () => {
      cancelled = true;
      if (timer) clearTimeout(timer);
    };
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!normalized) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      return;
    }
    const { frame, rows, cols, min, max } = normalized;
    const range = max - min;
    const img = ctx.createImageData(cols, rows);
    for (let r = 0; r < rows; r++) {
      const row = frame[r] || [];
      for (let c = 0; c < cols; c++) {
        const value = row[c];
        const t = range > 0.1 ? (value - min) / range : 0.5;
        const [R, G, B] = ironColor(Math.max(0, Math.min(1, t)));
        const idx = (r * cols + c) * 4;
        img.data[idx + 0] = R;
        img.data[idx + 1] = G;
        img.data[idx + 2] = B;
        img.data[idx + 3] = 255;
      }
    }
    // Render at native resolution then upscale via canvas size.
    const tmp = document.createElement("canvas");
    tmp.width = cols;
    tmp.height = rows;
    tmp.getContext("2d").putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(tmp, 0, 0, canvas.width, canvas.height);
  }, [normalized]);

  const thermalError = health?.errors?.thermal || health?.error;
  const thermalOffline = health && health.thermal_running === false;
  const sidecarOffline = health?.ok === false && Boolean(health?.error);
  const status = normalized
    ? "live"
    : thermalError
      ? sidecarOffline ? "sidecar offline" : "sensor error"
      : thermalOffline
        ? "thermal offline"
        : "no data";

  return (
    <section className="panel thermal-panel">
      <header className="panel__header">
        <h2>Thermal</h2>
        <span className={`pill${normalized ? " is-ok" : " is-bad"}`}>{status}</span>
      </header>
      <div className="thermal-panel__viewport">
        <canvas
          ref={canvasRef}
          width={32 * SCALE}
          height={24 * SCALE}
          className="thermal-panel__canvas"
        />
        {!normalized && (
          <div className="thermal-panel__empty">
            <strong>
              {sidecarOffline
                ? "Sensor sidecar is not reachable"
                : thermalOffline
                  ? "MLX90640 not streaming"
                  : "Waiting for thermal frame"}
            </strong>
            {thermalError && <span>{thermalError}</span>}
          </div>
        )}
      </div>
      {normalized && (
        <div className="thermal-panel__legend">
          <span>{normalized.min.toFixed(1)} °C</span>
          <span>{normalized.max.toFixed(1)} °C</span>
        </div>
      )}
    </section>
  );
}

function normalizeThermalFrame(thermal) {
  if (!thermal || !Array.isArray(thermal.frame)) return null;
  const rows = Number.isFinite(thermal.rows) ? thermal.rows : thermal.frame.length;
  const cols = Number.isFinite(thermal.cols)
    ? thermal.cols
    : Array.isArray(thermal.frame[0])
      ? thermal.frame[0].length
      : 0;
  if (rows <= 0 || cols <= 0) return null;

  const frame = [];
  let min = Number.isFinite(thermal.min_c) ? thermal.min_c : Infinity;
  let max = Number.isFinite(thermal.max_c) ? thermal.max_c : -Infinity;
  let finiteCount = 0;

  for (let r = 0; r < rows; r++) {
    const srcRow = Array.isArray(thermal.frame[r]) ? thermal.frame[r] : [];
    const row = [];
    for (let c = 0; c < cols; c++) {
      const value = Number(srcRow[c]);
      if (Number.isFinite(value)) {
        row.push(value);
        min = Math.min(min, value);
        max = Math.max(max, value);
        finiteCount++;
      } else {
        row.push(null);
      }
    }
    frame.push(row);
  }

  if (!finiteCount || !Number.isFinite(min) || !Number.isFinite(max)) return null;
  for (const row of frame) {
    for (let i = 0; i < row.length; i++) {
      if (row[i] === null) row[i] = min;
    }
  }
  return { frame, rows, cols, min, max };
}

// Min→max gradient: black → red → yellow → white.
function ironColor(t) {
  if (t < 0.33) {
    const u = t / 0.33;
    return [Math.round(255 * u), 0, 0];
  }
  if (t < 0.66) {
    const u = (t - 0.33) / 0.33;
    return [255, Math.round(255 * u), 0];
  }
  const u = (t - 0.66) / 0.34;
  return [255, 255, Math.round(255 * u)];
}
