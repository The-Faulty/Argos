// MLX90640 thermal grid (24 × 32 px) rendered into a canvas.
//
// The Python sidecar pushes one frame every ~125 ms over the same WS the
// Pi server fans out to the browser. Each frame is `rows × cols` Celsius
// values; we colour-map them with a simple "iron"-ish gradient between the
// frame's own min/max so the gradient stays useful at any ambient temp.

import { useEffect, useRef } from "react";

const SCALE = 8; // px per cell — 32×8 = 256, 24×8 = 192

export default function ThermalPanel({ thermal }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !thermal?.frame) return;
    const { frame, rows = 24, cols = 32, min_c, max_c } = thermal;
    const ctx = canvas.getContext("2d");
    const range = Math.max(0.1, (max_c ?? 1) - (min_c ?? 0));
    const img = ctx.createImageData(cols, rows);
    for (let r = 0; r < rows; r++) {
      const row = frame[r] || [];
      for (let c = 0; c < cols; c++) {
        const t = (row[c] - min_c) / range;
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
  }, [thermal]);

  const min = thermal?.min_c;
  const max = thermal?.max_c;

  return (
    <section className="panel thermal-panel">
      <header className="panel__header">
        <h2>Thermal</h2>
        {!thermal && <span className="pill is-bad">no data</span>}
      </header>
      <canvas
        ref={canvasRef}
        width={32 * SCALE}
        height={24 * SCALE}
        className="thermal-panel__canvas"
      />
      {thermal && (
        <div className="thermal-panel__legend">
          <span>{min?.toFixed(1)} °C</span>
          <span>{max?.toFixed(1)} °C</span>
        </div>
      )}
    </section>
  );
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
