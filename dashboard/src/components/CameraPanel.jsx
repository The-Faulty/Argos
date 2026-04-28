// Live RealSense color/depth stream.
//
// The Pi server proxies the Python sidecar's MJPEG endpoint at
// `/api/camera/stream` and `/api/camera/depth-stream`. We render a single
// <img> against the selected URL; the browser handles the multipart frame
// sequence natively. On error we
// auto-retry with a cache-busting query param so the stream comes back
// once the sidecar reconnects.

import { useEffect, useRef, useState } from "react";

const STREAM_URL = "/api/camera/stream";
const DEPTH_STREAM_URL = "/api/camera/depth-stream";

export default function CameraPanel({ depth }) {
  const imgRef = useRef(null);
  const [retries, setRetries] = useState(0);
  const [error, setError] = useState(null);
  const [view, setView] = useState("color");

  useEffect(() => {
    if (!error) return;
    const t = setTimeout(() => setRetries((n) => n + 1), 2000);
    return () => clearTimeout(t);
  }, [error]);

  const streamUrl = view === "depth" ? DEPTH_STREAM_URL : STREAM_URL;
  const src = `${streamUrl}?r=${retries}`;
  return (
    <section className="panel camera-panel">
      <header className="panel__header">
        <h2>RealSense</h2>
        {error && <span className="pill is-bad">no stream</span>}
      </header>
      <div className="camera-panel__tabs" aria-label="Camera stream">
        <button
          type="button"
          className={view === "color" ? "is-active" : ""}
          onClick={() => {
            setView("color");
            setError(null);
          }}
        >
          Color
        </button>
        <button
          type="button"
          className={view === "depth" ? "is-active" : ""}
          onClick={() => {
            setView("depth");
            setError(null);
          }}
        >
          Depth
        </button>
      </div>
      <div className="camera-panel__viewport">
        <img
          ref={imgRef}
          src={src}
          alt={`RealSense ${view} stream`}
          onLoad={() => setError(null)}
          onError={() => setError("stream-unavailable")}
        />
        {view === "depth" && <span className="camera-panel__crosshair" aria-hidden="true" />}
      </div>
      <DepthReadout depth={depth} />
    </section>
  );
}

function DepthReadout({ depth }) {
  const center = depth?.center?.m;
  const roiMean = depth?.roi?.mean_m;
  const nearest = depth?.nearest_m;
  const validPct = depth?.valid_pct;
  return (
    <dl className="camera-panel__measurements">
      <div>
        <dt>center</dt>
        <dd>{meters(center)}</dd>
      </div>
      <div>
        <dt>roi avg</dt>
        <dd>{meters(roiMean)}</dd>
      </div>
      <div>
        <dt>nearest</dt>
        <dd>{meters(nearest)}</dd>
      </div>
      <div>
        <dt>valid</dt>
        <dd>{Number.isFinite(validPct) ? `${validPct.toFixed(0)}%` : "--"}</dd>
      </div>
    </dl>
  );
}

function meters(value) {
  return Number.isFinite(value) ? `${value.toFixed(2)} m` : "--";
}
