// Live RealSense color stream.
//
// The Pi server proxies the Python sidecar's MJPEG endpoint at
// `/api/camera/stream`. We render a single <img> against that URL — the
// browser handles the multipart frame sequence natively. On error we
// auto-retry with a cache-busting query param so the stream comes back
// once the sidecar reconnects.

import { useEffect, useRef, useState } from "react";

const STREAM_URL = "/api/camera/stream";

export default function CameraPanel() {
  const imgRef = useRef(null);
  const [retries, setRetries] = useState(0);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (!error) return;
    const t = setTimeout(() => setRetries((n) => n + 1), 2000);
    return () => clearTimeout(t);
  }, [error]);

  const src = `${STREAM_URL}?r=${retries}`;
  return (
    <section className="panel camera-panel">
      <header className="panel__header">
        <h2>Camera</h2>
        {error && <span className="pill is-bad">no stream</span>}
      </header>
      <div className="camera-panel__viewport">
        <img
          ref={imgRef}
          src={src}
          alt="RealSense color stream"
          onLoad={() => setError(null)}
          onError={() => setError("stream-unavailable")}
        />
      </div>
    </section>
  );
}
