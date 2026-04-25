// CSV recorder controls.
//
// One button toggles start/stop. The Pi server writes to
// ~/argos_recordings/<iso>.csv and exposes the file list; we render that
// so the user can at least see "your recording landed at /home/pi/..."
// from the browser. Actually downloading a file over HTTP would need a
// `/api/recording/download/<name>` route on the server — keep that for a
// later PR.

import { useEffect, useState } from "react";

export default function TelemetryRecorder({ recording, onStart, onStop, onList }) {
  const [files, setFiles] = useState([]);

  async function refresh() {
    try { const res = await onList?.(); setFiles(res?.files ?? []); } catch {}
  }

  useEffect(() => { refresh(); /* on mount */ }, []);
  useEffect(() => { if (!recording?.active) refresh(); }, [recording?.active]);

  return (
    <section className="telemetry-recorder">
      <header><h3>Recorder</h3></header>
      <div className="telemetry-recorder__row">
        {!recording?.active ? (
          <button type="button" className="record-btn" onClick={onStart}>● Start</button>
        ) : (
          <button type="button" className="record-btn is-active" onClick={onStop}>■ Stop</button>
        )}
        <span className="muted">
          {recording?.active
            ? `rows: ${recording.rowCount ?? 0}`
            : files.length ? `${files.length} past files` : "idle"}
        </span>
      </div>
      {!recording?.active && files.length > 0 && (
        <ul className="telemetry-recorder__files">
          {files.slice(-5).reverse().map((path) => (
            <li key={path} title={path}>{path.split(/[\\/]/).pop()}</li>
          ))}
        </ul>
      )}
    </section>
  );
}
