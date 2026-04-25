// Animation clip player.
//
// Three surfaces:
//   1. Clip library — locally-stored JSON clips (uploaded via file input).
//      Held in memory + localStorage; not persisted to the Pi yet. The
//      upload handshake to pi_server is wired through `createUploadFrames`
//      so the format is ready when the bridge node grows playback support.
//   2. Play / stop — posts `animation_play {name}` / `animation_stop` to /api/command.
//   3. Scrub — sends the interpolated-at-time frame as a foot_targets
//      command. Useful for previewing a clip without the bridge running it.

import { useMemo, useState } from "react";
import {
  convertLegacyLegClip,
  interpolateFullBodyClip,
  validateClip,
} from "../../shared/animation.js";
import { LEG_IDS } from "../../shared/robot-config.js";

const LIB_KEY = "argos.clipLibrary";

export default function AnimationPlayer({ onPlay, onStop, onSendFootTargets, enabled }) {
  const [library, setLibrary] = useState(() => readLibrary());
  const [selected, setSelected] = useState(null);
  const [scrub, setScrub] = useState(0);

  function persist(next) {
    setLibrary(next);
    try { window.localStorage.setItem(LIB_KEY, JSON.stringify(next)); } catch {}
  }

  function addClip(rawJson) {
    try {
      const parsed = JSON.parse(rawJson);
      const clip = parsed.tracks ? validateClip(parsed) : convertLegacyLegClip(parsed);
      const name = clip.name || `clip-${Date.now()}`;
      persist({ ...library, [name]: clip });
    } catch (err) {
      alert(`Clip rejected: ${err.message}`);
    }
  }

  function removeClip(name) {
    const next = { ...library };
    delete next[name];
    persist(next);
    if (selected === name) setSelected(null);
  }

  const clip = selected ? library[selected] : null;

  const scrubFrame = useMemo(() => {
    if (!clip) return null;
    return interpolateFullBodyClip(clip, scrub);
  }, [clip, scrub]);

  function onScrubChange(value) {
    setScrub(value);
    if (!clip || !enabled) return;
    const frame = interpolateFullBodyClip(clip, value);
    const matrix = LEG_IDS.map((leg) => [frame[leg].x, frame[leg].y, frame[leg].z]);
    onSendFootTargets?.(matrix);
  }

  return (
    <section className="animation-player">
      <header><h3>Animations</h3></header>

      <div className="animation-player__lib">
        <select value={selected ?? ""} onChange={(e) => setSelected(e.target.value || null)}>
          <option value="">— select clip —</option>
          {Object.keys(library).map((name) => <option key={name} value={name}>{name}</option>)}
        </select>
        <label className="file-pick">
          Upload
          <input type="file" accept="application/json"
                 onChange={async (e) => {
                   const file = e.target.files?.[0];
                   if (!file) return;
                   addClip(await file.text());
                   e.target.value = "";
                 }} />
        </label>
        {selected && <button type="button" onClick={() => removeClip(selected)}>Remove</button>}
      </div>

      {clip && (
        <>
          <div className="animation-player__transport">
            <button type="button" onClick={() => onPlay?.(selected)} disabled={!enabled}>▶ Play</button>
            <button type="button" onClick={() => onStop?.()} disabled={!enabled}>■ Stop</button>
          </div>
          <label className="slider">
            <span>Scrub (s)</span>
            <input type="range" min={0} max={clip.duration} step={0.01}
                   value={scrub} onChange={(e) => onScrubChange(Number(e.target.value))}
                   disabled={!enabled} />
            <span className="slider__value">{scrub.toFixed(2)} / {clip.duration.toFixed(2)}</span>
          </label>
          {scrubFrame && (
            <div className="animation-player__frame muted">
              {LEG_IDS.map((leg) => (
                <span key={leg}>{leg}: ({scrubFrame[leg].x.toFixed(2)}, {scrubFrame[leg].y.toFixed(2)}, {scrubFrame[leg].z.toFixed(2)})</span>
              ))}
            </div>
          )}
        </>
      )}
    </section>
  );
}

function readLibrary() {
  try {
    const raw = window.localStorage.getItem(LIB_KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw);
    if (!parsed || typeof parsed !== "object") return {};
    // Re-validate so a hand-edited localStorage entry can't crash the UI.
    const out = {};
    for (const [k, v] of Object.entries(parsed)) {
      try { out[k] = validateClip(v); } catch { /* skip bad */ }
    }
    return out;
  } catch {
    return {};
  }
}
