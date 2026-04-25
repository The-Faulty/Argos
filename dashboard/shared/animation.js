// Full-body foot-trajectory clips. Argos extends andy-servo's 2-D keyframes
// to 3-D by adding a `z` channel; legacy 2-D clips parse fine because `z`
// defaults to DEFAULT_Z_REF (the neutral stance height) when missing.
// Leg ids in clips should be FR/FL/RR/RL; a light back-compat shim converts
// andy-servo's front_left/front_right/rear_left/rear_right names.

import { DEFAULT_FULL_BODY_CLIP, LEG_IDS, DEFAULT_Z_REF } from "./robot-config.js";

const LEGACY_LEG_ID_MAP = {
  front_left: "FL",
  front_right: "FR",
  rear_left: "RL",
  rear_right: "RR",
};

function canonicalizeLegId(id) {
  if (LEG_IDS.includes(id)) return id;
  if (id in LEGACY_LEG_ID_MAP) return LEGACY_LEG_ID_MAP[id];
  return null;
}

function normalizeFoot(foot) {
  if (!foot || typeof foot !== "object") return null;
  if (!Number.isFinite(foot.x) || !Number.isFinite(foot.y)) return null;
  return {
    x: foot.x,
    y: foot.y,
    z: Number.isFinite(foot.z) ? foot.z : DEFAULT_Z_REF,
  };
}

function normalizeTrack(track = [], duration) {
  return [...track]
    .map((item) => {
      if (!item || typeof item.time !== "number") return null;
      const foot = normalizeFoot(item.foot);
      if (!foot) return null;
      return { time: Math.max(0, Math.min(duration, item.time)), foot };
    })
    .filter(Boolean)
    .sort((a, b) => a.time - b.time);
}

export function validateClip(candidate) {
  if (!candidate || typeof candidate !== "object") {
    throw new Error("Animation clip must be an object.");
  }

  const duration = Math.max(0.1, Number(candidate.duration) || DEFAULT_FULL_BODY_CLIP.duration);
  const name = typeof candidate.name === "string" && candidate.name ? candidate.name : "argos-animation";

  const rawTracks = candidate.tracks ?? {};
  const tracks = {};
  for (const legId of LEG_IDS) tracks[legId] = [];

  for (const [key, value] of Object.entries(rawTracks)) {
    const canonical = canonicalizeLegId(key);
    if (!canonical) continue;
    tracks[canonical] = normalizeTrack(value, duration);
  }

  return { version: 2, name, duration, tracks };
}

export function interpolateTrack(track, time) {
  if (!track || track.length === 0) return null;
  if (time <= track[0].time) return track[0].foot;
  if (time >= track[track.length - 1].time) return track[track.length - 1].foot;

  for (let i = 0; i < track.length - 1; i += 1) {
    const a = track[i];
    const b = track[i + 1];
    if (time >= a.time && time <= b.time) {
      const span = b.time - a.time || 1;
      const t = (time - a.time) / span;
      return {
        x: a.foot.x + (b.foot.x - a.foot.x) * t,
        y: a.foot.y + (b.foot.y - a.foot.y) * t,
        z: a.foot.z + (b.foot.z - a.foot.z) * t,
      };
    }
  }

  return track[track.length - 1].foot;
}

export function interpolateFullBodyClip(clip, time) {
  const validated = validateClip(clip);
  const frame = {};
  for (const legId of LEG_IDS) {
    frame[legId] = interpolateTrack(validated.tracks[legId], time) ?? { x: 0, y: 0, z: DEFAULT_Z_REF };
  }
  return frame;
}

/**
 * Convert a single-leg clip (andy-servo's old `keyframes: [...]` format)
 * into a full-body clip. `placement.legId` picks where to deposit the track;
 * `placement.mode === "mirrored_pair"` mirrors it across a pair of legs.
 */
export function convertLegacyLegClip(legacyClip, placement = { mode: "selected_leg", legId: "FL" }) {
  if (!legacyClip || typeof legacyClip !== "object") {
    throw new Error("Legacy clip file is invalid.");
  }

  const duration = Math.max(0.1, Number(legacyClip.duration) || 2);
  const keyframes = normalizeTrack(legacyClip.keyframes, duration);
  const tracks = Object.fromEntries(LEG_IDS.map((legId) => [legId, []]));

  if (placement.mode === "mirrored_pair") {
    const pair = placement.pair ?? "front";
    const legIds =
      pair === "rear"  ? ["RL", "RR"] :
      pair === "left"  ? ["FL", "RL"] :
      pair === "right" ? ["FR", "RR"] :
                         ["FL", "FR"];
    for (const legId of legIds) {
      tracks[legId] = keyframes.map((frame) => ({ ...frame, foot: { ...frame.foot } }));
    }
  } else {
    const legId = canonicalizeLegId(placement.legId) ?? "FL";
    tracks[legId] = keyframes.map((frame) => ({ ...frame, foot: { ...frame.foot } }));
  }

  return validateClip({
    version: 2,
    name: legacyClip.name || "converted-leg-clip",
    duration,
    tracks,
  });
}

/** Wire frames used by the upload handshake when POSTing a clip to pi_server. */
export function createUploadFrames(clip) {
  const validated = validateClip(clip);
  const frames = [
    {
      type: "upload_animation",
      stage: "begin",
      name: validated.name,
      duration: validated.duration,
      trackCount: LEG_IDS.length,
    },
  ];

  for (const legId of LEG_IDS) {
    for (const keyframe of validated.tracks[legId]) {
      frames.push({
        type: "upload_animation",
        stage: "frame",
        legId,
        time: keyframe.time,
        x: keyframe.foot.x,
        y: keyframe.foot.y,
        z: keyframe.foot.z,
      });
    }
  }

  frames.push({ type: "upload_animation", stage: "commit", name: validated.name });
  return frames;
}
