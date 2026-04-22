import { DEFAULT_FULL_BODY_CLIP, LEG_IDS } from "./robot-config.js";

function normalizeTrack(track = [], duration) {
  return [...track]
    .filter((item) => item && typeof item.time === "number" && item.foot && Number.isFinite(item.foot.x) && Number.isFinite(item.foot.y))
    .map((item) => ({
      time: Math.max(0, Math.min(duration, item.time)),
      foot: { x: item.foot.x, y: item.foot.y }
    }))
    .sort((a, b) => a.time - b.time);
}

export function validateClip(candidate) {
  if (!candidate || typeof candidate !== "object") {
    throw new Error("Animation clip must be an object.");
  }

  const duration = Math.max(0.1, Number(candidate.duration) || DEFAULT_FULL_BODY_CLIP.duration);
  const name = typeof candidate.name === "string" && candidate.name ? candidate.name : "robot-dog-animation";
  const tracks = {};

  for (const legId of LEG_IDS) {
    tracks[legId] = normalizeTrack(candidate.tracks?.[legId], duration);
  }

  return {
    version: 2,
    name,
    duration,
    tracks
  };
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
        y: a.foot.y + (b.foot.y - a.foot.y) * t
      };
    }
  }

  return track[track.length - 1].foot;
}

export function interpolateFullBodyClip(clip, time) {
  const validated = validateClip(clip);
  const frame = {};
  for (const legId of LEG_IDS) {
    frame[legId] = interpolateTrack(validated.tracks[legId], time) ?? { x: 0, y: 0 };
  }
  return frame;
}

export function convertLegacyLegClip(legacyClip, placement = { mode: "selected_leg", legId: "front_left" }) {
  if (!legacyClip || typeof legacyClip !== "object") {
    throw new Error("Legacy clip file is invalid.");
  }

  const duration = Math.max(0.1, Number(legacyClip.duration) || 2);
  const keyframes = normalizeTrack(legacyClip.keyframes, duration);
  const tracks = Object.fromEntries(LEG_IDS.map((legId) => [legId, []]));

  if (placement.mode === "mirrored_pair") {
    const pair = placement.pair ?? "front";
    const legIds =
      pair === "rear"
        ? ["rear_left", "rear_right"]
        : pair === "left"
          ? ["front_left", "rear_left"]
          : pair === "right"
            ? ["front_right", "rear_right"]
            : ["front_left", "front_right"];

    for (const legId of legIds) {
      tracks[legId] = keyframes.map((frame) => ({ ...frame, foot: { ...frame.foot } }));
    }
  } else {
    const legId = placement.legId ?? "front_left";
    tracks[legId] = keyframes.map((frame) => ({ ...frame, foot: { ...frame.foot } }));
  }

  return validateClip({
    version: 2,
    name: legacyClip.name || "converted-leg-clip",
    duration,
    tracks
  });
}

export function createUploadFrames(clip) {
  const validated = validateClip(clip);
  const frames = [
    {
      type: "upload_animation",
      stage: "begin",
      name: validated.name,
      duration: validated.duration,
      trackCount: LEG_IDS.length
    }
  ];

  for (const legId of LEG_IDS) {
    for (const keyframe of validated.tracks[legId]) {
      frames.push({
        type: "upload_animation",
        stage: "frame",
        legId,
        time: keyframe.time,
        x: keyframe.foot.x,
        y: keyframe.foot.y
      });
    }
  }

  frames.push({
    type: "upload_animation",
    stage: "commit",
    name: validated.name
  });

  return frames;
}
