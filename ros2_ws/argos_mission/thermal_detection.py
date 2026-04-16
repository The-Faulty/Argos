"""Pure thermal-processing utilities shared by the ROS nodes and unit tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


Pixel = Tuple[int, int]


@dataclass(frozen=True)
class ThermalCluster:
    """Summary statistics for one connected thermal hotspot."""

    centroid_row: float
    centroid_col: float
    pixel_count: int
    mean_temp_c: float
    peak_temp_c: float
    contrast_temp_c: float
    ambient_temp_c: float
    threshold_temp_c: float
    bbox: Tuple[int, int, int, int]


@dataclass
class TrackedHotspot:
    """State for a hotspot being tracked across frames."""

    track_id: int
    cluster: ThermalCluster
    consecutive_hits: int = 1
    missed_frames: int = 0


def smooth_thermal_frame(frame: np.ndarray, passes: int = 1) -> np.ndarray:
    """Apply a small 3x3 box filter to suppress single-pixel MLX noise."""
    result = np.asarray(frame, dtype=np.float32)
    if passes <= 0:
        return np.ascontiguousarray(result, dtype=np.float32)

    for _ in range(int(passes)):
        padded = np.pad(result, 1, mode="edge")
        result = (
            padded[:-2, :-2]
            + padded[:-2, 1:-1]
            + padded[:-2, 2:]
            + padded[1:-1, :-2]
            + padded[1:-1, 1:-1]
            + padded[1:-1, 2:]
            + padded[2:, :-2]
            + padded[2:, 1:-1]
            + padded[2:, 2:]
        ) / 9.0
    return np.ascontiguousarray(result, dtype=np.float32)


def _neighbors(connectivity: int) -> Sequence[Tuple[int, int]]:
    if int(connectivity) == 4:
        return ((1, 0), (-1, 0), (0, 1), (0, -1))
    return (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    )


def connected_components(mask: np.ndarray, connectivity: int = 8) -> List[List[Pixel]]:
    """Find connected components in a binary mask."""
    visited = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    components: List[List[Pixel]] = []
    neighbor_offsets = _neighbors(connectivity)

    for row in range(height):
        for col in range(width):
            if not mask[row, col] or visited[row, col]:
                continue

            queue = [(row, col)]
            visited[row, col] = True
            pixels: List[Pixel] = []

            while queue:
                current_row, current_col = queue.pop()
                pixels.append((current_row, current_col))
                for delta_row, delta_col in neighbor_offsets:
                    next_row = current_row + delta_row
                    next_col = current_col + delta_col
                    if (
                        0 <= next_row < height
                        and 0 <= next_col < width
                        and mask[next_row, next_col]
                        and not visited[next_row, next_col]
                    ):
                        visited[next_row, next_col] = True
                        queue.append((next_row, next_col))

            components.append(pixels)

    return components


def detect_hotspots(
    frame: np.ndarray,
    *,
    relative_threshold_c: float,
    min_cluster_pixels: int,
    min_victim_temp_c: float,
    max_victim_temp_c: float,
    min_peak_delta_c: float = 2.5,
    max_cluster_pixels: Optional[int] = None,
    smoothing_passes: int = 1,
    connectivity: int = 8,
) -> tuple[List[ThermalCluster], float, float]:
    """Detect connected thermal hotspots that look like human-sized warm objects."""
    raw_frame = np.asarray(frame, dtype=np.float32)
    if raw_frame.ndim != 2:
        raise ValueError(f"Expected a 2-D thermal frame, got shape {raw_frame.shape}.")

    finite_mask = np.isfinite(raw_frame)
    if not np.any(finite_mask):
        return [], float("nan"), float("nan")

    fill_value = float(np.median(raw_frame[finite_mask]))
    filled_frame = np.where(finite_mask, raw_frame, fill_value)
    smoothed_frame = smooth_thermal_frame(filled_frame, passes=smoothing_passes)
    ambient_temp_c = float(np.median(smoothed_frame[finite_mask]))
    threshold_temp_c = max(min_victim_temp_c, ambient_temp_c + relative_threshold_c)
    candidate_frame = np.maximum(smoothed_frame, filled_frame)
    hot_mask = np.logical_and(
        finite_mask,
        np.logical_and(candidate_frame >= threshold_temp_c, candidate_frame <= max_victim_temp_c),
    )

    clusters: List[ThermalCluster] = []
    for pixels in connected_components(hot_mask, connectivity=connectivity):
        pixel_count = len(pixels)
        if pixel_count < int(min_cluster_pixels):
            continue
        if max_cluster_pixels is not None and pixel_count > int(max_cluster_pixels):
            continue

        rows = np.array([pixel[0] for pixel in pixels], dtype=np.float32)
        cols = np.array([pixel[1] for pixel in pixels], dtype=np.float32)
        temperatures = np.array([filled_frame[row, col] for row, col in pixels], dtype=np.float32)
        peak_temp_c = float(np.max(temperatures))
        mean_temp_c = float(np.mean(temperatures))
        contrast_temp_c = peak_temp_c - ambient_temp_c

        if peak_temp_c < min_victim_temp_c or peak_temp_c > max_victim_temp_c:
            continue
        if contrast_temp_c < float(min_peak_delta_c):
            continue

        clusters.append(
            ThermalCluster(
                centroid_row=float(rows.mean()),
                centroid_col=float(cols.mean()),
                pixel_count=pixel_count,
                mean_temp_c=mean_temp_c,
                peak_temp_c=peak_temp_c,
                contrast_temp_c=contrast_temp_c,
                ambient_temp_c=ambient_temp_c,
                threshold_temp_c=threshold_temp_c,
                bbox=(
                    int(rows.min()),
                    int(cols.min()),
                    int(rows.max()),
                    int(cols.max()),
                ),
            )
        )

    clusters.sort(
        key=lambda cluster: (cluster.peak_temp_c, cluster.pixel_count, cluster.contrast_temp_c),
        reverse=True,
    )
    return clusters, ambient_temp_c, threshold_temp_c


class HotspotTracker:
    """Track detections across frames so single-frame noise does not trigger a victim report."""

    def __init__(
        self,
        *,
        match_radius_px: float,
        min_confirmed_frames: int,
        max_missed_frames: int,
    ):
        self.match_radius_px = float(match_radius_px)
        self.min_confirmed_frames = max(1, int(min_confirmed_frames))
        self.max_missed_frames = max(0, int(max_missed_frames))
        self._next_track_id = 1
        self._tracks: Dict[int, TrackedHotspot] = {}

    def update(self, clusters: Iterable[ThermalCluster]) -> List[TrackedHotspot]:
        """Update active tracks and return the currently confirmed detections."""
        cluster_list = list(clusters)
        track_ids = list(self._tracks.keys())
        candidate_matches: List[Tuple[float, int, int]] = []

        for track_id in track_ids:
            track = self._tracks[track_id]
            for cluster_index, cluster in enumerate(cluster_list):
                distance = math.hypot(
                    cluster.centroid_row - track.cluster.centroid_row,
                    cluster.centroid_col - track.cluster.centroid_col,
                )
                if distance <= self.match_radius_px:
                    candidate_matches.append((distance, track_id, cluster_index))

        matched_track_ids = set()
        matched_cluster_indices = set()
        for _, track_id, cluster_index in sorted(candidate_matches, key=lambda item: item[0]):
            if track_id in matched_track_ids or cluster_index in matched_cluster_indices:
                continue
            track = self._tracks[track_id]
            track.cluster = cluster_list[cluster_index]
            track.consecutive_hits += 1
            track.missed_frames = 0
            matched_track_ids.add(track_id)
            matched_cluster_indices.add(cluster_index)

        for track_id in track_ids:
            if track_id in matched_track_ids:
                continue
            track = self._tracks[track_id]
            track.missed_frames += 1
            if track.missed_frames > self.max_missed_frames:
                del self._tracks[track_id]

        for cluster_index, cluster in enumerate(cluster_list):
            if cluster_index in matched_cluster_indices:
                continue
            track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[track_id] = TrackedHotspot(track_id=track_id, cluster=cluster)

        return self.confirmed_tracks()

    def confirmed_tracks(self) -> List[TrackedHotspot]:
        """Return hotspots that persisted for long enough to be treated as victims."""
        confirmed = [
            track
            for track in self._tracks.values()
            if track.missed_frames == 0 and track.consecutive_hits >= self.min_confirmed_frames
        ]
        confirmed.sort(
            key=lambda track: (
                track.cluster.peak_temp_c,
                track.cluster.pixel_count,
                track.consecutive_hits,
            ),
            reverse=True,
        )
        return confirmed
