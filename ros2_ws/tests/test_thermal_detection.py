"""Unit tests for the pure thermal detection helpers."""

from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from argos_mission.thermal_common import apply_frame_orientation, oriented_fov_radians
from argos_mission.thermal_detection import HotspotTracker, detect_hotspots


def test_apply_frame_orientation_rotates_and_flips():
    frame = np.arange(6, dtype=np.float32).reshape((2, 3))
    oriented = apply_frame_orientation(
        frame,
        rotation_degrees=90,
        flip_horizontal=True,
    )
    expected = np.array(
        [
            [5.0, 2.0],
            [4.0, 1.0],
            [3.0, 0.0],
        ],
        dtype=np.float32,
    )
    assert np.array_equal(oriented, expected)


def test_oriented_fov_swaps_when_rotated_sideways():
    horizontal, vertical = oriented_fov_radians(1.0, 2.0, rotation_degrees=90)
    assert horizontal == 2.0
    assert vertical == 1.0


def test_detect_hotspots_finds_compact_warm_cluster():
    frame = np.full((24, 32), 22.0, dtype=np.float32)
    frame[10:12, 15:18] = 36.0

    clusters, ambient, threshold = detect_hotspots(
        frame,
        relative_threshold_c=6.0,
        min_cluster_pixels=4,
        max_cluster_pixels=90,
        min_victim_temp_c=30.0,
        max_victim_temp_c=42.0,
        min_peak_delta_c=2.5,
        smoothing_passes=1,
        connectivity=8,
    )

    assert len(clusters) == 1
    cluster = clusters[0]
    assert ambient == 22.0
    assert threshold == 30.0
    assert cluster.pixel_count >= 4
    assert cluster.peak_temp_c == 36.0
    assert 10.0 <= cluster.centroid_row <= 11.5
    assert 15.0 <= cluster.centroid_col <= 17.5


def test_detect_hotspots_rejects_single_pixel_noise():
    frame = np.full((24, 32), 22.0, dtype=np.float32)
    frame[12, 16] = 38.0

    clusters, _, _ = detect_hotspots(
        frame,
        relative_threshold_c=6.0,
        min_cluster_pixels=4,
        max_cluster_pixels=90,
        min_victim_temp_c=30.0,
        max_victim_temp_c=42.0,
        min_peak_delta_c=2.5,
        smoothing_passes=1,
        connectivity=8,
    )

    assert clusters == []


def test_hotspot_tracker_requires_persistence():
    frame = np.full((24, 32), 22.0, dtype=np.float32)
    frame[10:12, 15:18] = 36.0
    clusters, _, _ = detect_hotspots(
        frame,
        relative_threshold_c=6.0,
        min_cluster_pixels=4,
        max_cluster_pixels=90,
        min_victim_temp_c=30.0,
        max_victim_temp_c=42.0,
        min_peak_delta_c=2.5,
        smoothing_passes=1,
        connectivity=8,
    )

    tracker = HotspotTracker(
        match_radius_px=3.5,
        min_confirmed_frames=2,
        max_missed_frames=1,
    )

    assert tracker.update(clusters) == []
    confirmed = tracker.update(clusters)
    assert len(confirmed) == 1
    assert confirmed[0].consecutive_hits == 2
