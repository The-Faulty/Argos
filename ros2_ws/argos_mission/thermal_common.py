"""Shared helpers for Argos thermal-camera acquisition and geometry."""

from __future__ import annotations

import math

import numpy as np


MLX90640_NATIVE_WIDTH = 32
MLX90640_NATIVE_HEIGHT = 24
VALID_ROTATIONS_DEG = (0, 90, 180, 270)


def normalize_rotation_degrees(rotation_degrees: int) -> int:
    """Normalize a requested image rotation to one of the supported values."""
    normalized = int(rotation_degrees) % 360
    if normalized not in VALID_ROTATIONS_DEG:
        raise ValueError(
            f"rotation_degrees must be one of {VALID_ROTATIONS_DEG}, got {rotation_degrees}."
        )
    return normalized


def apply_frame_orientation(
    frame: np.ndarray,
    *,
    rotation_degrees: int = 0,
    flip_horizontal: bool = False,
    flip_vertical: bool = False,
) -> np.ndarray:
    """Rotate and flip a thermal frame to match the sensor's physical mounting."""
    oriented = np.asarray(frame, dtype=np.float32)
    rotation_degrees = normalize_rotation_degrees(rotation_degrees)
    turns = rotation_degrees // 90
    if turns:
        oriented = np.rot90(oriented, k=turns)
    if flip_horizontal:
        oriented = np.fliplr(oriented)
    if flip_vertical:
        oriented = np.flipud(oriented)
    return np.ascontiguousarray(oriented, dtype=np.float32)


def oriented_fov_radians(
    horizontal_fov_rad: float,
    vertical_fov_rad: float,
    *,
    rotation_degrees: int = 0,
) -> tuple[float, float]:
    """Return the effective horizontal/vertical FOV after image rotation."""
    rotation_degrees = normalize_rotation_degrees(rotation_degrees)
    if rotation_degrees in (90, 270):
        return vertical_fov_rad, horizontal_fov_rad
    return horizontal_fov_rad, vertical_fov_rad


def camera_matrix_from_fov(
    *,
    width: int,
    height: int,
    horizontal_fov_rad: float,
    vertical_fov_rad: float,
) -> tuple[float, float, float, float]:
    """Compute pinhole intrinsics from image size and field of view."""
    fx = width / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    fy = height / (2.0 * math.tan(vertical_fov_rad / 2.0))
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    return fx, fy, cx, cy
