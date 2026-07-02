"""Vectorized quaternion helpers. Pure numpy."""

from __future__ import annotations

import numpy as np

_NLERP_EPSILON = 1e-6  # below this sin(theta), slerp's basis degenerates, fall back to nlerp


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize a batch of xyzw quaternions along the last axis."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    norm = np.where(norm == 0.0, 1.0, norm)
    return q / norm


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Spherically interpolate two batches of xyzw quaternions, shape (J, 4).

    Antipodal pairs are resolved to the shortest path by negating b where
    the dot product is negative. Falls back to normalized lerp when the
    angle between a and b is too small for the slerp basis to be stable.
    """
    a = quat_normalize(a)
    b = np.asarray(b, dtype=float)

    dot = np.sum(a * b, axis=-1, keepdims=True)
    b = np.where(dot < 0.0, -b, b)
    b = quat_normalize(b)
    dot = np.clip(np.abs(dot), -1.0, 1.0)

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    small = sin_theta < _NLERP_EPSILON

    safe_sin_theta = np.where(small, 1.0, sin_theta)
    w_a = np.where(small, 1.0 - t, np.sin((1.0 - t) * theta) / safe_sin_theta)
    w_b = np.where(small, t, np.sin(t * theta) / safe_sin_theta)

    return quat_normalize(w_a * a + w_b * b)
