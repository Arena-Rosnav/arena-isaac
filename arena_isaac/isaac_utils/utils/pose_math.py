"""Rigid transforms as position and wxyz quaternion arrays."""

from __future__ import annotations

import numpy as np


def _mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def _conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    return _mul(_mul(q, np.array([0.0, *v])), _conjugate(q))[1:]


def _unit(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def relative(base_position: np.ndarray, base_orientation: np.ndarray, position: np.ndarray, orientation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pose of a world pose in the frame of a base pose."""
    inverse = _conjugate(_unit(base_orientation))
    return _rotate(inverse, position - base_position), _unit(_mul(inverse, orientation))


def compose(base_position: np.ndarray, base_orientation: np.ndarray, offset_position: np.ndarray, offset_orientation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World pose of an offset given in the frame of a base pose."""
    base = _unit(base_orientation)
    return base_position + _rotate(base, offset_position), _unit(_mul(base, offset_orientation))
