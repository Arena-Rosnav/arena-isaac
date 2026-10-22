"""Sole contacts that stand a posed skeleton on the floor. Pure numpy."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from peds.providers.base import JointPose


def _matrices(xyzw: np.ndarray) -> np.ndarray:
    """Rotation matrices (n, 3, 3) of xyzw quaternions (n, 4)."""
    q = xyzw / np.linalg.norm(xyzw, axis=1, keepdims=True)
    x, y, z, w = q.T
    return np.stack(
        [
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=-1),
            np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=-1),
            np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=-1),
        ],
        axis=-2,
    )


class Soles:
    """Heel and toe points of the feet, each in the frame of the joint it rides on."""

    def __init__(self, parents: Sequence[int], joints: Sequence[int], points: np.ndarray) -> None:
        chain: set[int] = set()
        for joint in joints:
            while joint >= 0 and joint not in chain:
                chain.add(joint)
                joint = parents[joint]
        self._parents = tuple(parents)
        self._chain = sorted(chain)
        self._joints = tuple(joints)
        self._points = np.asarray(points, dtype=float).reshape(-1, 3)

    def height(self, pose: JointPose) -> float:
        """Height of the lowest sole point above the skeleton origin."""
        rotations = _matrices(np.asarray(pose.rotations, dtype=float)[self._chain])
        worlds: dict[int, np.ndarray] = {}
        for rotation, joint in zip(rotations, self._chain, strict=True):
            local = np.eye(4)
            local[:3, :3] = rotation
            local[:3, 3] = pose.translations[joint]
            parent = self._parents[joint]
            worlds[joint] = worlds[parent] @ local if parent >= 0 else local
        return min(float(worlds[joint][2, :3] @ point + worlds[joint][2, 3]) for joint, point in zip(self._joints, self._points, strict=True))
