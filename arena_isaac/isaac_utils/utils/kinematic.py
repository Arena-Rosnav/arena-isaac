"""Poses and world-frame velocities of moving boxes, advanced by the server every physics step."""

from __future__ import annotations

from collections.abc import Iterable, Sequence

import attrs
import numpy as np


@attrs.define
class _Box:
    position: np.ndarray
    orientation: np.ndarray
    velocity: np.ndarray


class KinematicBoxes:
    """Moving boxes keyed by prim path, orientations as wxyz quaternions."""

    def __init__(self) -> None:
        self._boxes: dict[str, _Box] = {}
        self.revision = 0

    def __contains__(self, path: str) -> bool:
        return path in self._boxes

    def __len__(self) -> int:
        return len(self._boxes)

    @property
    def paths(self) -> tuple[str, ...]:
        return tuple(self._boxes)

    def add(self, path: str, position: Sequence[float], orientation: Sequence[float], velocity: Sequence[float]) -> None:
        self._boxes[path] = _Box(np.array(position, dtype=float), np.array(orientation, dtype=float), np.array(velocity, dtype=float))
        self.revision += 1

    def forget_under(self, path: str) -> bool:
        """Drop every box at or under path."""
        prefix = f'{path.rstrip("/")}/'
        gone = [key for key in self._boxes if key == path or key.startswith(prefix)]
        for key in gone:
            del self._boxes[key]
        if gone:
            self.revision += 1
        return bool(gone)

    def teleport(self, path: str, position: Sequence[float], orientation: Sequence[float]) -> bool:
        box = self._boxes.get(path)
        if box is None:
            return False
        box.position = np.array(position, dtype=float)
        box.orientation = np.array(orientation, dtype=float)
        return True

    def set_velocity(self, path: str, velocity: Sequence[float]) -> bool:
        box = self._boxes.get(path)
        if box is None:
            return False
        box.velocity = np.array(velocity, dtype=float)
        return True

    def step(self, dt: float) -> None:
        for box in self._boxes.values():
            box.position = box.position + box.velocity * dt

    def transforms(self, paths: Iterable[str]) -> np.ndarray:
        """Rows of x, y, z, qx, qy, qz, qw for paths, the physics tensor layout, KeyError on an unknown path."""
        rows = []
        for path in paths:
            box = self._boxes[path]
            rows.append([*box.position, *box.orientation[1:], box.orientation[0]])
        return np.array(rows, dtype=np.float32).reshape(-1, 7)


__all__ = ['KinematicBoxes']
