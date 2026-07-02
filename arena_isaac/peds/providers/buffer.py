"""Stamped pose ring buffer, for providers driven by external/upstream skeleton samples."""

from __future__ import annotations

from collections import deque

import numpy as np

from peds.providers.base import JointPose
from peds.providers.math import quat_slerp


class StampedPoseBuffer:
    """Ring buffer of (stamp, rotations, translations) frames, interpolated at read time."""

    def __init__(self, maxlen: int = 32) -> None:
        self._stamps: deque[float] = deque(maxlen=maxlen)
        self._rotations: deque[np.ndarray] = deque(maxlen=maxlen)
        self._translations: deque[np.ndarray] = deque(maxlen=maxlen)

    def append(self, stamp: float, rotations: np.ndarray, translations: np.ndarray) -> None:
        self._stamps.append(stamp)
        self._rotations.append(np.asarray(rotations, dtype=float))
        self._translations.append(np.asarray(translations, dtype=float))

    @property
    def newest_stamp(self) -> float | None:
        return self._stamps[-1] if self._stamps else None

    def staleness(self, t: float) -> float:
        """Seconds between t and the newest buffered stamp, 0.0 if empty or not stale."""
        newest = self.newest_stamp
        if newest is None:
            return 0.0
        return max(0.0, t - newest)

    def evaluate(self, t: float) -> JointPose | None:
        """Bracket t between two buffered frames and slerp/lerp between them.

        Clamps to the newest frame when t is beyond it, joints are never
        extrapolated.
        """
        if not self._stamps:
            return None

        if len(self._stamps) == 1 or t <= self._stamps[0]:
            return JointPose(rotations=self._rotations[0].copy(), translations=self._translations[0].copy())

        if t >= self._stamps[-1]:
            return JointPose(rotations=self._rotations[-1].copy(), translations=self._translations[-1].copy())

        stamps = list(self._stamps)
        idx = int(np.searchsorted(stamps, t, side="right")) - 1
        idx = min(max(idx, 0), len(stamps) - 2)
        next_idx = idx + 1

        span = stamps[next_idx] - stamps[idx]
        alpha = 0.0 if span <= 0.0 else (t - stamps[idx]) / span

        rotations = quat_slerp(self._rotations[idx], self._rotations[next_idx], alpha)
        translations = (1.0 - alpha) * self._translations[idx] + alpha * self._translations[next_idx]
        return JointPose(rotations=rotations, translations=translations)
