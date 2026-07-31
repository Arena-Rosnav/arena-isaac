"""Wire-driven skeletal pose provider, fed by pushed ROS4HRI joint angles.

The arena_peds wire is the single source of motion: bones mapped by BONE_MAP
follow the pushed angles absolutely, composed onto a neutral standing stance
(the bundle idle clip's first frame, extracted into meta.json at conversion,
NOT the skeleton rest, which is a T-pose on CMU rigs). Unmapped bones hold the
neutral stance. When the wire goes stale the blend weight ramps back to the
neutral stance so there is no visual pop, and ramps up again on a fresh push.
Pure numpy, importable without carb/omni/pxr (carb is only touched lazily,
inside push(), for a best-effort once-per-name warning).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from peds.providers.base import JointPose
from peds.providers.bone_map import BONE_MAP
from peds.providers.math import quat_normalize, quat_slerp

_DEFAULT_PERIOD_S = 0.05  # assumed frame spacing when fewer than 2 samples are buffered
_INTERP_DELAY_PERIODS = 1.5  # latency-compensation window, in units of the assumed period

_unknown_names_seen: set[str] = set()  # module-level, so a name is warned about only once


def _warn_unknown_once(name: str) -> None:
    """Log once per never-before-seen joint name absent from BONE_MAP, no-op without carb."""
    if name in _unknown_names_seen:
        return
    _unknown_names_seen.add(name)
    try:
        import carb
    except ImportError:
        return
    carb.log_warn(f"ExternalPoseProvider: unrecognized joint name '{name}', ignoring")


def axis_angle_quat(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    """Build an xyzw quaternion for a rotation of angle radians about axis (auto-normalized)."""
    axis_arr = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis_arr))
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis_arr = axis_arr / norm
    half = angle / 2.0
    sin_half = math.sin(half)
    return np.array([axis_arr[0] * sin_half, axis_arr[1] * sin_half, axis_arr[2] * sin_half, math.cos(half)], dtype=float)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two xyzw quaternions: q2 applied first, then q1."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


class _AngleRingBuffer:
    """Ring buffer of (stamp, angle vector) frames for externally pushed scalar joint angles.

    Stores angles rather than quats: the raw arena_people_msgs contract is
    scalar joint angles, and the per-bone axis/sign to turn them into quats is
    resolved once from BONE_MAP, not carried per-frame.
    """

    def __init__(self, maxlen: int) -> None:
        self._stamps: deque[float] = deque(maxlen=maxlen)
        self._angles: deque[np.ndarray] = deque(maxlen=maxlen)

    def append(self, stamp: float, angles: np.ndarray) -> None:
        self._stamps.append(stamp)
        self._angles.append(angles)

    def clear(self) -> None:
        self._stamps.clear()
        self._angles.clear()

    @property
    def newest_stamp(self) -> float | None:
        return self._stamps[-1] if self._stamps else None

    def staleness(self, t: float) -> float:
        """Seconds between t and the newest buffered stamp, 0.0 if empty or not stale."""
        newest = self.newest_stamp
        if newest is None:
            return 0.0
        return max(0.0, t - newest)

    def recent_period(self, n: int = 4) -> float:
        """Mean spacing of the last n buffered frame gaps, falls back to _DEFAULT_PERIOD_S."""
        if len(self._stamps) < 2:
            return _DEFAULT_PERIOD_S
        stamps = list(self._stamps)[-(n + 1) :]
        gaps = [b - a for a, b in zip(stamps, stamps[1:], strict=False) if b > a]
        if not gaps:
            return _DEFAULT_PERIOD_S
        return sum(gaps) / len(gaps)

    def evaluate(self, t: float) -> np.ndarray | None:
        """Bracket t between two buffered frames and linearly interpolate, clamped at both ends."""
        if not self._stamps:
            return None
        if len(self._stamps) == 1 or t <= self._stamps[0]:
            return self._angles[0].copy()
        if t >= self._stamps[-1]:
            return self._angles[-1].copy()

        stamps = list(self._stamps)
        idx = int(np.searchsorted(stamps, t, side="right")) - 1
        idx = min(max(idx, 0), len(stamps) - 2)
        next_idx = idx + 1

        span = stamps[next_idx] - stamps[idx]
        alpha = 0.0 if span <= 0.0 else (t - stamps[idx]) / span
        return (1.0 - alpha) * self._angles[idx] + alpha * self._angles[next_idx]


class ExternalPoseProvider:
    """Renders pushed ROS4HRI joint angles over a neutral standing stance."""

    def __init__(
        self,
        joint_order: tuple[str, ...],
        neutral_rotations: np.ndarray,
        neutral_translations: np.ndarray,
        maxlen: int = 32,
        staleness_s: float = 0.5,
        blend_s: float = 0.3,
    ) -> None:
        self._staleness_s = staleness_s
        self._blend_s = blend_s
        self._buffer = _AngleRingBuffer(maxlen=maxlen)
        self._weight = 0.0

        self._neutral_rotations = quat_normalize(np.asarray(neutral_rotations, dtype=float).reshape(len(joint_order), 4))
        self._neutral_translations = np.asarray(neutral_translations, dtype=float).reshape(len(joint_order), 3)

        bone_index: dict[str, int] = {}
        for i, name in enumerate(joint_order):
            bone_index.setdefault(name.rsplit("/", 1)[-1], i)
        schema_names: list[str] = []
        schema_bone_idx: list[int] = []
        schema_axis: list[tuple[float, float, float]] = []
        schema_gain: list[float] = []
        for name, targets in BONE_MAP.items():
            for target in targets:
                idx = bone_index.get(target.bone)
                if idx is None:
                    continue
                schema_names.append(name)
                schema_bone_idx.append(idx)
                schema_axis.append(target.axis)
                schema_gain.append(target.sign * target.scale)

        self._schema_names: tuple[str, ...] = tuple(schema_names)
        self._schema_bone_idx: np.ndarray = np.asarray(schema_bone_idx, dtype=int)
        self._schema_axis: np.ndarray = np.asarray(schema_axis, dtype=float).reshape(-1, 3)
        self._schema_gain: np.ndarray = np.asarray(schema_gain, dtype=float)

    def push(self, stamp_sec: float, names: list[str], positions: list[float]) -> None:
        """Ingest one frame of externally-sourced ROS4HRI joint angles.

        Names absent from BONE_MAP are unrecognized and warned about once; names present
        but mapped to None are a deliberate skip and never warn. Either way they are ignored.
        """
        value_by_name = dict(zip(names, positions, strict=True))
        angles = np.zeros(len(self._schema_names), dtype=float)
        for i, name in enumerate(self._schema_names):
            if name in value_by_name:
                angles[i] = value_by_name[name]

        for name in names:
            if name not in BONE_MAP:
                _warn_unknown_once(name)

        self._buffer.append(stamp_sec, angles)

    def _neutral_pose(self) -> JointPose:
        return JointPose(rotations=self._neutral_rotations.copy(), translations=self._neutral_translations.copy())

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        """Blend mapped bones from the neutral stance toward the buffered wire angles."""
        neutral_pose = self._neutral_pose()

        if self._buffer.newest_stamp is None:
            self._weight = 0.0
            return neutral_pose

        stale = self._buffer.staleness(sim_time) > self._staleness_s
        target_weight = 0.0 if stale else 1.0
        step = dt / self._blend_s if self._blend_s > 0.0 else 1.0
        if target_weight > self._weight:
            self._weight = min(target_weight, self._weight + step)
        else:
            self._weight = max(target_weight, self._weight - step)

        if self._weight <= 0.0:
            return neutral_pose

        interp_delay = _INTERP_DELAY_PERIODS * self._buffer.recent_period()
        angles = self._buffer.evaluate(sim_time - interp_delay)
        if angles is None:
            return neutral_pose

        rotations = neutral_pose.rotations
        overridden = rotations.copy()
        touched: list[int] = []
        seen: set[int] = set()
        for k in range(self._schema_bone_idx.shape[0]):
            bone_idx = int(self._schema_bone_idx[k])
            axis = (float(self._schema_axis[k, 0]), float(self._schema_axis[k, 1]), float(self._schema_axis[k, 2]))
            delta = axis_angle_quat(axis, float(self._schema_gain[k] * angles[k]))
            base = overridden[bone_idx] if bone_idx in seen else self._neutral_rotations[bone_idx]
            overridden[bone_idx] = quat_normalize(quat_multiply(base, delta))
            if bone_idx not in seen:
                touched.append(bone_idx)
                seen.add(bone_idx)

        if touched:
            idx_arr = np.asarray(touched, dtype=int)
            rotations[idx_arr] = quat_slerp(self._neutral_rotations[idx_arr], overridden[idx_arr], self._weight)

        return JointPose(rotations=rotations, translations=neutral_pose.translations)

    def reset_phase(self) -> None:
        self._buffer.clear()
        self._weight = 0.0
