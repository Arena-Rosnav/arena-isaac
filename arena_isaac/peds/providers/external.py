"""Wire-driven skeletal pose provider, fed by pushed body-rig joint rotations.

The arena_peds wire is the single source of motion: each Arena joint's
parent-relative rotation is composed down the .rig hierarchy and sets the
world rotation of the skeleton bones the rig maps to it. Unmapped bones hold the
neutral clip rotation, driver bones turn about their .rig axis by the pushed
scalar. When the wire goes stale the blend weight ramps back to the rest pose so
there is no visual pop, and ramps up again on a fresh push.
Pure numpy, importable without carb/omni/pxr (carb is only touched lazily,
inside push(), for a best-effort once-per-name warning).
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from typing import NamedTuple

import numpy as np

from arena_assets.rig import Rig
from peds.providers.base import JointPose
from peds.providers.math import quat_normalize, quat_slerp

_DEFAULT_PERIOD_S = 0.05  # assumed frame spacing when fewer than 2 samples are buffered
_INTERP_DELAY_PERIODS = 1.5  # latency-compensation window, in units of the assumed period

_IDENTITY = (0.0, 0.0, 0.0, 1.0)

_unknown_names_seen: set[str] = set()  # module-level, so a name is warned about only once

Quat = tuple[float, float, float, float]


def _warn_unknown_once(name: str) -> None:
    """Log once per never-before-seen joint name absent from the rig, no-op without carb."""
    if name in _unknown_names_seen:
        return
    _unknown_names_seen.add(name)
    try:
        import carb
    except ImportError:
        return
    carb.log_warn(f"ExternalPoseProvider: unrecognized joint name '{name}', ignoring")


def axis_angle_quat(axis: Sequence[float], angle: float) -> np.ndarray:
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
    return np.array(_qm(tuple(q1), tuple(q2)), dtype=float)


def _qm(a: Sequence[float], b: Sequence[float]) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _qc(q: Sequence[float]) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _qn(q: Sequence[float]) -> Quat:
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm == 0.0:
        return _IDENTITY
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def _qpow(q: Sequence[float], exponent: float) -> Quat:
    """The rotation about q's axis by exponent times q's angle."""
    x, y, z, w = q if q[3] >= 0.0 else (-q[0], -q[1], -q[2], -q[3])
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1e-12:
        return _IDENTITY
    half = exponent * math.atan2(sin_half, w)
    scale = math.sin(half) / sin_half
    return (x * scale, y * scale, z * scale, math.cos(half))


class _Frame(NamedTuple):
    rotations: np.ndarray  # (J, 4) xyzw, one per rig joint
    offset: np.ndarray  # (3,)
    drivers: np.ndarray  # (D,)


def _mix(a: _Frame, b: _Frame, alpha: float) -> _Frame:
    sign = np.where(np.sum(a.rotations * b.rotations, axis=1, keepdims=True) < 0.0, -1.0, 1.0)
    rotations = quat_normalize((1.0 - alpha) * a.rotations + alpha * sign * b.rotations)
    return _Frame(rotations, (1.0 - alpha) * a.offset + alpha * b.offset, (1.0 - alpha) * a.drivers + alpha * b.drivers)


class _FrameBuffer:
    """Ring buffer of (stamp, frame) pairs for externally pushed body poses."""

    def __init__(self, maxlen: int) -> None:
        self._stamps: deque[float] = deque(maxlen=maxlen)
        self._frames: deque[_Frame] = deque(maxlen=maxlen)

    def append(self, stamp: float, frame: _Frame) -> None:
        self._stamps.append(stamp)
        self._frames.append(frame)

    def clear(self) -> None:
        self._stamps.clear()
        self._frames.clear()

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

    def evaluate(self, t: float) -> _Frame | None:
        """Bracket t between two buffered frames and interpolate, clamped at both ends."""
        if not self._stamps:
            return None
        if len(self._stamps) == 1 or t <= self._stamps[0]:
            return self._frames[0]
        if t >= self._stamps[-1]:
            return self._frames[-1]

        stamps = list(self._stamps)
        idx = int(np.searchsorted(stamps, t, side="right")) - 1
        idx = min(max(idx, 0), len(stamps) - 2)
        next_idx = idx + 1

        span = stamps[next_idx] - stamps[idx]
        alpha = 0.0 if span <= 0.0 else (t - stamps[idx]) / span
        return _mix(self._frames[idx], self._frames[next_idx], alpha)


class _Joint(NamedTuple):
    parent: int
    spread: bool
    bones: tuple[tuple[int, float, Quat], ...]  # (bone index, cumulative weight, offset)


class ExternalPoseProvider:
    """Renders pushed body-rig rotations over the skeleton's neutral stance."""

    def __init__(
        self,
        joint_order: tuple[str, ...],
        neutral_rotations: np.ndarray,
        neutral_translations: np.ndarray,
        rig: Rig,
        maxlen: int = 32,
        staleness_s: float = 0.5,
        blend_s: float = 0.3,
        rest_rotations: np.ndarray | None = None,
        rest_translations: np.ndarray | None = None,
    ) -> None:
        self._staleness_s = staleness_s
        self._blend_s = blend_s
        self._buffer = _FrameBuffer(maxlen=maxlen)
        self._weight = 0.0

        count = len(joint_order)
        self._clip = quat_normalize(np.asarray(neutral_rotations, dtype=float).reshape(count, 4))
        self._clip_tuples: list[Quat] = [tuple(row) for row in self._clip.tolist()]
        self._neutral_translations = np.asarray(neutral_translations, dtype=float).reshape(count, 3)

        path_index = {path: i for i, path in enumerate(joint_order)}
        bone_index: dict[str, int] = {}
        for i, path in enumerate(joint_order):
            bone_index.setdefault(path.rsplit("/", 1)[-1], i)
        self._bone_parent = [path_index[path.rsplit("/", 1)[0]] if "/" in path else -1 for path in joint_order]

        self._joint_names = list(rig.joints)
        self._joint_index = {name: i for i, name in enumerate(self._joint_names)}
        self._joints: list[_Joint] = []
        for name, parent in zip(self._joint_names, rig.parents.tolist(), strict=True):
            rows = rig.bones_of(name)
            cumulative = np.cumsum([weight for _, weight, _ in rows]).tolist()
            bones = tuple((bone_index[bone], share, _qn(offset.tolist())) for (bone, _, offset), share in zip(rows, cumulative, strict=True) if bone in bone_index)
            self._joints.append(_Joint(parent, len(rows) > 1, bones))

        self._mapped = {bone for joint in self._joints for bone, _, _ in joint.bones}
        self._top_bone = next((joint.bones[0][0] for joint in self._joints if joint.parent < 0 and joint.bones), -1)

        self._driver_names = tuple(driver.name for driver in rig.drivers)
        self._driver_bones = [bone_index.get(driver.bone, -1) for driver in rig.drivers]
        self._driver_axes = [tuple(float(v) for v in driver.axis) for driver in rig.drivers]

        needed = set(self._mapped) | {bone for bone in self._driver_bones if bone >= 0}
        chain: set[int] = set()
        for bone in needed:
            while bone >= 0 and bone not in chain:
                chain.add(bone)
                bone = self._bone_parent[bone]
        self._chain = sorted(chain)
        self._chain_idx = np.asarray(self._chain, dtype=int)

        world = {name: _qn(q) for name, q in zip(self._joint_names, rig.stance, strict=True)}
        stance_locals: list[Quat] = []
        for name, joint in zip(self._joint_names, self._joints, strict=True):
            parent_world = world[self._joint_names[joint.parent]] if joint.parent >= 0 else _IDENTITY
            stance_locals.append(_qn(_qm(_qc(parent_world), world[name])))
        self._stance_locals = np.array(stance_locals, dtype=float).reshape(-1, 4)
        self._neutral_rotations = self._solve(stance_locals, [0.0] * len(self._driver_names))

        self._rest_rotations = self._neutral_rotations if rest_rotations is None else quat_normalize(np.asarray(rest_rotations, dtype=float).reshape(count, 4))
        self._rest_translations = self._neutral_translations if rest_translations is None else np.asarray(rest_translations, dtype=float).reshape(count, 3)
        self._rest_bones = np.flatnonzero(np.any(self._rest_rotations != self._neutral_rotations, axis=1))

    def push(
        self,
        stamp_sec: float,
        names: Sequence[str],
        rotations: Sequence[Sequence[float]],
        root_offset: Sequence[float],
        driver_names: Sequence[str],
        driver_positions: Sequence[float],
    ) -> None:
        """Ingest one frame of parent-relative xyzw rotations, the root offset and driver angles."""
        frame_rotations = self._stance_locals.copy()
        for name, rotation in zip(names, rotations, strict=True):
            index = self._joint_index.get(name)
            if index is None:
                _warn_unknown_once(name)
                continue
            frame_rotations[index] = _qn(rotation)

        angles = np.zeros(len(self._driver_names), dtype=float)
        driver_index = {name: i for i, name in enumerate(self._driver_names)}
        for name, position in zip(driver_names, driver_positions, strict=True):
            index = driver_index.get(name)
            if index is None:
                _warn_unknown_once(name)
                continue
            angles[index] = position

        self._buffer.append(stamp_sec, _Frame(frame_rotations, np.asarray(root_offset, dtype=float).reshape(3), angles))

    def _solve(self, locals_: Sequence[Quat], angles: Sequence[float]) -> np.ndarray:
        """Bone-local rotations (B, 4) for rig joint locals: neutral clip rotations outside the rig chain."""
        globals_: list[Quat] = []
        mapped: dict[int, Quat] = {}
        for joint, local in zip(self._joints, locals_, strict=True):
            parent_global = globals_[joint.parent] if joint.parent >= 0 else _IDENTITY
            globals_.append(_qm(parent_global, local))
            for bone, cumulative, offset in joint.bones:
                base = _qm(parent_global, _qpow(local, cumulative)) if joint.spread else globals_[-1]
                mapped[bone] = _qm(base, offset)

        deltas: dict[int, list[Quat]] = {}
        for bone, axis, angle in zip(self._driver_bones, self._driver_axes, angles, strict=True):
            if bone >= 0:
                deltas.setdefault(bone, []).append(_qn(axis_angle_quat(axis, float(angle)).tolist()))

        rotations = self._clip.copy()
        worlds: dict[int, Quat] = {}
        for bone in self._chain:
            parent = self._bone_parent[bone]
            parent_world = worlds[parent] if parent >= 0 else _IDENTITY
            local = _qm(_qc(parent_world), mapped[bone]) if bone in mapped else self._clip_tuples[bone]
            for delta in deltas.get(bone, ()):
                local = _qm(local, delta)
            local = _qn(local)
            rotations[bone] = local
            worlds[bone] = _qm(parent_world, local)
        return rotations

    def _rest_pose(self) -> JointPose:
        return JointPose(rotations=self._rest_rotations.copy(), translations=self._rest_translations.copy())

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        """Blend from the rest pose toward the buffered wire pose."""
        rest_pose = self._rest_pose()

        if self._buffer.newest_stamp is None:
            self._weight = 0.0
            return rest_pose

        stale = self._buffer.staleness(sim_time) > self._staleness_s
        target_weight = 0.0 if stale else 1.0
        step = dt / self._blend_s if self._blend_s > 0.0 else 1.0
        if target_weight > self._weight:
            self._weight = min(target_weight, self._weight + step)
        else:
            self._weight = max(target_weight, self._weight - step)

        if self._weight <= 0.0:
            return rest_pose

        interp_delay = _INTERP_DELAY_PERIODS * self._buffer.recent_period()
        frame = self._buffer.evaluate(sim_time - interp_delay)
        if frame is None:
            return rest_pose

        target = self._solve([tuple(row) for row in frame.rotations.tolist()], frame.drivers.tolist())
        target_translations = self._neutral_translations.copy()
        if self._top_bone >= 0:
            target_translations[self._top_bone] += frame.offset

        rotations = rest_pose.rotations
        idx_arr = np.union1d(self._chain_idx, self._rest_bones)
        if idx_arr.size:
            rotations[idx_arr] = quat_slerp(self._rest_rotations[idx_arr], target[idx_arr], self._weight)

        translations = self._rest_translations + self._weight * (target_translations - self._rest_translations)
        return JointPose(rotations=rotations, translations=translations)

    def reset_phase(self) -> None:
        self._buffer.clear()
        self._weight = 0.0
