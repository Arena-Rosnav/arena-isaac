"""UsdSkel.Animation-backed clip loading and sampling.

pxr (usd-core) is imported lazily inside Clip.load, not at module top, so
this module (and the Clip dataclass/ClipSampler it defines) stays
importable without pxr or omni/carb, e.g. from gait.py in a plain test.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from peds.providers.base import JointPose
from peds.providers.math import quat_slerp

if TYPE_CHECKING:
    from pxr import Usd, UsdSkel


@dataclass(eq=False)
class Clip:
    """A loaded SkelAnimation clip, resampled into flat numpy arrays."""

    joint_order: tuple[str, ...]
    times: np.ndarray  # (K,) seconds, relative to the first keyframe, times[0] == 0.0
    rotations: np.ndarray  # (K, J, 4) xyzw quaternions
    translations: np.ndarray  # (K, J, 3)
    duration: float

    @classmethod
    def load(cls, path: str, prim_path: str | None = None) -> Clip:
        """Load a Clip from a SkelAnimation prim in a .usda/.usd file.

        When prim_path is None, the first SkelAnimation prim found by a
        stage traversal is used.
        """
        from pxr import Usd

        stage = Usd.Stage.Open(path)
        if stage is None:
            raise ValueError(f"could not open USD stage at {path}")

        anim = _find_animation(stage, prim_path)

        joint_order = tuple(str(joint) for joint in anim.GetJointsAttr().Get())
        joint_count = len(joint_order)

        rotations_attr = anim.GetRotationsAttr()
        translations_attr = anim.GetTranslationsAttr()

        time_samples = sorted(set(rotations_attr.GetTimeSamples()) | set(translations_attr.GetTimeSamples()))
        if not time_samples:
            raise ValueError(f"SkelAnimation at {anim.GetPrim().GetPath()} in {path} has no time samples")

        rotations = np.zeros((len(time_samples), joint_count, 4), dtype=float)
        translations = np.zeros((len(time_samples), joint_count, 3), dtype=float)

        for k, time_code in enumerate(time_samples):
            rotation_sample = rotations_attr.Get(time_code)
            translation_sample = translations_attr.Get(time_code)
            for j in range(joint_count):
                quat = rotation_sample[j]
                imaginary = quat.GetImaginary()
                rotations[k, j] = (imaginary[0], imaginary[1], imaginary[2], quat.GetReal())
                translations[k, j] = tuple(translation_sample[j])

        fps = stage.GetTimeCodesPerSecond() or 24.0
        times = np.asarray(time_samples, dtype=float) / fps
        times = times - times[0]
        duration = float(times[-1])

        return cls(
            joint_order=joint_order,
            times=times,
            rotations=rotations,
            translations=translations,
            duration=duration,
        )


def _find_animation(stage: Usd.Stage, prim_path: str | None) -> UsdSkel.Animation:
    from pxr import UsdSkel

    if prim_path is not None:
        anim = UsdSkel.Animation(stage.GetPrimAtPath(prim_path))
        if not anim:
            raise ValueError(f"prim at {prim_path} is not a SkelAnimation")
        return anim

    for prim in stage.Traverse():
        anim = UsdSkel.Animation(prim)
        if anim:
            return anim

    raise ValueError("no SkelAnimation prim found in stage")


class ClipSampler:
    """Bracket a clip's keyframes by phase time and blend between the pair."""

    def sample(self, clip: Clip, phase_time: float, looping: bool = True) -> JointPose:
        if clip.times.shape[0] == 1 or clip.duration <= 0.0:
            return JointPose(rotations=clip.rotations[0].copy(), translations=clip.translations[0].copy())

        phase = phase_time % clip.duration if looping else min(max(phase_time, 0.0), clip.duration)

        idx = int(np.searchsorted(clip.times, phase, side="right")) - 1
        idx = min(max(idx, 0), clip.times.shape[0] - 2)
        next_idx = idx + 1

        span = clip.times[next_idx] - clip.times[idx]
        t = 0.0 if span <= 0.0 else float((phase - clip.times[idx]) / span)

        rotations = quat_slerp(clip.rotations[idx], clip.rotations[next_idx], t)
        translations = (1.0 - t) * clip.translations[idx] + t * clip.translations[next_idx]
        return JointPose(rotations=rotations, translations=translations)
