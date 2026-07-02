"""Clip-based gait provider: state-driven clip selection with crossfade."""

from __future__ import annotations

from peds.providers.base import JointPose
from peds.providers.clip import Clip, ClipSampler
from peds.providers.math import quat_slerp

# Animation state constants, matching arena_people_msgs/msg/Pedestrian.msg
IDLE = 0
WALKING = 1
RUNNING = 2
PANIC = 3
SURPRISED = 4
CURIOUS = 5
THREATENING = 6

CLIP_MAP: dict[int, str] = {
    IDLE: "idle",
    WALKING: "walk",
    RUNNING: "run",
    PANIC: "run",
    SURPRISED: "idle",
    CURIOUS: "idle",
    THREATENING: "walk",
}

_BLEND_WINDOW = 0.25  # s, crossfade duration on a clip switch


class GaitProvider:
    """Samples a dict of clips, driven by animation state and planar distance."""

    def __init__(self, clips: dict[str, Clip], stride_length: float) -> None:
        self.clips = clips
        self.stride_length = stride_length
        self.speed = 0.0

        self._clip_key = "idle"
        self._phase = 0.0
        self._previous_key: str | None = None
        self._previous_phase = 0.0
        self._blend_elapsed = 0.0
        self._sampler = ClipSampler()

    @property
    def clip_key(self) -> str:
        return self._clip_key

    @property
    def phase(self) -> float:
        return self._phase

    def set_state(self, animation_state: int, speed: float) -> None:
        """Select the active clip for an animation_state, starting a crossfade on change."""
        self.speed = speed
        key = CLIP_MAP.get(animation_state, "idle")
        if key not in self.clips:
            key = "idle"
        if key == self._clip_key:
            return

        if self._clip_key in self.clips:
            self._previous_key = self._clip_key
            self._previous_phase = self._phase
            self._blend_elapsed = 0.0

        self._clip_key = key
        self._phase = 0.0

    def advance(self, distance: float, dt: float) -> None:
        """Advance the active (and, mid-blend, previous) clip phase.

        Moving clips advance by distance-driven time, idle advances by dt.
        """
        self._phase = self._advance_phase(self._clip_key, self._phase, distance, dt)

        if self._previous_key is not None:
            self._previous_phase = self._advance_phase(self._previous_key, self._previous_phase, distance, dt)
            self._blend_elapsed += dt
            if self._blend_elapsed >= _BLEND_WINDOW:
                self._previous_key = None

    def _advance_phase(self, key: str, phase: float, distance: float, dt: float) -> float:
        if key == "idle":
            return phase + dt
        clip = self.clips.get(key)
        if clip is None or self.stride_length <= 0.0:
            return phase + dt
        return phase + distance * (clip.duration / self.stride_length)

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        """Sample the active clip, slerped against the outgoing clip during a blend."""
        clip = self.clips.get(self._clip_key, self.clips.get("idle"))
        if clip is None:
            raise ValueError("GaitProvider requires at least an 'idle' clip")
        pose = self._sampler.sample(clip, self._phase, looping=True)

        if self._previous_key is None:
            return pose
        previous_clip = self.clips.get(self._previous_key)
        if previous_clip is None:
            return pose

        blend_t = min(1.0, self._blend_elapsed / _BLEND_WINDOW)
        previous_pose = self._sampler.sample(previous_clip, self._previous_phase, looping=True)
        rotations = quat_slerp(previous_pose.rotations, pose.rotations, blend_t)
        translations = (1.0 - blend_t) * previous_pose.translations + blend_t * pose.translations
        return JointPose(rotations=rotations, translations=translations)

    def reset_phase(self) -> None:
        """Clear gait phase and any in-flight crossfade, e.g. after a teleport."""
        self._phase = 0.0
        self._previous_key = None
        self._previous_phase = 0.0
        self._blend_elapsed = 0.0
