"""Pose provider contract. Pure numpy, no omni/carb/pxr imports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np


@dataclass(eq=False)
class JointPose:
    """Per-evaluation joint-local pose.

    Joint scales are constant and owned by the writer, they are never
    carried through JointPose.
    """

    rotations: np.ndarray  # (J, 4) xyzw quaternions, joint-local
    translations: np.ndarray  # (J, 3), joint-local


class PoseProvider(Protocol):
    """Anything that can produce a joint-local skeleton pose for a sim time."""

    def evaluate(self, sim_time: float, dt: float) -> JointPose: ...


@runtime_checkable
class ResettablePhase(Protocol):
    """Optional capability: providers that keep an internal phase resettable on teleport."""

    def reset_phase(self) -> None: ...


@runtime_checkable
class DistanceDrivenPose(Protocol):
    """Optional capability: providers whose phase advances with planar displacement."""

    def advance(self, distance: float, dt: float) -> None: ...
