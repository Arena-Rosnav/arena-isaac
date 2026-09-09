"""Pedestrian entity: dead-reckoned root motion plus a pluggable skeleton pose provider.

No USD calls here, a later phase wires this into a SkelWriter.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

import numpy as np

from peds.providers.base import PoseProvider, ResettablePhase


def _identity_orientation() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])


@dataclass(eq=False)
class Ped:
    """A pedestrian: sim_path is both the registry key and the published name."""

    sim_path: str
    prim_path: str
    provider: PoseProvider

    command_position: np.ndarray | None = None
    command_orientation: np.ndarray = field(default_factory=_identity_orientation)
    command_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    command_age: float = 0.0

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=_identity_orientation)

    MAX_EXTRAPOLATION: ClassVar[float] = 0.5

    def update_command(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        velocity: np.ndarray,
        now: float,
        stamp_sec: float = 0.0,
    ) -> None:
        """Set the commanded pose/velocity that desired_root dead-reckons from.

        stamp_sec seeds command_age with the pipeline latency, so the first
        desired_root call after a command doesn't rewind by velocity times
        the transport delay.
        """
        self.command_position = np.array([position[0], position[1], self.position[2]])
        self.command_orientation = np.array(orientation, dtype=float)
        self.command_velocity = np.array([velocity[0], velocity[1], 0.0])
        self.command_age = max(0.0, now - stamp_sec) if stamp_sec > 0.0 else 0.0

    def teleport(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Snap to an absolute pose, clearing pending commands and provider gait phase."""
        self.command_position = None
        self.command_orientation = np.array(orientation, dtype=float)
        self.command_velocity = np.zeros(3)
        self.command_age = 0.0
        self.position = np.array(position, dtype=float)
        self.orientation = np.array(orientation, dtype=float)
        if isinstance(self.provider, ResettablePhase):
            self.provider.reset_phase()

    def desired_root(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Advance command_age by dt and dead-reckon (position, orientation).

        Position holds z from the current state, orientation is the commanded one verbatim.
        """
        if self.command_position is None:
            return self.position, self.orientation

        self.command_age += dt
        extrapolate = min(self.command_age, self.MAX_EXTRAPOLATION)
        desired_position = self.command_position + self.command_velocity * extrapolate
        desired_position[2] = self.position[2]

        self.position = desired_position
        self.orientation = self.command_orientation
        return self.position, self.orientation
