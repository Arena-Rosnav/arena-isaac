from __future__ import annotations

import numpy as np
import pytest

from peds.ped import Ped
from peds.providers.base import JointPose
from peds.registry import PedRegistry

_IDENTITY = [0.0, 0.0, 0.0, 1.0]
_STEP = 0.004


class _BlendProvider:
    """Records the dt of every evaluation."""

    def __init__(self) -> None:
        self.dts: list[float] = []

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        self.dts.append(dt)
        return JointPose(rotations=np.zeros((1, 4)), translations=np.zeros((1, 3)))


class _RecordingWriter:
    def __init__(self) -> None:
        self.roots: list[np.ndarray] = []
        self.poses = 0

    def set_root(self, ped: Ped, position: np.ndarray, orientation: np.ndarray) -> None:
        self.roots.append(np.array(position))

    def write(self, ped: Ped, pose: JointPose) -> None:
        self.poses += 1


def _walking_ped() -> tuple[Ped, _BlendProvider]:
    provider = _BlendProvider()
    ped = Ped(sim_path="p", prim_path="/World/p", provider=provider)
    ped.update_command(orientation=_IDENTITY, position=[0.0, 0.0, 0.0], velocity=[1.0, 0.0], now=0.0)
    return ped, provider


def test_a_frame_of_five_steps_writes_once_with_the_frame_time() -> None:
    writer = _RecordingWriter()
    registry = PedRegistry(writer, substeps_per_frame=5)
    ped, provider = _walking_ped()
    registry.spawn("p", ped)
    registry.begin_frame()
    for step in range(1, 5):
        registry.tick(step * _STEP, _STEP)
    assert writer.poses == 0
    registry.tick(5 * _STEP, _STEP)
    assert writer.poses == 1
    assert provider.dts == [pytest.approx(5 * _STEP)]
    assert writer.roots[-1][0] == pytest.approx(5 * _STEP)


def test_begin_frame_restarts_the_count_and_keeps_the_unwritten_time() -> None:
    writer = _RecordingWriter()
    registry = PedRegistry(writer, substeps_per_frame=5)
    ped, provider = _walking_ped()
    registry.spawn("p", ped)
    for _ in range(3):
        registry.tick(0.0, _STEP)
    registry.begin_frame()
    for _ in range(4):
        registry.tick(0.0, _STEP)
    assert writer.poses == 0
    registry.tick(0.0, _STEP)
    assert writer.poses == 1
    assert provider.dts == [pytest.approx(8 * _STEP)]
    assert writer.roots[-1][0] == pytest.approx(8 * _STEP)


def test_one_step_per_frame_writes_every_step() -> None:
    writer = _RecordingWriter()
    registry = PedRegistry(writer, substeps_per_frame=1)
    ped, provider = _walking_ped()
    registry.spawn("p", ped)
    for _ in range(3):
        registry.tick(0.0, _STEP)
    assert writer.poses == 3
    assert provider.dts == [pytest.approx(_STEP)] * 3
