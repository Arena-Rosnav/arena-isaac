from __future__ import annotations

import numpy as np
import pytest

from peds.ped import Ped
from peds.providers.base import JointPose


class _StaticProvider:
    """A PoseProvider with no reset_phase, exercising the duck-typed teleport path."""

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        return JointPose(rotations=np.zeros((1, 4)), translations=np.zeros((1, 3)))


class _ResettableProvider(_StaticProvider):
    def __init__(self) -> None:
        self.reset_calls = 0

    def reset_phase(self) -> None:
        self.reset_calls += 1


_IDENTITY = [0.0, 0.0, 0.0, 1.0]


def _yaw_quat(yaw: float) -> list[float]:
    return [0.0, 0.0, float(np.sin(yaw / 2.0)), float(np.cos(yaw / 2.0))]


def _ped() -> Ped:
    return Ped(sim_path="p", prim_path="/World/p", provider=_StaticProvider())


def test_update_command_seeds_age_from_stamp() -> None:
    ped = _ped()
    ped.update_command(orientation=_IDENTITY, position=[1.0, 2.0, 0.0], velocity=[0.5, 0.0], now=10.0, stamp_sec=9.5)
    assert ped.command_age == pytest.approx(0.5)


def test_update_command_zero_stamp_is_unstamped() -> None:
    ped = _ped()
    ped.update_command(orientation=_IDENTITY, position=[1.0, 2.0, 0.0], velocity=[0.5, 0.0], now=10.0)
    assert ped.command_age == 0.0


def test_update_command_negative_latency_clamped_to_zero() -> None:
    ped = _ped()
    ped.update_command(orientation=_IDENTITY, position=[1.0, 2.0, 0.0], velocity=[0.5, 0.0], now=10.0, stamp_sec=12.0)
    assert ped.command_age == 0.0


def test_desired_root_extrapolates_position() -> None:
    ped = _ped()
    ped.update_command(orientation=_IDENTITY, position=[0.0, 0.0, 0.0], velocity=[1.0, 0.0], now=0.0)
    position, _ = ped.desired_root(dt=0.1)
    np.testing.assert_allclose(position[:2], [0.1, 0.0], atol=1e-9)


def test_desired_root_extrapolation_capped_at_max() -> None:
    ped = _ped()
    ped.update_command(orientation=_IDENTITY, position=[0.0, 0.0, 0.0], velocity=[1.0, 0.0], now=0.0)
    position = ped.position
    for _ in range(20):
        position, _ = ped.desired_root(dt=0.1)
    np.testing.assert_allclose(position[:2], [Ped.MAX_EXTRAPOLATION, 0.0], atol=1e-9)
    assert ped.command_age > Ped.MAX_EXTRAPOLATION


def test_desired_root_holds_z_from_current_state() -> None:
    ped = _ped()
    ped.position = np.array([0.0, 0.0, 1.7])
    ped.update_command(orientation=_IDENTITY, position=[1.0, 0.0, 0.0], velocity=[0.0, 0.0], now=0.0)
    position, _ = ped.desired_root(dt=0.1)
    assert position[2] == pytest.approx(1.7)


def test_desired_root_takes_orientation_verbatim() -> None:
    ped = _ped()
    commanded = _yaw_quat(np.pi / 2.0)
    ped.update_command(orientation=commanded, position=[0.0, 0.0, 0.0], velocity=[0.0, 1.0], now=0.0)
    _, orientation = ped.desired_root(dt=0.1)
    np.testing.assert_allclose(orientation, commanded, atol=1e-9)


def test_desired_root_orientation_ignores_travel_direction() -> None:
    ped = _ped()
    facing = _yaw_quat(np.pi)
    ped.update_command(orientation=facing, position=[0.0, 0.0, 0.0], velocity=[1.0, 0.0], now=0.0)
    _, orientation = ped.desired_root(dt=0.1)
    np.testing.assert_allclose(orientation, facing, atol=1e-9)


def test_desired_root_tracks_orientation_while_stationary() -> None:
    ped = _ped()
    turned = _yaw_quat(np.pi / 4.0)
    ped.update_command(orientation=turned, position=[0.0, 0.0, 0.0], velocity=[0.0, 0.0], now=0.0)
    _, orientation = ped.desired_root(dt=0.1)
    np.testing.assert_allclose(orientation, turned, atol=1e-9)


def test_desired_root_without_command_holds_position() -> None:
    ped = _ped()
    ped.position = np.array([3.0, 4.0, 0.0])
    position, _ = ped.desired_root(dt=0.1)
    np.testing.assert_allclose(position, [3.0, 4.0, 0.0])


def test_teleport_clears_command_state() -> None:
    ped = _ped()
    ped.update_command(orientation=_IDENTITY, position=[1.0, 1.0, 0.0], velocity=[1.0, 0.0], now=0.0)
    ped.teleport(position=[5.0, 5.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    assert ped.command_position is None
    np.testing.assert_allclose(ped.command_velocity, [0.0, 0.0, 0.0])
    assert ped.command_age == 0.0
    np.testing.assert_allclose(ped.position, [5.0, 5.0, 0.0])
    np.testing.assert_allclose(ped.orientation, [0.0, 0.0, 0.0, 1.0])


def test_teleport_resets_gait_phase_when_provider_supports_it() -> None:
    provider = _ResettableProvider()
    ped = Ped(sim_path="p", prim_path="/World/p", provider=provider)
    ped.teleport(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    assert provider.reset_calls == 1


def test_teleport_without_resettable_provider_does_not_error() -> None:
    ped = _ped()
    ped.teleport(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
