from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("pxr")

from peds.ground import Soles
from peds.ped import Ped
from peds.providers.base import JointPose
from peds.registry import PedRegistry
from peds.support import SIT_ON, Support


class _Provider:
    """A pose whose one joint (the foot) sits `lift` m above the skeleton origin."""

    def __init__(self) -> None:
        self.lift = 0.0

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        return JointPose(rotations=np.array([[0.0, 0.0, 0.0, 1.0]]), translations=np.array([[0.0, 0.0, self.lift]]))


class _Writer:
    def __init__(self) -> None:
        self.heights: list[float] = []

    def set_root(self, ped: Ped, position: np.ndarray, orientation: np.ndarray) -> None:
        self.heights.append(float(position[2]) + ped.ground_z)

    def write(self, ped: Ped, pose: JointPose) -> None:
        pass


def test_soles_plant_a_ped_on_its_feet_but_not_one_resting_on_a_seat() -> None:
    writer, provider = _Writer(), _Provider()
    ped = Ped(sim_path="p", prim_path="/World/p", provider=provider, soles=Soles([-1], [0], np.zeros((1, 3))))
    registry = PedRegistry(writer)  # type: ignore[arg-type]
    registry.spawn("p", ped)

    provider.lift = 0.02  # standing: the soles 2 cm above the origin are planted on the floor
    registry.tick(0.0, 0.1)
    assert ped.ground_z == pytest.approx(-0.02)

    ped.support = Support(SIT_ON, 0.45)  # seated: the folded legs lift the feet, the seat sets the root
    provider.lift = 0.4
    registry.tick(0.1, 0.1)
    assert ped.ground_z == pytest.approx(-0.02)

    ped.support = None  # up again
    registry.tick(0.2, 0.1)
    assert ped.ground_z == pytest.approx(-0.4)
