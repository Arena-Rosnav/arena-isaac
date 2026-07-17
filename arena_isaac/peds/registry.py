"""Keyed collection of Peds, ticked once per sim step."""

from __future__ import annotations

from peds.ped import Ped
from peds.write import SkelWriter


class PedRegistry:
    """Advances root motion for every spawned Ped, then hands poses to a writer."""

    def __init__(self, writer: SkelWriter) -> None:
        self._writer = writer
        self._peds: dict[str, Ped] = {}

    def spawn(self, sim_path: str, ped: Ped) -> None:
        if sim_path in self._peds:
            import carb

            carb.log_warn(f"peds: overwriting existing ped at {sim_path}")
        self._peds[sim_path] = ped

    def get(self, sim_path: str) -> Ped | None:
        return self._peds.get(sim_path)

    def remove(self, sim_path: str) -> None:
        if self._peds.pop(sim_path, None) is None:
            import carb

            carb.log_warn(f"peds: remove called for unknown ped {sim_path}")

    def tick(self, sim_time: float, dt: float) -> None:
        """Compute root motion and write each provider's evaluated pose."""
        for ped in self._peds.values():
            position, orientation = ped.desired_root(dt)
            pose = ped.provider.evaluate(sim_time, dt)
            self._writer.set_root(ped, position, orientation)
            self._writer.write(ped, pose)
