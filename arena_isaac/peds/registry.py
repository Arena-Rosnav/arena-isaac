"""Keyed collection of Peds, ticked every physics step and posed once per rendered frame."""

from __future__ import annotations

from peds.ped import Ped
from peds.write import SkelWriter


class PedRegistry:
    """Advances root motion for every spawned Ped, then hands poses to a writer."""

    def __init__(self, writer: SkelWriter, substeps_per_frame: int) -> None:
        self._writer = writer
        self._peds: dict[str, Ped] = {}
        self._substeps_per_frame = substeps_per_frame
        self._substep = 0
        self._pending_dt = 0.0

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

    def begin_frame(self) -> None:
        """Count the coming physics steps as one rendered frame."""
        self._substep = 0

    def tick(self, sim_time: float, dt: float) -> None:
        """On the last physics step of a rendered frame, write each ped's root and pose over the time since its last write."""
        self._pending_dt += dt
        self._substep += 1
        if self._substep < self._substeps_per_frame:
            return
        dt, self._pending_dt, self._substep = self._pending_dt, 0.0, 0
        for ped in self._peds.values():
            position, orientation = ped.desired_root(dt)
            pose = ped.provider.evaluate(sim_time, dt)
            if ped.soles is not None:
                ped.ground_z = -ped.soles.height(pose)
            self._writer.set_root(ped, position, orientation)
            self._writer.write(ped, pose)
