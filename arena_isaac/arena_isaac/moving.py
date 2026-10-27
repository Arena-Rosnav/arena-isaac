"""Moving boxes as kinematic rigid bodies whose PhysX kinematic targets advance by their velocity every physics step."""

from __future__ import annotations

from collections.abc import Sequence

import carb
import numpy as np
import omni.physics.tensors
import omni.usd
from isaacsim.core.experimental.prims import XformPrim

from arena_isaac import run_after_tick_queue
from isaac_utils.utils.kinematic import KinematicBoxes

_RETRY_STEPS = 60


class MovingBoxes:
    """Registry of moving boxes plus the rigid body view that drives them."""

    def __init__(self) -> None:
        self._boxes = KinematicBoxes()
        self._sim_view: omni.physics.tensors.SimulationView | None = None
        self._view: omni.physics.tensors.RigidBodyView | None = None
        self._paths: tuple[str, ...] = ()
        self._built = -1
        self._warned = -1
        self._cooldown = 0
        self._rebuild_queued = False
        self._shown: XformPrim | None = None
        self._shown_revision = -1

    def add(self, path: str, position: Sequence[float], orientation: Sequence[float], velocity: Sequence[float]) -> None:
        self._boxes.add(path, position, orientation, velocity)

    def forget_under(self, path: str) -> None:
        self._boxes.forget_under(path)

    def set_velocity(self, path: str, velocity: Sequence[float]) -> bool:
        return self._boxes.set_velocity(path, velocity)

    def teleport(self, path: str, position: Sequence[float], orientation: Sequence[float]) -> bool:
        """Set the pose of a moving box, False when path is not one."""
        if not self._boxes.teleport(path, position, orientation):
            return False
        view = self._live_view()
        if view is not None and path in self._paths:
            try:
                view.set_transforms(self._boxes.transforms(self._paths), np.array([self._paths.index(path)], dtype=np.uint32))
                return True
            except Exception as error:
                carb.log_warn(f"arena: moving box teleport of {path} failed: {error}")
                self._view = None
        XformPrim(path).set_world_poses(np.atleast_2d(position), np.atleast_2d(orientation))
        return True

    def _live_view(self) -> omni.physics.tensors.RigidBodyView | None:
        if self._sim_view is None or not self._sim_view.is_valid:
            return None
        return self._view

    def tick(self, dt: float) -> None:
        if not len(self._boxes):
            return
        self._boxes.step(float(dt))
        view = self._live_view()
        if view is not None:
            try:
                view.set_kinematic_targets(self._boxes.transforms(self._paths), np.arange(len(self._paths), dtype=np.uint32))
            except Exception as error:
                carb.log_warn(f"arena: moving box kinematic targets failed: {error}")
                self._view = None
        self._show()
        if self._rebuild_queued or (self._live_view() is not None and self._built == self._boxes.revision and len(self._paths) == len(self._boxes)):
            return
        if self._cooldown > 0:
            self._cooldown -= 1
            return
        self._rebuild_queued = True
        run_after_tick_queue.put(self._rebuild)

    def _show(self) -> None:
        """Author every box's pose on its prim, the pose the renderer draws."""
        try:
            if self._shown is None or self._shown_revision != self._boxes.revision:
                self._shown = XformPrim(list(self._boxes.paths))
                self._shown_revision = self._boxes.revision
            rows = self._boxes.transforms(self._boxes.paths)
            self._shown.set_world_poses(rows[:, :3], rows[:, [6, 3, 4, 5]])
        except Exception as error:
            self._shown = None
            if self._warned != self._boxes.revision:
                self._warned = self._boxes.revision
                carb.log_warn(f"arena: moving box poses not shown: {error}")

    def _rebuild(self) -> None:
        self._rebuild_queued = False
        self._view = None
        self._paths = ()
        if not len(self._boxes):
            return
        revision = self._boxes.revision
        try:
            sim_view = self._sim_view
            if sim_view is None or not sim_view.is_valid:
                sim_view = self._sim_view = omni.physics.tensors.create_simulation_view('numpy', stage_id=omni.usd.get_context().get_stage_id())
            view = sim_view.create_rigid_body_view(list(self._boxes.paths))
            if view is None or not view.check():
                raise RuntimeError("rigid body view is not valid")
            paths = tuple(view.prim_paths)
            view.set_transforms(self._boxes.transforms(paths), np.arange(len(paths), dtype=np.uint32))
        except Exception as error:
            self._cooldown = _RETRY_STEPS
            if self._warned != revision:
                self._warned = revision
                carb.log_warn(f"arena: moving boxes have no rigid body view yet: {error}")
            return
        self._view, self._paths, self._built = view, paths, revision
        if len(paths) != len(self._boxes):
            self._cooldown = _RETRY_STEPS


moving_boxes = MovingBoxes()

__all__ = ['MovingBoxes', 'moving_boxes']
