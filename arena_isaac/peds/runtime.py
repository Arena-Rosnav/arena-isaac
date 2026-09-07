"""Runtime glue between the pedestrian services and the UsdSkel ped registry.

Instantiated once from run_isaacsim after the World exists. A single physics
callback drives registry.tick every step. The four pedestrian services call the
module-level spawn/update/move/delete helpers, which forward to the singleton.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import omni.usd
from isaacsim.core.utils.prims import create_prim, delete_prim
from pxr import Gf, Sdf, Usd, UsdSkel, Vt

from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path
from peds.cache import convert_cached
from peds.ped import Ped
from peds.providers.external import ExternalPoseProvider
from peds.registry import PedRegistry
from peds.write import anim_prim_path, make_writer

if TYPE_CHECKING:
    from isaacsim.core.api import World

_PHYSICS_CALLBACK = "peds/tick"


class PedRuntime:
    """Owns the registry, the shared writer, and the per-ped wire providers."""

    def __init__(self, world: World) -> None:
        self._world = world
        self._writer = make_writer()
        self._registry = PedRegistry(self._writer)
        self._externals: dict[str, ExternalPoseProvider] = {}
        world.add_physics_callback(_PHYSICS_CALLBACK, self._on_physics)

    def _on_physics(self, dt: float) -> None:
        self._registry.tick(float(self._world.current_time), float(dt))

    def spawn(self, name: str, position: np.ndarray, orientation: np.ndarray, model_source: str) -> None:
        """Reference the cached actor, author a per-ped SkelAnimation, and register it.

        Raises on any failure so the service maps it to FAILED_CREATE.
        """
        cache_dir = Path(convert_cached(model_source))
        meta = json.loads((cache_dir / "meta.json").read_text())
        joint_order = tuple(str(joint) for joint in meta["joints"])
        neutral_rotations = np.asarray(meta["neutral"]["rotations_xyzw"], dtype=float)
        neutral_translations = np.asarray(meta["neutral"]["translations"], dtype=float)

        provider = ExternalPoseProvider(joint_order, neutral_rotations, neutral_translations)

        prim_path = world_path(name)
        ensure_path(os.path.dirname(prim_path))
        create_prim(prim_path, "Xform", usd_path=str(cache_dir / "character.usda"))
        self._author_animation(prim_path, joint_order, neutral_translations)

        ped = Ped(sim_path=name, prim_path=prim_path, provider=provider)
        ped.teleport(np.asarray(position, dtype=float), np.asarray(orientation, dtype=float))

        self._registry.spawn(name, ped)
        self._externals[name] = provider
        self._writer.set_root(ped, ped.position, ped.orientation)

    def update(
        self,
        name: str,
        position: tuple[float, float, float],
        velocity: tuple[float, float],
        animation_state: int,
        joint_names: list[str],
        joint_positions: list[float],
        stamp_sec: float,
    ) -> bool:
        """Dead-reckoned per-tick command plus the wire's joint angles.

        animation_state is consumed upstream by the gait source that authors
        the wire angles, it selects nothing here.
        """
        ped = self._registry.get(name)
        if ped is None:
            return False

        ped.update_command(
            np.asarray(position, dtype=float),
            np.asarray(velocity, dtype=float),
            float(self._world.current_time),
            stamp_sec=stamp_sec,
        )
        if joint_names and (external := self._externals.get(name)) is not None:
            external.push(stamp_sec, joint_names, joint_positions)
        return True

    def move(self, name: str, position: tuple[float, float, float], orientation: tuple[float, float, float, float]) -> bool:
        """Hard teleport: reset gait phase and write the root pose immediately."""
        ped = self._registry.get(name)
        if ped is None:
            return False
        ped.teleport(np.asarray(position, dtype=float), np.asarray(orientation, dtype=float))
        self._writer.set_root(ped, ped.position, ped.orientation)
        return True

    def delete(self, name: str) -> bool:
        ped = self._registry.get(name)
        if ped is None:
            return False
        self._registry.remove(name)
        self._externals.pop(name, None)
        delete_prim(ped.prim_path)
        return True

    def _author_animation(self, prim_path: str, joint_order: tuple[str, ...], rest_translations: np.ndarray) -> None:
        """Author a local SkelAnimation and bind it as the skeleton's animation source."""
        stage = omni.usd.get_context().get_stage()

        anim_path = anim_prim_path(prim_path)
        anim = UsdSkel.Animation.Define(stage, anim_path)
        anim.CreateJointsAttr(Vt.TokenArray(list(joint_order)))

        anim.CreateTranslationsAttr(Vt.Vec3fArray([Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])) for t in rest_translations]))
        anim.CreateRotationsAttr(Vt.QuatfArray([Gf.Quatf(1.0, 0.0, 0.0, 0.0) for _ in joint_order]))
        anim.CreateScalesAttr(Vt.Vec3hArray([Gf.Vec3h(1.0, 1.0, 1.0) for _ in joint_order]))

        skeleton = _find_skeleton(stage, prim_path)
        if skeleton is None:
            raise ValueError(f"peds: no Skeleton found under {prim_path}")
        binding = UsdSkel.BindingAPI.Apply(skeleton.GetPrim())
        binding.CreateAnimationSourceRel().SetTargets([Sdf.Path(anim_path)])


def _find_skeleton(stage: Usd.Stage, root_path: str) -> UsdSkel.Skeleton | None:
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        skeleton = UsdSkel.Skeleton(prim)
        if skeleton:
            return skeleton
    return None


_runtime: PedRuntime | None = None


def initialize(world: World) -> None:
    global _runtime
    _runtime = PedRuntime(world)


def _get() -> PedRuntime:
    if _runtime is None:
        raise RuntimeError("peds runtime not initialized")
    return _runtime


def spawn(name: str, position: np.ndarray, orientation: np.ndarray, model_source: str) -> None:
    _get().spawn(name, position, orientation, model_source)


def update(
    name: str,
    position: tuple[float, float, float],
    velocity: tuple[float, float],
    animation_state: int,
    joint_names: list[str],
    joint_positions: list[float],
    stamp_sec: float,
) -> bool:
    return _get().update(name, position, velocity, animation_state, joint_names, joint_positions, stamp_sec)


def move(name: str, position: tuple[float, float, float], orientation: tuple[float, float, float, float]) -> bool:
    return _get().move(name, position, orientation)


def delete(name: str) -> bool:
    return _get().delete(name)
