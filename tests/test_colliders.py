from __future__ import annotations

import pytest

pytest.importorskip("pxr")

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from isaac_utils.utils.colliders import uninstance_colliders


def _instance(stage: Usd.Stage, path: str, prototype: str) -> Usd.Prim:
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddInternalReference(Sdf.Path(prototype))
    prim.SetInstanceable(True)
    return prim


def _stage() -> Usd.Stage:
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Prototypes/collision")
    UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, "/Prototypes/collision/mesh").GetPrim())
    UsdGeom.Xform.Define(stage, "/Prototypes/visual")
    UsdGeom.Cube.Define(stage, "/Prototypes/visual/mesh")
    UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage, "/World/robot/link").GetPrim())
    _instance(stage, "/World/robot/link/collision", "/Prototypes/collision")
    _instance(stage, "/World/robot/link/visual", "/Prototypes/visual")
    return stage


def test_collider_instances_become_plain_prims() -> None:
    stage = _stage()
    uninstance_colliders(stage.GetPrimAtPath("/World/robot"))
    collider = stage.GetPrimAtPath("/World/robot/link/collision/mesh")
    assert not stage.GetPrimAtPath("/World/robot/link/collision").IsInstance()
    assert collider.HasAPI(UsdPhysics.CollisionAPI)
    assert not collider.IsInstanceProxy()


def test_visual_instances_stay_instanced() -> None:
    stage = _stage()
    uninstance_colliders(stage.GetPrimAtPath("/World/robot"))
    assert stage.GetPrimAtPath("/World/robot/link/visual").IsInstance()


def test_colliders_outside_root_stay_instanced() -> None:
    stage = _stage()
    _instance(stage, "/World/other/collision", "/Prototypes/collision")
    uninstance_colliders(stage.GetPrimAtPath("/World/robot"))
    assert stage.GetPrimAtPath("/World/other/collision").IsInstance()
