"""Objects held at a fixed pose in a robot body's frame on every physics step."""

from __future__ import annotations

import attrs
import carb
import numpy as np
import omni.usd
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from pxr import Usd, UsdPhysics

from isaac_utils.utils import frames, pose_math


@attrs.define
class _Weld:
    body: str
    offset_position: np.ndarray
    offset_orientation: np.ndarray
    body_view: RigidPrim
    shown: XformPrim
    driven: RigidPrim | None
    colliders: dict[str, bool]


_welds: dict[str, _Weld] = {}


def _pose(view: RigidPrim | XformPrim) -> tuple[np.ndarray, np.ndarray]:
    positions, orientations = view.get_world_poses()
    return positions.numpy()[0], orientations.numpy()[0]


def _body_of(link: Usd.Prim) -> Usd.Prim | None:
    """The link itself or its nearest ancestor that is a rigid body."""
    prim = link
    while prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def _silence(obj: Usd.Prim) -> dict[str, bool]:
    """Switch off every collider under obj, their paths and former collisionEnabled values."""
    colliders: dict[str, bool] = {}
    for prim in Usd.PrimRange(obj):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        colliders[str(prim.GetPath())] = bool(enabled.Get()) if enabled.HasAuthoredValue() else True
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
    return colliders


def _restore(weld: _Weld) -> None:
    stage = omni.usd.get_context().get_stage()
    for path, enabled in weld.colliders.items():
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(enabled)


def attach(object_path: str, robot_path: str, link_name: str) -> bool:
    """Hold the object at its current pose relative to the rigid body that carries the robot's link, False when there is none."""
    stage = omni.usd.get_context().get_stage()
    robot = stage.GetPrimAtPath(robot_path)
    obj = stage.GetPrimAtPath(object_path)
    if not robot.IsValid() or not obj.IsValid():
        return False
    link = frames.link(robot, link_name)
    body = None if link is None else _body_of(link)
    if body is None:
        return False
    body_path = str(body.GetPath())
    body_view = RigidPrim(body_path)
    shown = XformPrim(object_path)
    driven = RigidPrim(object_path) if obj.HasAPI(UsdPhysics.RigidBodyAPI) else None
    offset = pose_math.relative(*_pose(body_view), *_pose(shown))
    if (held := _welds.pop(object_path, None)) is not None:
        _restore(held)
    _welds[object_path] = _Weld(body_path, *offset, body_view, shown, driven, _silence(obj))
    return True


def release(object_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """Stop holding the object, its world position and wxyz orientation, None when it was not held."""
    weld = _welds.pop(object_path, None)
    if weld is None:
        return None
    _restore(weld)
    return _pose(weld.shown)


def forget_under(path: str) -> None:
    """Drop every weld whose object or body is at or under path, a surviving object's colliders back on."""
    prefix = f'{path.rstrip("/")}/'
    for key in [key for key, weld in _welds.items() if key == path or key.startswith(prefix) or weld.body == path or weld.body.startswith(prefix)]:
        _restore(_welds.pop(key))


def clear() -> None:
    """Drop every weld, its object's colliders back on."""
    for weld in _welds.values():
        _restore(weld)
    _welds.clear()


def tick() -> None:
    """Place every welded object at its offset from its body's pose of this step."""
    for key, weld in list(_welds.items()):
        try:
            position, orientation = pose_math.compose(*_pose(weld.body_view), weld.offset_position, weld.offset_orientation)
            if weld.driven is None:
                weld.shown.set_world_poses(position[None], orientation[None])
            else:
                weld.driven.set_world_poses(position[None], orientation[None])
                weld.driven.set_velocities(np.zeros((1, 3)), np.zeros((1, 3)))
        except Exception as error:
            _restore(_welds.pop(key))
            carb.log_warn(f"arena: weld of {key} to {weld.body} dropped: {error}")
