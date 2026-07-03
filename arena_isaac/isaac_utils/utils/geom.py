from __future__ import annotations

import threading
import typing

import attrs
import carb
import geometry_msgs.msg
import numpy as np
from isaacsim.core.experimental.prims import RigidPrim, XformPrim, Articulation
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from pxr import Gf, Usd, UsdGeom, UsdPhysics

import isaacsim_msgs.msg
from isaac_utils.graphs import physics_engine
from isaac_utils.utils.prim import resolve_paths, resolve_prim


_robot_articulation_registry: dict[str, str] = {}
_robot_articulation_registry_lock = threading.RLock()


def _normalize_prim_path(prim_path: str) -> str:
    return prim_path.rstrip('/') or '/'


def register_robot(robot_prim_path: str, articulation_prim_path: str):
    robot_path = _normalize_prim_path(robot_prim_path)
    articulation_path = _normalize_prim_path(articulation_prim_path)

    with _robot_articulation_registry_lock:
        _robot_articulation_registry[robot_path] = articulation_path


def unregister_robot(prim_path: str):
    normalized_path = _normalize_prim_path(prim_path)

    with _robot_articulation_registry_lock:
        _robot_articulation_registry.pop(normalized_path, None)
        to_remove = [
            robot_path for robot_path, articulation_path in _robot_articulation_registry.items()
            if articulation_path == normalized_path
        ]
        for robot_path in to_remove:
            _robot_articulation_registry.pop(robot_path, None)


def _resolve_robot(prim_path: str) -> str:
    normalized_path = _normalize_prim_path(prim_path)

    with _robot_articulation_registry_lock:
        return _robot_articulation_registry.get(normalized_path, normalized_path)


@attrs.define
class Translation:
    x: float
    y: float
    z: float

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __add__(self, other: Translation) -> Translation:
        return Translation(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
        )

    def __mul__(self, other: float) -> Translation:
        return Translation(
            x=self.x * other,
            y=self.y * other,
            z=self.z * other,
        )

    def tuple(self) -> typing.Tuple[float, float, float]:
        return self.x, self.y, self.z

    def Vec3d(self) -> Gf.Vec3d:
        return Gf.Vec3d(self.x, self.y, self.z)

    @classmethod
    def parse(cls, values: geometry_msgs.msg.Point | typing.Sequence[float]) -> Translation:
        if isinstance(values, geometry_msgs.msg.Point):
            return cls(
                x=values.x,
                y=values.y,
                z=values.z,
            )

        if len(values) == 3:
            return cls(*values)

        if len(values) == 2:
            return cls(
                values[0],
                values[1],
                0.0,
            )

        raise ValueError(
            f"Translation must be [x,y] or [x,y,z], got {values}"
        )


@attrs.define
class Rotation:
    w: float
    x: float
    y: float
    z: float

    def __iter__(self):
        yield self.w
        yield self.x
        yield self.y
        yield self.z

    def __add__(self, other: Rotation) -> Rotation:
        return Rotation(
            w=self.w + other.w,
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
        )

    def __mul__(self, other: Rotation) -> Rotation:
        return Rotation(
            w=self.w * other.w,
            x=self.x * other.x,
            y=self.y * other.y,
            z=self.z * other.z,
        )

    def quat(self, convention: str = 'wxyz') -> typing.List[float]:
        return [float(getattr(self, axis)) for axis in convention if axis in 'wxyz']

    def euler(self, convention: str = 'xyz') -> typing.List[float]:
        x, y, z = quat_to_euler_angles(self.quat())
        axes: dict[str, float] = dict(
            x=x,
            y=y,
            z=z,
        )
        return [axes[axis] for axis in convention if axis in 'xyz']

    def Quatd(self) -> Gf.Quatd:
        return Gf.Quatd(self.w, self.x, self.y, self.z)

    @classmethod
    def parse(cls, values: geometry_msgs.msg.Quaternion | typing.Sequence[float]) -> Rotation:
        if isinstance(values, geometry_msgs.msg.Quaternion):
            return cls(
                x=values.x,
                y=values.y,
                z=values.z,
                w=values.w,
            )

        if len(values) == 4:
            return cls(
                *values
            )

        if len(values) == 3:
            return cls(
                *euler_angles_to_quat(values)
            )

        raise ValueError(
            f"Rotation must be [x,y,z] or [w,x,y,z], got {values}"
        )


@attrs.define
class Scale:
    x: float
    y: float
    z: float

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __add__(self, other: Scale) -> Scale:
        return Scale(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
        )

    def __mul__(self, other: float) -> Scale:
        return Scale(
            x=self.x * other,
            y=self.y * other,
            z=self.z * other,
        )

    def tuple(self) -> typing.Tuple[float, float, float]:
        return self.x, self.y, self.z

    def Vec3d(self) -> Gf.Vec3d:
        return Gf.Vec3d(self.x, self.y, self.z)

    @classmethod
    def parse(cls, values: isaacsim_msgs.msg.Scale | float | typing.Sequence[float]) -> Scale:
        if isinstance(values, isaacsim_msgs.msg.Scale):
            return cls(
                x=values.x,
                y=values.y,
                z=values.z,
            )

        if isinstance(values, (int, float)):
            return cls(
                x=float(values),
                y=float(values),
                z=float(values),
            )

        if len(values) == 3:
            return cls(*values)

        if len(values) == 2:
            return cls(
                values[0],
                values[1],
                1.0,
            )

        raise ValueError(
            f"Scale must be a single float or [x,y] or [x,y,z], got {values}"
        )


def move(
    prim_path: str,
    *,
    translation: Translation | None = None,
    rotation: Rotation | None = None,
    local: bool = False,
):
    prim_path = _resolve_robot(prim_path)
    prim = resolve_prim(prim_path)
    if prim is None:
        return

    target = None
    if physics_engine() == 'newton':
        # newton rebuilds its model from authored usd on every unpause, and pose
        # writes through the tensor views are lost with the rebuilt state, so
        # teleports must land in the usd xform to survive
        target = XformPrim(prim_path, reset_xform_op_properties=True)

    if target is None and all(p.HasAPI(UsdPhysics.ArticulationRootAPI) for p in prim.prims):
        try:
            target = Articulation(prim_path)
        except Exception:
            target = None

    if target is None and all(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prim.prims):
        try:
            target = RigidPrim(prim_path)
        except Exception:
            target = None

    if target is None:
        target = XformPrim(prim_path)

    if local:
        target.set_local_poses(
            np.array(np.atleast_2d(translation.tuple())) if translation is not None else None,
            np.array(np.atleast_2d(rotation.quat())) if rotation is not None else None
        )
    else:
        target.set_world_poses(
            np.array(np.atleast_2d(translation.tuple())) if translation is not None else None,
            np.array(np.atleast_2d(rotation.quat())) if rotation is not None else None
        )


def get_world_translation(prim_path: str) -> Translation | None:
    prim_path = _resolve_robot(prim_path)
    prim = resolve_prim(prim_path)
    if prim is None:
        return None

    try:
        transform = UsdGeom.Xformable(prim.prims[0]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = transform.ExtractTranslation()
        return Translation(float(translation[0]), float(translation[1]), float(translation[2]))
    except Exception:
        return None


def rescale(
    prim_path: str,
    scale: Scale,
):
    if not resolve_paths(prim_path):
        return

    xform_prim = XformPrim(prim_path)

    xform_prim.set_local_scales(np.array(np.atleast_2d(scale.tuple())))
