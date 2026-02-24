from __future__ import annotations

import typing

import attrs
import geometry_msgs.msg
import isaacsim_msgs.msg
import numpy as np
from omni.isaac.core.articulations import Articulation
from isaacsim.core.prims import RigidPrim, XFormPrim
from isaacsim.core.utils.rotations import (euler_angles_to_quat,
                                             quat_to_euler_angles)
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, UsdPhysics


@attrs.define
class Translation:
    x: float
    y: float
    z: float

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
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        return

    target = None
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        target = Articulation(prim_path)

    if target is None and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        target = RigidPrim(prim_path)

    if target is None:
        target = XFormPrim(prim_path)

    if local:
        target.set_local_pose(
            translation=translation.tuple() if translation is not None else None,
            orientation=rotation.quat() if rotation is not None else None
        )
    else:
        target.set_world_pose(
            position=translation.tuple() if translation is not None else None,
            orientation=rotation.quat() if rotation is not None else None
        )


def rescale(
    prim_path: str,
    scale: Scale,
):
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        return

    xform_prim = XFormPrim(prim_path)

    xform_prim.set_local_scale(scale.tuple())
