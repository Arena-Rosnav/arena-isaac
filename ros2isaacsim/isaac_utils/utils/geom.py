import typing

import attrs
import geometry_msgs.msg
import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.utils.rotations import (euler_angles_to_quat,
                                             quat_to_euler_angles)
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf, UsdPhysics


@attrs.define
class Translation:
    x: float
    y: float
    z: float

    def tuple(self) -> typing.Tuple[float, float, float]:
        return self.x, self.y, self.z

    def Vec3d(self) -> Gf.Vec3d:
        return Gf.Vec3d(self.x, self.y, self.z)

    @classmethod
    def parse(cls, values: geometry_msgs.msg.Point | typing.Sequence[float]) -> "Translation":
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
    def parse(cls, values: geometry_msgs.msg.Quaternion | typing.Sequence[float]) -> "Rotation":
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


def move(
    prim_path: str,
    translation: Translation,
    rotation: Rotation,
):
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        return

    target = None
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        target = Articulation(prim_path)

    if target is not None:  # i am an articulation
        dof = target.num_dof
        target.set_joint_positions(positions=np.zeros(dof, dtype="float32"))
        target.set_joint_velocities(velocities=np.zeros(dof, dtype="float32"))
        target.wake_up()

    elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
        target = RigidPrim(prim_path)

    if target is not None:  # i am an articulation / rigidbody
        target.set_linear_velocity([0, 0, 0])
        target.set_angular_velocity([0, 0, 0])

    if target is None:
        target = XFormPrim(prim_path)

    target.set_world_pose(position=translation.tuple(), orientation=rotation.quat())
