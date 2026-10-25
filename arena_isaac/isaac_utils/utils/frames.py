from pxr import Usd, UsdGeom


def link(robot: Usd.Prim, name: str) -> Usd.Prim | None:
    """The first prim named `name` under `robot`."""
    return next((prim for prim in Usd.PrimRange(robot) if prim.GetName() == name), None)


def relative_pose(frame: Usd.Prim, prim: Usd.Prim) -> tuple[list[float], list[float]]:
    """Current pose of `prim` in the frame of `frame`, translation and xyzw quaternion."""
    cache = UsdGeom.XformCache()
    pose = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(frame).GetInverse()
    quat = pose.RemoveScaleShear().ExtractRotationQuat()
    return list(pose.ExtractTranslation()), [*quat.GetImaginary(), quat.GetReal()]
