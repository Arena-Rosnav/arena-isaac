"""Isaac side of the viewport contract: the active viewport camera and entity poses."""

from __future__ import annotations

import math

import carb
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom

from isaac_utils.utils.prim import resolve_prim

from .controller import Pose, Quat, q_conj, q_mul

# Body (+X forward, +Z up) -> USD camera (-Z forward, +Y up).
_USD_FROM_BODY: Quat = (0.5, 0.5, -0.5, -0.5)


def _to_usd(orientation: Quat) -> Quat:
    return q_mul(orientation, _USD_FROM_BODY)


def _from_usd(orientation: Quat) -> Quat:
    return q_mul(orientation, q_conj(_USD_FROM_BODY))


def entity_pose(entity: str) -> Pose | None:
    """World pose of a prim, None when the path does not resolve."""
    prim = resolve_prim(entity)
    if prim is None:
        return None
    transform = UsdGeom.Xformable(prim.prims[0]).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = transform.ExtractTranslation()
    rotation = transform.RemoveScaleShear().ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return Pose(
        (float(translation[0]), float(translation[1]), float(translation[2])),
        (float(rotation.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])),
    )


class CameraBackend:
    """The active viewport camera, resolved lazily so startup order does not matter."""

    def __init__(self) -> None:
        self._camera_path: str | None = None

    def _resolve(self) -> str | None:
        if self._camera_path is not None:
            return self._camera_path
        viewport = get_active_viewport()
        if viewport is None:
            return None
        self._camera_path = str(viewport.camera_path)
        carb.log_info(f"arena: viewport camera resolved at {self._camera_path}")
        return self._camera_path

    def _usd_camera(self) -> UsdGeom.Camera | None:
        path = self._resolve()
        if path is None:
            return None
        prim = resolve_prim(path)
        return UsdGeom.Camera(prim.prims[0]) if prim is not None else None

    def world_pose(self) -> Pose | None:
        path = self._resolve()
        if path is None:
            return None
        pose = entity_pose(path)
        return Pose(pose.position, _from_usd(pose.orientation)) if pose is not None else None

    def set_world_pose(self, pose: Pose) -> None:
        """Write the pose as a single transform op, the form Kit's viewport camera uses."""
        path = self._resolve()
        if path is None:
            return
        prim = resolve_prim(path)
        if prim is None:
            return
        xform = UsdGeom.Xformable(prim.prims[0])
        w, x, y, z = _to_usd(pose.orientation)
        local = Gf.Matrix4d().SetTransform(
            Gf.Rotation(Gf.Quatd(w, Gf.Vec3d(x, y, z))),
            Gf.Vec3d(*pose.position),
        )
        # the camera sits at the stage root, but compose out any parent regardless
        parent = xform.ComputeParentToWorldTransform(Usd.TimeCode.Default())
        if parent != Gf.Matrix4d(1.0):
            local = local * parent.GetInverse()
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                op.Set(local)
                return
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(local)

    def set_hfov(self, fov: float) -> None:
        """Set the horizontal field of view in radians, solved into a focal length."""
        camera = self._usd_camera()
        if camera is None:
            return
        aperture = float(camera.GetHorizontalApertureAttr().Get())
        camera.GetFocalLengthAttr().Set(aperture / (2.0 * math.tan(fov / 2.0)))

    def set_projection(self, projection: str) -> None:
        camera = self._usd_camera()
        if camera is None:
            return
        camera.GetProjectionAttr().Set(projection)
