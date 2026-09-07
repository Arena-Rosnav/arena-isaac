"""Writer abstraction: sinks a Ped's root pose and joint pose onto the stage.

The Protocol and NullWriter stay importable without pxr/usdrt/omni, so plain
tests can import them. The two real writers import their USD backends lazily
inside their methods.
"""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np

from peds.ped import Ped
from peds.providers.base import JointPose


def anim_prim_path(prim_path: str) -> str:
    """Per-ped SkelAnimation prim path, a deterministic child of the character root.

    Both the spawner (which authors the prim) and the writers (which resolve it)
    derive the anim path from the character root, so no extra state is needed.
    """
    return f"{prim_path}/SkelAnim"


class SkelWriter(Protocol):
    """USD-backed in the real writer, a test double elsewhere."""

    def write(self, ped: Ped, pose: JointPose) -> None: ...

    def set_root(self, ped: Ped, position: np.ndarray, orientation: np.ndarray) -> None: ...


class NullWriter:
    """Records write/set_root calls instead of touching USD, for tests."""

    def __init__(self) -> None:
        self.writes: list[tuple[Ped, JointPose]] = []
        self.roots: list[tuple[Ped, np.ndarray, np.ndarray]] = []

    def write(self, ped: Ped, pose: JointPose) -> None:
        self.writes.append((ped, pose))

    def set_root(self, ped: Ped, position: np.ndarray, orientation: np.ndarray) -> None:
        self.roots.append((ped, position, orientation))


class _RootWriter:
    """Shared root-pose sink: authors the character root Xform via plain pxr USD.

    With no AnimationGraph nobody owns the root in Fabric, so pxr xformOp writes
    reach the render through the same USD to Fabric notice sync the spike verified
    for the plain-USD joint path.
    """

    def set_root(self, ped: Ped, position: np.ndarray, orientation: np.ndarray) -> None:
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(ped.prim_path)
        if not prim or not prim.IsValid():
            return

        xformable = UsdGeom.Xformable(prim)
        translate_op = None
        orient_op = None
        for op in xformable.GetOrderedXformOps():
            op_type = op.GetOpType()
            if op_type == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif op_type == UsdGeom.XformOp.TypeOrient:
                orient_op = op
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        if orient_op is None:
            orient_op = xformable.AddOrientOp()

        translate_op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        real, imag_i, imag_j, imag_k = (
            float(orientation[3]),
            float(orientation[0]),
            float(orientation[1]),
            float(orientation[2]),
        )
        precision = orient_op.GetPrecision()
        if precision == UsdGeom.XformOp.PrecisionDouble:
            orient_op.Set(Gf.Quatd(real, imag_i, imag_j, imag_k))
        elif precision == UsdGeom.XformOp.PrecisionHalf:
            orient_op.Set(Gf.Quath(real, imag_i, imag_j, imag_k))
        else:
            orient_op.Set(Gf.Quatf(real, imag_i, imag_j, imag_k))


class UsdrtSkelWriter(_RootWriter):
    """Primary path: writes joint rotations/translations as Fabric attrs via usdrt.

    Handles are resolved lazily per ped and revalidated on every write, so a ped
    respawned at the same prim path re-binds transparently.
    """

    def __init__(self) -> None:
        import omni.usd
        import usdrt

        self._usdrt = usdrt
        self._rt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        self._handles: dict[str, tuple[Any, Any]] = {}

    def write(self, ped: Ped, pose: JointPose) -> None:
        handle = self._handle(ped)
        if handle is None:
            return
        usdrt = self._usdrt
        rotations, translations = pose.rotations, pose.translations
        joints = rotations.shape[0]
        handle[0].Set(
            usdrt.Vt.QuatfArray(
                [
                    usdrt.Gf.Quatf(
                        float(rotations[j, 3]),
                        float(rotations[j, 0]),
                        float(rotations[j, 1]),
                        float(rotations[j, 2]),
                    )
                    for j in range(joints)
                ]
            )
        )
        handle[1].Set(usdrt.Vt.Vec3fArray([usdrt.Gf.Vec3f(float(translations[j, 0]), float(translations[j, 1]), float(translations[j, 2])) for j in range(joints)]))

    def _handle(self, ped: Ped) -> tuple[Any, Any] | None:
        cached = self._handles.get(ped.sim_path)
        if cached is not None and cached[0].IsValid():
            return cached

        usdrt = self._usdrt
        rt_prim = self._rt_stage.GetPrimAtPath(anim_prim_path(ped.prim_path))
        if not rt_prim.IsValid():
            return None

        rotations_attr = rt_prim.GetAttribute("rotations")
        if not rotations_attr.IsValid():
            rotations_attr = rt_prim.CreateAttribute("rotations", usdrt.Sdf.ValueTypeNames.QuatfArray)
        translations_attr = rt_prim.GetAttribute("translations")
        if not translations_attr.IsValid():
            translations_attr = rt_prim.CreateAttribute("translations", usdrt.Sdf.ValueTypeNames.Vec3fArray)

        handle = (rotations_attr, translations_attr)
        self._handles[ped.sim_path] = handle
        return handle


class UsdSkelWriter(_RootWriter):
    """Verified fallback: writes joint rotations/translations as default USD values."""

    def __init__(self) -> None:
        self._handles: dict[str, tuple[Any, Any]] = {}

    def write(self, ped: Ped, pose: JointPose) -> None:
        handle = self._handle(ped)
        if handle is None:
            return
        from pxr import Gf, Vt

        rotations, translations = pose.rotations, pose.translations
        joints = rotations.shape[0]
        handle[0].Set(
            Vt.QuatfArray(
                [
                    Gf.Quatf(
                        float(rotations[j, 3]),
                        Gf.Vec3f(float(rotations[j, 0]), float(rotations[j, 1]), float(rotations[j, 2])),
                    )
                    for j in range(joints)
                ]
            )
        )
        handle[1].Set(Vt.Vec3fArray([Gf.Vec3f(float(translations[j, 0]), float(translations[j, 1]), float(translations[j, 2])) for j in range(joints)]))

    def _handle(self, ped: Ped) -> tuple[Any, Any] | None:
        cached = self._handles.get(ped.sim_path)
        if cached is not None and cached[0].GetPrim().IsValid():
            return cached

        import omni.usd
        from pxr import UsdSkel

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(anim_prim_path(ped.prim_path))
        if not prim or not prim.IsValid():
            return None
        anim = UsdSkel.Animation(prim)
        if not anim:
            return None

        handle = (anim.GetRotationsAttr(), anim.GetTranslationsAttr())
        self._handles[ped.sim_path] = handle
        return handle


def make_writer() -> SkelWriter:
    """Prefer the usdrt Fabric writer, fall back to the plain-USD writer once."""
    import carb

    try:
        return UsdrtSkelWriter()
    except Exception as exc:
        carb.log_warn(f"peds: usdrt writer unavailable ({exc!r}), using plain-USD writer")
        return UsdSkelWriter()
