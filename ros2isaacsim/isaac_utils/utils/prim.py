import os
from pxr import Usd, UsdGeom
from isaacsim.core.utils.prims import create_prim
from typing import Optional

import omni.usd
stage = omni.usd.get_context().get_stage()


def get_default_prim(stage: Usd.Stage) -> Optional[Usd.Prim]:
    if not stage:
        return None
    default_prim_obj = stage.GetDefaultPrim()
    if default_prim_obj and default_prim_obj.IsValid():
        return default_prim_obj
    return None


def create_prim_safe(
    prim_path: str,
    usd_path: str,
    **kwargs,
) -> Optional[Usd.Prim]:
    """
    create_prim wrapper that sets default_prim to first Xform if missing
    """

    temp_stage = Usd.Stage.Open(usd_path, Usd.Stage.LoadNone)

    if get_default_prim(temp_stage):
        del temp_stage
        return create_prim(
            prim_path=prim_path,
            prim_type='Xform',
            usd_path=usd_path,
            **kwargs,
        )

    for root_prim in temp_stage.GetPseudoRoot().GetChildren():
        if root_prim.IsA(UsdGeom.Xform):
            xform_prim = root_prim.GetPath()
            break
    else:
        raise RuntimeError(f"Failed to find root Xform in stage for USD path '{usd_path}'.")

    prim = create_prim(
        prim_path=prim_path,
        prim_type='Xform',
        **kwargs,
    )

    if not prim or not prim.IsValid():
        raise RuntimeError(f"Failed to create placeholder prim at '{prim_path}'.")

    prim.GetReferences().AddReference(
        assetPath=usd_path,
        primPath=xform_prim
    )
    return prim


def ensure_path(path: str):
    if not path or path == "":
        return

    # If prim already exists nothing to do
    if stage.GetPrimAtPath(path):
        return

    parent = os.path.dirname(path)
    # Stop recursion when reaching filesystem root or when parent equals path
    if parent and parent != path:
        ensure_path(parent)

    # Define an Xform prim at the requested path
    UsdGeom.Xform.Define(stage, path).GetPrim()
