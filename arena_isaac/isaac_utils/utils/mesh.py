"""Create simple meshes at runtime."""

import os

import omni.kit.commands
from isaacsim.core.experimental.prims import Prim
from pxr import UsdPhysics

import isaac_utils.utils.geom as geom
from isaac_utils.utils.prim import ensure_path, resolve_prim


def create_mesh(
    prim_path: str,
    mesh_type: str,
    *,
    position: geom.Translation | None = None,
    rotation: geom.Rotation | None = None,
    scale: geom.Scale | None = None,
    collide: bool = True,
    **kwargs: object,
):
    """Create a mesh prim of the given type."""

    ensure_path(os.path.dirname(prim_path))
    omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=mesh_type, prim_path=prim_path, select_new_prim=False, **kwargs)
    if collide:
        prim = resolve_prim(prim_path)
        if prim is not None:
            Prim.ensure_api(prim.prims, UsdPhysics.CollisionAPI)
    if position is not None:
        geom.move(
            prim_path,
            translation=position,
        )
    if rotation is not None:
        geom.move(
            prim_path,
            rotation=rotation,
        )
    if scale is not None:
        geom.rescale(
            prim_path,
            scale=scale,
        )


def create_cube(
    prim_path: str,
    *,
    position: geom.Translation | None = None,
    rotation: geom.Rotation | None = None,
    scale: geom.Scale | None = None,
    collide: bool = True,
    **kwargs: object,
) -> None:
    """Create a cube mesh prim."""

    return create_mesh(
        prim_path,
        "Cube",
        position=position,
        rotation=rotation,
        scale=scale,
        collide=collide,
        **kwargs,
    )


def create_plane(
    prim_path: str,
    *,
    position: geom.Translation | None = None,
    rotation: geom.Rotation | None = None,
    scale: geom.Scale | None = None,
    collide: bool = True,
    **kwargs: object,
) -> None:
    """Create a plane mesh prim."""

    return create_mesh(
        prim_path,
        "Plane",
        position=position,
        rotation=rotation,
        scale=scale,
        collide=collide,
        **kwargs,
    )
