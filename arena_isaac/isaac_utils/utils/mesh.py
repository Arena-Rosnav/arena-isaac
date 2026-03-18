"""Create simple meshes at runtime.
"""
import omni.kit.commands
from isaacsim.core.experimental.prims import Prim
from pxr import UsdPhysics

import isaac_utils.utils.geom as geom


def create_mesh(
    prim_path: str,
    mesh_type: str,
    *,
    position: geom.Translation | None = None,
    rotation: geom.Rotation | None = None,
    scale: geom.Scale | None = None,
    collide: bool = True,
    **kwargs,
):
    """Create a mesh prim of the given type.
    """

    omni.kit.commands.execute(
        "CreateMeshPrimWithDefaultXform",
        prim_type=mesh_type,
        prim_path=prim_path,
        select_new_prim=False,
        **kwargs
    )
    if collide:
        prim = Prim([prim_path])
        if prim.valid:
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
    **kwargs,
):
    """Create a cube mesh prim.
    """

    return create_mesh(
        prim_path,
        "Cube",
        position=position,
        rotation=rotation,
        scale=scale,
        collide=collide,
        **kwargs,
    )
