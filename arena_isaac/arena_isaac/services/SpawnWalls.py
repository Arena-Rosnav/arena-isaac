import math

import numpy as np
from isaacsim_msgs.msg import Wall
from isaacsim_msgs.srv import SpawnWalls
from pxr import Sdf, UsdGeom

from isaac_utils.utils.geom import Rotation, Scale, Translation
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.mesh import create_cube
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import stage

from .utils import Service, on_exception


@on_exception(False)
def wall_spawner(wall: Wall) -> bool:

    prim_path = world_path(wall.name)
    thickness = wall.thickness

    start = Translation.parse(wall.start).Vec3d()
    end = Translation.parse(wall.end).Vec3d()
    vector_ab = end - start

    center = (start + end) / 2

    length = float(np.linalg.norm(vector_ab[:2]))
    angle = math.atan2(vector_ab[1], vector_ab[0])
    # print("wall angle", angle)

    # create wall
    create_cube(
        prim_path=prim_path,
        position=Translation(*center),
        scale=Scale(length, thickness, end[2] - start[2]),
        rotation=Rotation.parse([0, 0, angle]),
        collide=wall.solid,
    )

    if material := Material.from_msg(wall.material):
        material.bind_to(prim_path)

    if wall.solid:
        Material.physics(
            parent_prim_path=world_path(),
            key='ground_default',
            params=PhysicsParams(
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
                combine_mode='min',
            ),
        ).bind_to(prim_path)

    primvars = UsdGeom.PrimvarsAPI(stage.GetPrimAtPath(prim_path))
    if not wall.visible:
        primvars.CreatePrimvar('hideForCamera', Sdf.ValueTypeNames.Bool).Set(True)
    if not wall.shadows:
        primvars.CreatePrimvar('doNotCastShadows', Sdf.ValueTypeNames.Bool).Set(True)

    return True


def spawn_walls_callback(request: SpawnWalls.Request, response: SpawnWalls.Response) -> SpawnWalls.Response:
    response.ret = list(map(wall_spawner, request.walls))
    return response


spawn_walls_service = Service(srv_type=SpawnWalls, srv_name='isaac/SpawnWalls', callback=spawn_walls_callback)

__all__ = ['spawn_walls_service']
