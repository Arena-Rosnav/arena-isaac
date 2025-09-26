import math
import os

import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Gf

from isaac_utils.utils.material import Material
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Wall
from isaacsim_msgs.srv import SpawnWalls
from isaac_utils.utils.geom import Translation

from .utils import Service, on_exception


@on_exception(False)
def wall_spawner(wall: Wall) -> bool:

    prim_path = world_path(wall.name)
    thickness = wall.thickness

    start = Translation.parse(wall.start).Vec3d()
    end = Translation.parse(wall.end).Vec3d()
    vector_ab = end - start

    center = (start + end) / 2

    length = np.linalg.norm(vector_ab[:2])
    angle = math.atan2(vector_ab[1], vector_ab[0])
    # print("wall angle", angle)
    scale = Gf.Vec3f(length, thickness, end[2] - start[2])

    # create wall
    world = World.instance()

    world.scene.add(FixedCuboid(
        prim_path=prim_path,
        name=os.path.basename(prim_path),
        position=center,
        scale=scale,
        orientation=euler_angles_to_quat([0, 0, angle]),
    ))

    if (material := Material.from_msg(wall.material)):
        material.bind_to(prim_path)

    return True


def spawn_walls_callback(request: SpawnWalls.Request, response: SpawnWalls.Response):
    response.ret = list(map(wall_spawner, request.walls))
    return response


spawn_walls_service = Service(
    srv_type=SpawnWalls,
    srv_name='isaac/SpawnWalls',
    callback=spawn_walls_callback
)

__all__ = ['spawn_walls_service']
