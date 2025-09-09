import os

import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from pxr import Gf

from isaac_utils.utils.material import Material
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Floor
from isaacsim_msgs.srv import SpawnFloors

from .utils import Service, on_exception


@on_exception(False)
def spawn_floor(floor: Floor) -> bool:
    # Get service attributes
    prim_path = world_path(floor.name)
    x_len = floor.x_length
    y_len = floor.y_length
    pos = Gf.Vec3d(*np.append(np.array(floor.pos), 0.0))
    floor_material = floor.material.url
    floor_material_name = floor.material.name

    stage = omni.usd.get_context().get_stage()
    world = World.instance()
    scale = Gf.Vec3f(*[x_len, y_len, 0.01])
    world.scene.add(FixedCuboid(
        prim_path=prim_path,
        name=os.path.basename(prim_path),
        scale=scale,
        position=pos,
    ))

    if (material := Material.from_msg(floor.material)):
        material.bind_to(prim_path)

    return True


def spawn_floors_callback(request: SpawnFloors.Request, response: SpawnFloors.Response):
    response.ret = list(map(spawn_floor, request.floors))
    return response


spawn_floors_service = Service(
    srv_type=SpawnFloors,
    srv_name='isaac/SpawnFloors',
    callback=spawn_floors_callback
)

__all__ = ['spawn_floors_service']
