import os

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from pxr import Gf
from rclpy.qos import QoSProfile

from isaac_utils.utils import geom
from isaac_utils.utils.material import Material
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Elevator
from isaacsim_msgs.srv import SpawnElevators

from .utils import Service, on_exception

profile = QoSProfile(depth=2000)


@on_exception(False)
def spawn_elevator(elevator: Elevator) -> bool:
    prim_path = world_path(elevator.name)
    pos = geom.Translation.parse(elevator.position)
    size = Gf.Vec3f(*elevator.size)
    material = elevator.material

    world = World.instance()
    world.scene.add(FixedCuboid(
        prim_path=prim_path,
        name=os.path.basename(prim_path),
        position=pos.Vec3d(),
        scale=size,
    ))

    if (material := Material.from_msg(elevator.material)):
        material.bind_to(prim_path)

    return True


def spawn_elevators_callback(request: SpawnElevators.Request, response: SpawnElevators.Response):
    response.ret = list(map(spawn_elevator, request.elevators))
    return response


spawn_elevators_service = Service(
    srv_type=SpawnElevators,
    srv_name='isaac/SpawnElevators',
    callback=spawn_elevators_callback
)

__all__ = ['spawn_elevators_service']
