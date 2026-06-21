import math
import os

from isaacsim.core.api import World
from pedestrian.simulator.logic.people.person import Person

from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path
from arena_people_msgs.msg import Pedestrian
from arena_people_msgs.srv import SpawnPedestrians

from .utils import Service, on_exception


@on_exception(SpawnPedestrians.Response.FAILED_CREATE)
def spawn_pedestrian(pedestrian: Pedestrian, character_name: str) -> int:
    world = World.instance()

    position = [pedestrian.pose.position.x, pedestrian.pose.position.y, pedestrian.pose.position.z]
    orientation = 2.0 * math.atan2(pedestrian.pose.orientation.z, pedestrian.pose.orientation.w)

    usd_path = world_path(pedestrian.name)
    ensure_path(os.path.dirname(usd_path))
    Person(world, usd_path, character_name, position, orientation)

    return SpawnPedestrians.Response.SUCCESS


def spawn_pedestrians_callback(request: SpawnPedestrians.Request, response: SpawnPedestrians.Response):
    response.results = [
        spawn_pedestrian(item.pedestrian, item.model_ref)
        for item in request.pedestrians
    ]
    return response


spawn_pedestrians_service = Service(
    srv_type=SpawnPedestrians,
    srv_name='isaac/SpawnPedestrians',
    callback=spawn_pedestrians_callback
)

__all__ = ['spawn_pedestrians_service']
