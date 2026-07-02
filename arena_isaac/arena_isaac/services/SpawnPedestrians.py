import math

import carb
import numpy as np
from arena_people_msgs.msg import SpawnPedestrian
from arena_people_msgs.srv import SpawnPedestrians
from peds import runtime

from .utils import Service, on_exception


@on_exception(SpawnPedestrians.Response.FAILED_CREATE)
def spawn_pedestrian(item: SpawnPedestrian) -> int:
    pedestrian = item.pedestrian
    model_source = pedestrian.model_uri or item.model_ref
    if not model_source:
        carb.log_error(f"SpawnPedestrians: no model_uri or model_ref for {pedestrian.name}")
        return SpawnPedestrians.Response.FAILED_CREATE

    position = np.array([pedestrian.pose.position.x, pedestrian.pose.position.y, pedestrian.pose.position.z])
    yaw = 2.0 * math.atan2(pedestrian.pose.orientation.z, pedestrian.pose.orientation.w)
    orientation = np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])

    runtime.spawn(pedestrian.name, position, orientation, model_source)
    return SpawnPedestrians.Response.SUCCESS


def spawn_pedestrians_callback(
    request: SpawnPedestrians.Request, response: SpawnPedestrians.Response
) -> SpawnPedestrians.Response:
    response.results = [spawn_pedestrian(item) for item in request.pedestrians]
    return response


spawn_pedestrians_service = Service(
    srv_type=SpawnPedestrians,
    srv_name='isaac/SpawnPedestrians',
    callback=spawn_pedestrians_callback
)

__all__ = ['spawn_pedestrians_service']
