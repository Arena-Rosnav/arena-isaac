from isaac_utils.managers.elevator_manager import elevator_manager
import os

import omni
from rclpy.qos import QoSProfile

from isaac_utils.utils import geom
from isaac_utils.utils.material import Material
from isaac_utils.utils.mesh import create_cube
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path
from isaacsim_msgs.msg import Elevator
from isaacsim_msgs.srv import SpawnElevators

from .utils import Service, on_exception
import logging
profile = QoSProfile(depth=2000)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)

log = logging.getLogger(__name__)
#Note Error
@on_exception(False)
def spawn_elevator(elevator: Elevator) -> bool:
    prim_path = world_path(elevator.name)
    pos = geom.Translation.parse(elevator.position)
    size = geom.Scale.parse(elevator.size)
    material = elevator.material
    # Ensure parent path exists
    parent_path = os.path.dirname(prim_path)
    ensure_path(parent_path)

    create_cube(
        prim_path=prim_path,
        position=pos,
        scale=size,
    )
    log.info("SpawnElevators")
    if (material := Material.from_msg(elevator.material)):
        try:
            material.bind_to(prim_path)
        except Exception as e:
            print(f"[Elevator] Failed to bind material '{elevator.material}' to '{prim_path}': {e}")

    # Register elevator with elevator_manager
    elevator_manager.add_elevator(elevator, getattr(elevator, 'destination', None))
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
