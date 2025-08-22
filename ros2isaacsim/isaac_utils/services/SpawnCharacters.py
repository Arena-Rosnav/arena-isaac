import os
from isaac_utils.utils.path import pedestrian_path
from isaac_utils.utils.prim import ensure_path
from omni.isaac.core import World
from pedestrian.simulator.logic.people.person import Person
from rclpy.qos import QoSProfile

from isaacsim_msgs.srv import Pedestrian
from isaac_utils.managers.door_manager import door_manager

from .utils import safe

profile = QoSProfile(depth=2000)


# simple logger helpers
try:
    import rclpy
    from rclpy.logging import get_logger
    _LOGGER = get_logger('isaac_spawn_ped')
except Exception:
    _LOGGER = None


def _log_info(msg: str):
    try:
        if _LOGGER:
            _LOGGER.info(msg)
            return
    except Exception:
        pass
    print(msg)


def _log_debug(msg: str):
    try:
        if _LOGGER:
            _LOGGER.debug(msg)
            return
    except Exception:
        pass
    print(msg)


@safe()
def pedestrian_spawn(request, response):
    # Get service attributes
    world = World.instance()
    people = request.people

    for person in people:
        usd_path = pedestrian_path(person.stage_prefix)
        ensure_path(os.path.dirname(usd_path))
        if not person.controller_stats:
            p = Person(world, usd_path, person.character_name, person.initial_pose, person.orientation)
        else:
            p = Person(world, usd_path, person.character_name, person.initial_pose, person.orientation, person.controller_name)
        
        # Register the pedestrian root prim for door checks
        door_manager.add_pedestrian(usd_path)

    response.ret = True
    return response


def spawn_ped(controller):
    service = controller.create_service(
        srv_type=Pedestrian,
        qos_profile=profile,
        srv_name='isaac/spawn_pedestrian',
        callback=pedestrian_spawn
    )
    return service
