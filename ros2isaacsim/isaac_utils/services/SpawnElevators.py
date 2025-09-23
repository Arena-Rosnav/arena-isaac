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
    if hasattr(elevator.size, 'x') and hasattr(elevator.size, 'y') and hasattr(elevator.size, 'z'):
        size = Gf.Vec3f(elevator.size.x, elevator.size.y, elevator.size.z)
    else:
        size = Gf.Vec3f(*elevator.size)
    material = elevator.material

    # Ensure parent path exists
    parent_path = os.path.dirname(prim_path)
    try:
        from isaac_utils.utils.prim import ensure_path
        ensure_path(parent_path)
    except Exception as e:
        print(f"[Elevator] Failed to ensure parent path {parent_path}: {e}")

    # Use unique name for prim
    unique_name = prim_path.replace('/', '_') + f"_{id(elevator)}"

    print(f"[Elevator] Spawning elevator '{elevator.name}' at {pos.Vec3d()} with size {size} (prim_path: {prim_path}, unique_name: {unique_name})")

    world = World.instance()
    try:
        world.scene.add(FixedCuboid(
            prim_path=prim_path,
            name=unique_name,
            position=pos.Vec3d(),
            scale=size,
        ))
    except Exception as e:
        print(f"[Elevator] Failed to add FixedCuboid for '{elevator.name}': {e}")
        return False

    if (material := Material.from_msg(elevator.material)):
        try:
            material.bind_to(prim_path)
        except Exception as e:
            print(f"[Elevator] Failed to bind material '{elevator.material}' to '{prim_path}': {e}")

    # Register elevator with elevator_manager
    try:
        from isaac_utils.managers.elevator_manager import elevator_manager
        elevator_manager.add_elevator(elevator, getattr(elevator, 'destination', None))
    except Exception as e:
        print(f"[Elevator] Failed to register elevator '{elevator.name}' with elevator_manager: {e}")

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
