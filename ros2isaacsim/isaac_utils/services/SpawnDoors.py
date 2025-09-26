import math
import os

import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Gf

from isaac_utils.managers.door_manager import door_manager
from isaac_utils.utils.geom import Translation
from isaac_utils.utils.material import Material
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path
from isaacsim_msgs.msg import Door
from isaacsim_msgs.srv import SpawnDoors

from .utils import Service, on_exception

try:
    from rclpy.logging import get_logger
    _LOGGER = get_logger('isaac_spawn_door')
except Exception:
    _LOGGER = None


def _log_debug(msg: str):
    try:
        if _LOGGER:
            _LOGGER.debug(msg)
            return
    except Exception:
        pass
    print(msg)


def _log_info(msg: str):
    try:
        if _LOGGER:
            _LOGGER.info(msg)
            return
    except Exception:
        pass
    print(msg)


def _log_warn(msg: str):
    try:
        if _LOGGER:
            _LOGGER.warn(msg)
            return
    except Exception:
        pass
    print(msg)


@on_exception(False)
def spawn_door(door: Door) -> bool:
    # Get service attributes
    prim_path = world_path(door.name)
    _log_debug(f"DEBUG SpawnDoor called for '{door.name}' -> prim_path: {prim_path}")

    # Ensure parent path exists so creation won't fail silently
    try:
        ensure_path(os.path.dirname(prim_path))
    except Exception:
        pass

    kind = door.kind

    start = Translation.parse(door.start).Vec3d()
    end = Translation.parse(door.end).Vec3d()

    center = (start + end) / 2
    thickness = door.thickness
    height = end[2] - start[2]
    length = np.linalg.norm((end - start)[:2])
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    scale = Gf.Vec3f(length, thickness, height)

    # create door
    stage = omni.usd.get_context().get_stage()
    world = World.instance()

    # Generate unique name and check if object already exists
    unique_name = prim_path.replace('/', '_') + f"_{id(door)}"

    # Check if an object with this name already exists and remove it
    try:
        existing_object = world.scene.get_object(unique_name)
        if existing_object is not None:
            world.scene.remove_object(unique_name)
    except Exception:
        pass  # Object doesn't exist, which is fine

    world.scene.add(FixedCuboid(
        prim_path=prim_path,
        name=unique_name,
        position=center,
        scale=scale,
        orientation=euler_angles_to_quat([0, 0, angle]),
    ))

    # Diagnostic: list prims under the door path
    try:
        created = stage.GetPrimAtPath(prim_path)
        _log_debug(f"DEBUG SpawnDoor: prim at {prim_path} valid={bool(created and created.IsValid())}")
        # list any prims that start with this path
        found = []
        for p in stage.Traverse():
            pstr = str(p.GetPath())
            if pstr.startswith(str(prim_path)):
                found.append(pstr)
        _log_debug(f"DEBUG SpawnDoor: prims under {prim_path}: {found}")
        # If create resulted in a deeper prim, pick the first found prim as door_prim_path
        door_prim_path = prim_path if (created and created.IsValid()) else (found[0] if found else prim_path)
    except Exception as e:
        _log_warn(f"DEBUG SpawnDoor: diagnostics failed: {e}")
        door_prim_path = prim_path
        return False

    if (material := Material.from_msg(door.material)):
        material.bind_to(door_prim_path)

    # Register the actual prim path with DoorManager
    try:
        _log_info(f"DEBUG SpawnDoor: registering door prim with DoorManager: {door_prim_path}")
        door_manager.add_door(door_prim_path, kind)
    except Exception as e:
        _log_warn(f"DEBUG SpawnDoor: failed to register door: {e}")
        return False

    return True


def spawn_doors_callback(request: SpawnDoors.Request, response: SpawnDoors.Response):
    response.ret = list(map(spawn_door, request.doors))
    return response


spawn_doors_service = Service(
    srv_type=SpawnDoors,
    srv_name='isaac/SpawnDoors',
    callback=spawn_doors_callback


)

__all__ = ['spawn_doors_service']
