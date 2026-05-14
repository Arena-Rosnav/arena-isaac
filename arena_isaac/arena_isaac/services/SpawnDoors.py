import math
import os

import carb
import numpy as np
import omni

from isaac_utils.managers import entity_lifecycle
from isaac_utils.managers.door_manager import DoorManager
from isaac_utils.utils.geom import Rotation, Scale, Translation
from isaac_utils.utils.material import Material
from isaac_utils.utils.mesh import create_cube
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


@on_exception(False)
def spawn_door(door: Door) -> bool:
    # Get service attributes
    prim_path = world_path(door.name)
    carb.log_verbose(f"DEBUG SpawnDoor called for '{door.name}' -> prim_path: {prim_path}")

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
    length = float(np.linalg.norm((end - start)[:2]))
    angle = math.atan2(end[1] - start[1], end[0] - start[0])

    # create door
    stage = omni.usd.get_context().get_stage()

    existing_object = stage.GetPrimAtPath(prim_path)
    if existing_object is not None:
        stage.RemovePrim(prim_path)

    create_cube(
        prim_path=prim_path,
        position=Translation(*center),
        scale=Scale(length, thickness, height),
        rotation=Rotation.parse([0, 0, angle]),
    )

    # Diagnostic: list prims under the door path
    try:
        created = stage.GetPrimAtPath(prim_path)
        carb.log_verbose(f"DEBUG SpawnDoor: prim at {prim_path} valid={bool(created and created.IsValid())}")
        # list any prims that start with this path
        found = []
        for p in stage.Traverse():
            pstr = str(p.GetPath())
            if pstr.startswith(str(prim_path)):
                found.append(pstr)
        carb.log_verbose(f"DEBUG SpawnDoor: prims under {prim_path}: {found}")
        # If create resulted in a deeper prim, pick the first found prim as door_prim_path
        door_prim_path = prim_path if (created and created.IsValid()) else (found[0] if found else prim_path)
    except Exception as e:
        carb.log_warn(f"DEBUG SpawnDoor: diagnostics failed: {e}")
        door_prim_path = prim_path
        return False

    if (material := Material.from_msg(door.material)):
        material.bind_to(door_prim_path)

    # Register the actual prim path with DoorManager
    try:
        carb.log_info(f"DEBUG SpawnDoor: registering door prim with DoorManager: {door_prim_path}")
        DoorManager.instance().register_door(door_prim_path, kind, start, end)
        entity_lifecycle.register_door(door_prim_path)
    except Exception as e:
        import traceback
        carb.log_error(f"SpawnDoor: failed to register door: {e}\n{traceback.format_exc()}")
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
