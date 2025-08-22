import math
import os

import isaac_utils.utils.paths as Paths
import numpy as np
import omni
from isaac_utils.managers.door_manager import door_manager
from isaac_utils.utils.prim import ensure_path
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Gf
from rclpy.qos import QoSProfile

from isaacsim_msgs.srv import SpawnDoor

from .utils import safe

profile = QoSProfile(depth=2000)
try:
    import rclpy
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


@safe
def door_spawner(request: SpawnDoor.Request, response: SpawnDoor.Response):
    # Get service attributes
    prim_path = Paths.scene.door(request.name)
    _log_debug(f"DEBUG SpawnDoor called for '{request.name}' -> prim_path: {prim_path}")

    # Ensure parent path exists so creation won't fail silently
    try:
        ensure_path(os.path.dirname(prim_path))
    except Exception:
        pass

    height = request.height
    material = request.material
    kind = request.kind

    start = np.append(np.array(request.start), height / 2)
    end = np.append(np.array(request.end), height / 2)

    start_vec = Gf.Vec3d(*start)
    end_vec = Gf.Vec3d(*end)

    vector_ab = end - start

    center = (start_vec + end_vec) / 2
    length = np.linalg.norm(vector_ab[:2])
    angle = math.atan2(vector_ab[1], vector_ab[0])
    scale = Gf.Vec3f(*[length, 0.1, height])

    # create door
    stage = omni.usd.get_context().get_stage()
    world = World.instance()

    # Generate unique name and check if object already exists
    unique_name = prim_path.replace('/', '_') + f"_{id(request)}"

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

    # Persist door endpoint metadata so wall spawner can cut walls
    try:
        door_prim = stage.GetPrimAtPath(door_prim_path)
        if door_prim and door_prim.IsValid():
            # store full XYZ endpoints (start/end already include height/2)
            door_prim.SetMetadata('door_start', list(start))
            door_prim.SetMetadata('door_end', list(end))
            _log_debug(f"DEBUG SpawnDoor: set metadata door_start={list(start)} door_end={list(end)} on {door_prim_path}")
        else:
            _log_warn(f"DEBUG SpawnDoor: prim {door_prim_path} invalid, cannot set metadata")
    except Exception as e:
        _log_warn(f"DEBUG SpawnDoor: failed to set metadata on {door_prim_path}: {e}")

    # Create a simple material using OmniPBR instead of external URLs
    mtl_path = f"/World/Looks/DoorMaterial_{request.name}"
    mtl = stage.GetPrimAtPath(mtl_path)
    if not (mtl and mtl.IsValid()):
        try:
            omni.kit.commands.execute('CreateAndBindMdlMaterialFromLibrary',
                                      mdl_name='OmniPBR.mdl',
                                      mtl_name='OmniPBR',
                                      mtl_path=mtl_path,
                                      select_new_prim=False)
            # Set a brown/wood color for doors
            omni.kit.commands.execute('ChangeProperty',
                                      prop_path=f"{mtl_path}/Shader.inputs:diffuse_color_constant",
                                      value=(0.6, 0.4, 0.2),
                                      prev=None)
        except BaseException:
            # Fallback: create basic material without external dependencies
            pass

    try:
        omni.kit.commands.execute('BindMaterialCommand',
                                  prim_path=prim_path,
                                  material_path=mtl_path)
    except BaseException:
        pass  # Material binding failed, continue without material

    # Register the actual prim path with DoorManager
    try:
        _log_info(f"DEBUG SpawnDoor: registering door prim with DoorManager: {door_prim_path}")
        door_manager.add_door(door_prim_path, kind)
    except Exception as e:
        _log_warn(f"DEBUG SpawnDoor: failed to register door: {e}")

    response.ret = True
    return response


def spawn_door(controller):
    service = controller.create_service(
        srv_type=SpawnDoor,
        qos_profile=profile,
        srv_name='isaac/spawn_door',
        callback=door_spawner
    )
    return service
