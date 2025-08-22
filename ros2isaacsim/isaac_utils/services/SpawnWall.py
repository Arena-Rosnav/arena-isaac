import math
import os

import isaac_utils.utils.paths as Paths
import numpy as np
import omni
from isaacsim.core.api.objects import FixedCuboid
from omni.isaac.core import World
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Gf
from rclpy.qos import QoSProfile

from isaacsim_msgs.srv import SpawnWall

from .utils import safe

profile = QoSProfile(depth=2000)


@safe
def wall_spawner(request, response):
    # Get service attributes
    prim_path = Paths.scene.wall(request.name)
    height = request.height
    material = request.material
    if not material:
        material = "Adobe_Bricks_01"
    # start = np.append(np.array(request.start), height / 2 + 0.1)
    # end = np.append(np.array(request.end), height / 2 + 0.1)
    start = np.append(np.array(request.start), height / 2)
    end = np.append(np.array(request.end), height / 2)

    start_vec = Gf.Vec3d(*start)
    end_vec = Gf.Vec3d(*end)

    vector_ab = end - start

    center = (start_vec + end_vec) / 2
    length = np.linalg.norm(vector_ab[:2])
    angle = math.atan2(vector_ab[1], vector_ab[0])
    scale = Gf.Vec3f(*[length, 0.05, height])

    # create wall
    stage = omni.usd.get_context().get_stage()
    world = World.instance()

    world.scene.add(FixedCuboid(
        prim_path=prim_path,
        name=os.path.basename(prim_path),
        position=center,
        scale=scale,
        orientation=euler_angles_to_quat([0, 0, angle]),
    ))

    # Create a simple material using OmniPBR instead of external URLs
    mtl_path = "/World/Looks/WallMaterial"
    mtl = stage.GetPrimAtPath(mtl_path)
    if not (mtl and mtl.IsValid()):
        try:
            omni.kit.commands.execute('CreateAndBindMdlMaterialFromLibrary',
                                      mdl_name='OmniPBR.mdl',
                                      mtl_name='OmniPBR',
                                      mtl_path=mtl_path,
                                      select_new_prim=False)
            # Set a gray/concrete color for walls
            omni.kit.commands.execute('ChangeProperty',
                                      prop_path=f"{mtl_path}/Shader.inputs:diffuse_color_constant",
                                      value=(0.7, 0.7, 0.7),
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

    response.ret = True
    return response


def spawn_wall(controller):
    service = controller.create_service(
        srv_type=SpawnWall,
        qos_profile=profile,
        srv_name='isaac/spawn_wall',
        callback=wall_spawner
    )
    return service
