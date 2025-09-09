import os

import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from rclpy.qos import QoSProfile

from isaac_utils.utils import geom
from isaacsim_msgs.srv import SpawnUsd

from .utils import Service, on_exception

profile = QoSProfile(depth=2000)


@on_exception(False)
def spawn_usd(stage, request: SpawnUsd.Request) -> bool:
    name = request.name
    usd_path = request.usd_path
    prim_path = request.prim_path

    prim_utils.create_prim(
        prim_path=prim_path,
        prim_type="Xform",
        translation=np.array(geom.Translation.parse(request.pose.position).tuple),
        orientation=np.array(geom.Rotation.parse(request.pose.orientation).euler),
        scale=np.array([0.01, 0.01, 0.01]),
    )

    stage.add_reference_to_stage(usd_path, os.path.join(prim_path, name))
    return True


def spawn_usd_callback(request: SpawnUsd.Request, response: SpawnUsd.Response):
    stage = omni.usd.get_context().get_stage()
    response.ret = spawn_usd(stage, request)


spawn_usd_service = Service(
    srv_type=SpawnUsd,
    srv_name='isaac/SpawnUsd',
    callback=spawn_usd
)


__all__ = ['spawn_usd_service']
