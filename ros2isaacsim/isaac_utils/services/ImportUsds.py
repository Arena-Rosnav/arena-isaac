from .utils import safe
import os

import numpy as np
from isaac_utils.utils import geom
import isaacsim.core.utils.prims as prim_utils

from isaacsim_msgs.srv import ImportUsd
from rclpy.qos import QoSProfile

profile = QoSProfile(depth=2000)


@safe()
def usd_importer(stage, request, response):
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
    response.ret = True


def import_usd(controller):
    service = controller.create_service(
        srv_type=ImportUsd,
        profile=qos_profile,
        srv_name='isaac/import_usd',
        callback=usd_importer
    )
    return service
