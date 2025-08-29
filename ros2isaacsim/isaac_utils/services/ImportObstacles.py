from .utils import safe
import os

import numpy as np
from isaac_utils.utils import geom, prim
from isaac_utils.utils.path import world_path
from rclpy.qos import QoSProfile

from isaacsim_msgs.srv import ImportObstacles

profile = QoSProfile(depth=2000)


@safe()
def obstacle_importer(request, response):
    name = request.name
    usd_path = request.usd_path
    type_ = request.type
    if type_ == 'Obstacle':
        model_prim = prim.create_prim_safe(
            prim_path=world_path('obstacles', name),
            position=np.array(geom.Translation.parse(request.pose.position).tuple()),
            orientation=np.array(geom.Rotation.parse(request.pose.orientation).quat()),
            usd_path=os.path.abspath(usd_path),
        )
    elif type_ == "Wall":
        model_prim = prim.create_prim_safe(
            prim_path=world_path('Walls', name),
            position=np.array(geom.Translation.parse(request.pose.position).tuple()),
            orientation=np.array(geom.Rotation.parse(request.pose.orientation).quat()),
            usd_path=os.path.abspath(usd_path),
        )

    response.ret = True

    return response


def import_obstacle(controller):
    service = controller.create_service(
        srv_type=ImportObstacles,
        qos_profile=profile,
        srv_name='isaac/import_obstacle',
        callback=obstacle_importer
    )
    return service
