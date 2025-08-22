import isaac_utils.utils.paths as Paths
import numpy as np
from isaacsim.core.utils.prims import get_prim_at_path
from rclpy.qos import QoSProfile

from isaacsim_msgs.srv import GetPrimAttributes

from .utils import safe

profile = QoSProfile(depth=2000)


@safe
def get_prim_attributes(request, response):
    prim = get_prim_at_path(Paths.scene.path(request.prim_path))

    translate = np.array(prim.GetAttribute("xformOp:translate").Get(), dtype=np.float32)
    quat = prim.GetAttribute("xformOp:orient").Get()

    response.position.x = translate[0]
    response.position.y = translate[1]
    response.position.z = translate[2]

    response.orientation.w = quat.real
    response.orientation.x = quat.imaginary[0]
    response.orientation.y = quat.imaginary[1]
    response.orientation.z = quat.imaginary[2]

    response.scale = np.array(prim.GetAttribute("xformOp:scale").Get(), dtype=np.float32)

    return response


def get_prim_attr(controller):
    service = controller.create_service(
        srv_type=GetPrimAttributes,
        qos_profile=profile,
        srv_name='isaac/get_prim_attributes',
        callback=get_prim_attributes
    )
    return service
