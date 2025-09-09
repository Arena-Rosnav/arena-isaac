from omni.isaac.core.utils.prims import get_prim_at_path

from isaacsim_msgs.msg import Prim
from isaacsim_msgs.srv import GetPrims

from .utils import Service, on_exception


@on_exception(None)
def get_prim(name: str) -> Prim:

    msg = Prim()

    msg.name = name

    prim = get_prim_at_path(name)

    translate = prim.GetAttribute("xformOp:translate").Get()
    quat = prim.GetAttribute("xformOp:orient").Get()

    msg.pose.position.x = translate[0]
    msg.pose.position.y = translate[1]
    msg.pose.position.z = translate[2]

    msg.pose.orientation.w = quat.real
    msg.pose.orientation.x = quat.imaginary[0]
    msg.pose.orientation.y = quat.imaginary[1]
    msg.pose.orientation.z = quat.imaginary[2]

    s_x, s_y, s_z = prim.GetAttribute("xformOp:scale").Get()
    msg.scale.x = s_x
    msg.scale.y = s_y
    msg.scale.z = s_z

    return msg


def get_prims_callback(request: GetPrims.Request, response: GetPrims.Response):
    response.prims = list(filter(None, map(get_prim, request.prim_paths)))
    return response


get_prims_service = Service(
    srv_type=GetPrims,
    srv_name='isaac/GetPrims',
    callback=get_prims_callback,
)

__all__ = ['get_prims_service']
