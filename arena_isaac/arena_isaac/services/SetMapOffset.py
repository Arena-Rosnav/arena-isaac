from geometry_msgs.msg import Vector3
from isaacsim_msgs.srv import SetMapOffset

import isaac_utils.graphs.odom as odom
from isaac_utils.utils.path import world_path

from .utils import Service, on_exception


@on_exception(False)
def set_map_offset(name: str, translation: Vector3) -> bool:
    return odom.set_map_translation(world_path(name), (translation.x, translation.y, translation.z))


def set_map_offset_callback(request: SetMapOffset.Request, response: SetMapOffset.Response) -> SetMapOffset.Response:
    response.success = set_map_offset(request.name, request.translation)
    return response


set_map_offset_service = Service(srv_type=SetMapOffset, srv_name='isaac/SetMapOffset', callback=set_map_offset_callback)

__all__ = ['set_map_offset_service']
