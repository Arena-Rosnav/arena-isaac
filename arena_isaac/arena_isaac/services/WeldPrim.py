from isaacsim_msgs.srv import WeldPrim

from isaac_utils.utils import weld
from isaac_utils.utils.path import world_path

from .utils import Service, on_exception


@on_exception(False)
def weld_prim(request: WeldPrim.Request, response: WeldPrim.Response) -> bool:
    object_path = world_path(request.name)
    if request.weld:
        return weld.attach(object_path, world_path(request.robot), request.link)
    pose = weld.release(object_path)
    if pose is None:
        return False
    position, orientation = pose
    response.pose.position.x, response.pose.position.y, response.pose.position.z = map(float, position)
    response.pose.orientation.w, response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z = map(float, orientation)
    return True


def weld_prim_callback(request: WeldPrim.Request, response: WeldPrim.Response) -> WeldPrim.Response:
    response.success = weld_prim(request, response)
    return response


weld_prim_service = Service(srv_type=WeldPrim, srv_name='isaac/WeldPrim', callback=weld_prim_callback)

__all__ = ['weld_prim_service']
