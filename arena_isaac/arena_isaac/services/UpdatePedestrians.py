from arena_people_msgs.msg import Pedestrian
from arena_people_msgs.srv import UpdatePedestrians
from peds import runtime

from .utils import Service, on_exception


@on_exception(UpdatePedestrians.Response.NOT_FOUND)
def update_pedestrian(pedestrian: Pedestrian, stamp_sec: float) -> int:
    found = runtime.update(
        pedestrian.name,
        (pedestrian.pose.position.x, pedestrian.pose.position.y, pedestrian.pose.position.z),
        (pedestrian.twist.linear.x, pedestrian.twist.linear.y),
        pedestrian.animation_state,
        list(pedestrian.joint_state.name),
        list(pedestrian.joint_state.position),
        stamp_sec,
    )
    return UpdatePedestrians.Response.SUCCESS if found else UpdatePedestrians.Response.NOT_FOUND


def update_pedestrians_callback(
    request: UpdatePedestrians.Request, response: UpdatePedestrians.Response
) -> UpdatePedestrians.Response:
    stamp_sec = request.stamp.sec + request.stamp.nanosec * 1e-9
    response.results = [update_pedestrian(pedestrian, stamp_sec) for pedestrian in request.pedestrians]
    return response


update_pedestrians_service = Service(
    srv_type=UpdatePedestrians,
    srv_name='isaac/UpdatePedestrians',
    callback=update_pedestrians_callback
)

__all__ = ['update_pedestrians_service']
