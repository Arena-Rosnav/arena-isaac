from arena_people_msgs.msg import Pedestrian, Pedestrians
from arena_people_msgs.srv import UpdatePedestrians
from peds import runtime

from .utils import Service, Subscription, on_exception


@on_exception(UpdatePedestrians.Response.NOT_FOUND)
def update_pedestrian(pedestrian: Pedestrian, stamp_sec: float) -> int:
    found = runtime.update(
        pedestrian.name,
        (pedestrian.pose.position.x, pedestrian.pose.position.y, pedestrian.pose.position.z),
        (pedestrian.pose.orientation.x, pedestrian.pose.orientation.y, pedestrian.pose.orientation.z, pedestrian.pose.orientation.w),
        (pedestrian.twist.linear.x, pedestrian.twist.linear.y),
        pedestrian.animation_state,
        list(pedestrian.joint_state.name),
        list(pedestrian.joint_state.position),
        stamp_sec,
    )
    return UpdatePedestrians.Response.SUCCESS if found else UpdatePedestrians.Response.NOT_FOUND


def update_pedestrians_callback(request: UpdatePedestrians.Request, response: UpdatePedestrians.Response) -> UpdatePedestrians.Response:
    stamp_sec = request.stamp.sec + request.stamp.nanosec * 1e-9
    response.results = [update_pedestrian(pedestrian, stamp_sec) for pedestrian in request.pedestrians]
    return response


def update_pedestrians_msg(msg: Pedestrians) -> None:
    """Topic-borne variant: latest-wins state stream, no per-ped results."""
    stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    for pedestrian in msg.pedestrians:
        update_pedestrian(pedestrian, stamp_sec)


update_pedestrians_service = Service(srv_type=UpdatePedestrians, srv_name='isaac/UpdatePedestrians', callback=update_pedestrians_callback)

update_pedestrians_subscription = Subscription(msg_type=Pedestrians, topic='isaac/arena_peds', callback=update_pedestrians_msg)

__all__ = ['update_pedestrians_service', 'update_pedestrians_subscription']
