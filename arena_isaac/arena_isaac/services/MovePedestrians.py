from arena_people_msgs.msg import Pedestrian
from arena_people_msgs.srv import MovePedestrians
from peds import runtime

from .utils import Service, on_exception


@on_exception(MovePedestrians.Response.NOT_FOUND)
def move_pedestrian(pedestrian: Pedestrian) -> int:
    pose = pedestrian.pose
    found = runtime.move(
        pedestrian.name,
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )
    return MovePedestrians.Response.SUCCESS if found else MovePedestrians.Response.NOT_FOUND


def move_pedestrians_callback(
    request: MovePedestrians.Request, response: MovePedestrians.Response
) -> MovePedestrians.Response:
    response.results = [move_pedestrian(pedestrian) for pedestrian in request.pedestrians]
    return response


move_pedestrians_service = Service(
    srv_type=MovePedestrians,
    srv_name='isaac/MovePedestrians',
    callback=move_pedestrians_callback
)

__all__ = ['move_pedestrians_service']
