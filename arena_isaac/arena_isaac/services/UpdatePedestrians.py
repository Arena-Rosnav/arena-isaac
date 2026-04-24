import math

from pedestrian.simulator.logic.people.person import Person
from pedestrian.simulator.logic.people_manager import PeopleManager

from isaac_utils.utils.path import world_path
from arena_people_msgs.msg import Pedestrian
from arena_people_msgs.srv import UpdatePedestrians

from .utils import Service, on_exception


@on_exception(UpdatePedestrians.Response.NOT_FOUND)
def update_pedestrian(pedestrian: Pedestrian) -> int:
    usd_path = world_path(pedestrian.name)

    person = PeopleManager.get_people_manager().get_person(usd_path)
    if not isinstance(person, Person):
        return UpdatePedestrians.Response.NOT_FOUND

    target = [pedestrian.pose.position.x, pedestrian.pose.position.y, pedestrian.pose.position.z]
    speed = math.hypot(pedestrian.twist.linear.x, pedestrian.twist.linear.y)

    person._target_positions.clear()
    person.update_target_positions([target], speed)
    return UpdatePedestrians.Response.SUCCESS


def update_pedestrians_callback(request: UpdatePedestrians.Request, response: UpdatePedestrians.Response):
    response.results = list(map(update_pedestrian, request.pedestrians))
    return response


update_pedestrians_service = Service(
    srv_type=UpdatePedestrians,
    srv_name='isaac/UpdatePedestrians',
    callback=update_pedestrians_callback
)

__all__ = ['update_pedestrians_service']
