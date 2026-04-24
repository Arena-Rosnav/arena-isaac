from pedestrian.simulator.logic.people.person import Person
from pedestrian.simulator.logic.people_manager import PeopleManager

from isaac_utils.utils.path import world_path
from arena_people_msgs.msg import Pedestrian
from arena_people_msgs.srv import MovePedestrians

from .utils import Service, on_exception


@on_exception(MovePedestrians.Response.NOT_FOUND)
def move_pedestrian(pedestrian: Pedestrian) -> int:
    usd_path = world_path(pedestrian.name)

    person = PeopleManager.get_people_manager().get_person(usd_path)
    if not isinstance(person, Person):
        return MovePedestrians.Response.NOT_FOUND

    p = pedestrian.pose
    person.set_world_pose(
        (p.position.x, p.position.y, p.position.z),
        (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
    )
    return MovePedestrians.Response.SUCCESS


def move_pedestrians_callback(request: MovePedestrians.Request, response: MovePedestrians.Response):
    response.results = list(map(move_pedestrian, request.pedestrians))
    return response


move_pedestrians_service = Service(
    srv_type=MovePedestrians,
    srv_name='isaac/MovePedestrians',
    callback=move_pedestrians_callback
)

__all__ = ['move_pedestrians_service']
