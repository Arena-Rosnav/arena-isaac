import omni.anim.navigation.core as nav
from pedestrian.simulator.logic.people.person import Person
from pedestrian.simulator.logic.people_manager import PeopleManager

from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import PedestrianGoal
from isaacsim_msgs.srv import NavigatePedestrians

from .utils import Service, on_exception


@on_exception(False)
def navigate_pedestrian(goal: PedestrianGoal) -> bool:
    usd_path = world_path(goal.name)

    person = PeopleManager.get_people_manager().get_person(usd_path)
    if not isinstance(person, Person):
        return False
    person.update_target_goal(goal)
    return True


def navigate_pedestrians_callback(
    request: NavigatePedestrians.Request, response: NavigatePedestrians.Response
):
    response.ret = list(map(navigate_pedestrian, request.goals))
    return response


navigate_pedestrians_service = Service(
    srv_type=NavigatePedestrians,
    srv_name="isaac/NavigatePedestrians",
    callback=navigate_pedestrians_callback,
)

__all__ = ["navigate_pedestrians_service"]
