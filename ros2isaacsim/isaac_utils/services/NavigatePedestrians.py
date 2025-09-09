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
    assert isinstance(person, Person), f"Person not found for path: {usd_path}"
    inav = nav.acquire_interface()
    navmesh = inav.get_navmesh()
    if navmesh:
        navmesh_path = navmesh.query_shortest_path(
            person._state.position.tolist(), [goal.position.x, goal.position.y, goal.position.z]
        )
        if navmesh_path:
            path_points = navmesh_path.get_points()
            person.update_target_position(path_points, goal.velocity)
            return True

    return False


def navigate_pedestrians_callback(request: NavigatePedestrians.Request, response: NavigatePedestrians.Response):
    response.ret = list(map(navigate_pedestrian, request.goals))
    return response


navigate_pedestrians_service = Service(
    srv_type=NavigatePedestrians,
    srv_name="isaac/NavigatePedestrians",
    callback=navigate_pedestrians_callback
)

__all__ = ['navigate_pedestrians_service']
