import carb
import isaac_utils.utils.paths as Paths
import omni.anim.navigation.core as nav
from pedestrian.simulator.logic.people.person import Person
from pedestrian.simulator.logic.people_manager import PeopleManager
from rclpy.qos import QoSProfile

from isaacsim_msgs.msg import NavPed
from isaacsim_msgs.srv import MovePed

from .utils import safe

profile = QoSProfile(depth=2000)


@safe
def move_pedestrian(request: MovePed.Request, response: MovePed.Response):
    for nav_command in request.nav_list:
        nav_command: NavPed

        usd_path = Paths.scene.pedestrian(nav_command.path)

        # /Characters/D_gazebo_actor_1/ManRoot/male_adult_police_04
        person = PeopleManager.get_people_manager().get_person(usd_path)
        assert isinstance(person, Person), f"Person not found for path: {usd_path}"
        inav = nav.acquire_interface()
        navmesh = inav.get_navmesh()
        if navmesh:
            navmesh_path = navmesh.query_shortest_path(
                person._state.position.tolist(), nav_command.goal_pose
            )
            if navmesh_path:
                path_points = navmesh_path.get_points()
                person.update_target_position(path_points, nav_command.velocity)
            else:
                carb.log_error(f"NavMesh could not query points")

    response.ret = True
    return response


def move_ped(controller):
    service = controller.create_service(
        srv_type=MovePed,
        qos_profile=profile,
        srv_name="isaac/move_pedestrians",
        callback=move_pedestrian,
    )
    return service
