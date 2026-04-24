from isaac_utils.managers.door_manager import DoorManager
from isaac_utils.utils.path import world_path
from pedestrian.simulator.logic.people_manager import PeopleManager

from arena_people_msgs.srv import DeletePedestrians

from .utils import Service, on_exception


@on_exception(DeletePedestrians.Response.NOT_FOUND)
def remove_person(stage_prefix: str) -> int:
    person = PeopleManager.get_people_manager().get_person(world_path(stage_prefix))
    if person is None:
        return DeletePedestrians.Response.NOT_FOUND
    person.destroy()
    return DeletePedestrians.Response.SUCCESS


def delete_pedestrians_callback(request: DeletePedestrians.Request, response: DeletePedestrians.Response):
    response.results = [remove_person(name) for name in request.names]
    DoorManager.instance().reset_peds()
    return response


delete_pedestrians_service = Service(
    srv_type=DeletePedestrians,
    srv_name='isaac/DeletePedestrians',
    callback=delete_pedestrians_callback
)

__all__ = ['delete_pedestrians_service']
