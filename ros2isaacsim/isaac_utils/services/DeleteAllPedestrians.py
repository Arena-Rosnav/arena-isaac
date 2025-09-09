from pedestrian.simulator.logic.people_manager import PeopleManager

from isaac_utils.managers.door_manager import door_manager
from isaacsim_msgs.srv import DeletePrims

from .DeletePrims import delete_prims_callback
from .utils import Service, on_exception


@on_exception(False)
def reset_managers() -> bool:
    door_manager.reset()
    PeopleManager.get_people_manager().remove_all_people()
    return True


def delete_all_pedestrians_callback(request: DeletePrims.Request, response: DeletePrims.Response):
    delete_prims_callback(request, response)
    reset_managers()
    return response


delete_all_pedestrians_service = Service(
    srv_type=DeletePrims,
    srv_name='isaac/DeleteAllPedestrians',
    callback=delete_all_pedestrians_callback
)

__all__ = ['delete_all_pedestrians_service']
