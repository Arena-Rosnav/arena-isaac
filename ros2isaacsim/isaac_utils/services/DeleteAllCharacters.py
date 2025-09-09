import time

import omni.kit.commands as commands
from pedestrian.simulator.logic.people_manager import PeopleManager

from isaac_utils.managers.door_manager import door_manager
from isaac_utils.utils.path import world_path
from isaacsim_msgs.srv import DeletePrim

from .utils import safe


@safe()
def characters_delete(request, response):
    commands.execute(
        "IsaacSimDestroyPrim",
        prim_path=world_path(request.name),
    )
    door_manager.reset()
    PeopleManager.get_people_manager().remove_all_people()
    time.sleep(0.5)

    response.ret = True
    return response


def delete_all_characters(controller):
    service = controller.create_service(
        srv_type=DeletePrim,
        srv_name='isaac/delete_all_pedestrians',
        callback=characters_delete
    )
    return service
