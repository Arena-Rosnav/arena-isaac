import omni.kit.commands as commands
from isaacsim_msgs.srv import DeletePrim
from pedestrian.simulator.logic.people_manager import PeopleManager
import time


def characters_delete(request, response):
    commands.execute(
        "IsaacSimDestroyPrim",
        prim_path=request.name,
    )
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
