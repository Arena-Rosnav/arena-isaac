import omni.kit.commands as commands
from rclpy.qos import QoSProfile

from isaac_utils.managers.door_manager import door_manager
from isaac_utils.utils.path import world_path
from isaacsim_msgs.srv import DeletePrim

from .utils import safe

profile = QoSProfile(depth=2000)


@safe()
def prim_deleter(request, response):
    prim_path = world_path(request.name)
    try:
        if prim_path == (doors_path := world_path("Doors")) or prim_path.startswith(doors_path + "/"):
            door_manager.remove_all_doors()
    except Exception as e:
        pass
        # _LOGGER and _LOGGER.warn(f"Failed to remove doors on delete: {e}")

    commands.execute(
        "IsaacSimDestroyPrim",
        prim_path=prim_path,
    )
    response.ret = True
    return response


def delete_prim(controller):
    service = controller.create_service(
        srv_type=DeletePrim,
        qos_profile=profile,
        srv_name='isaac/delete_prim',
        callback=prim_deleter
    )
    return service
