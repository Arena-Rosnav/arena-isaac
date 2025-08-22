import omni.kit.commands as commands
from isaac_utils.utils.path import world_path
from rclpy.qos import QoSProfile
from isaac_utils.managers.door_manager import door_manager

from isaacsim_msgs.srv import DeletePrim

from .utils import safe

profile = QoSProfile(depth=2000)


@safe()
def prim_deleter(request, response):
    prim_path = world_path(request.name)
    if prim_path == "/World/pedestrians":
        door_manager.reset()
        
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
