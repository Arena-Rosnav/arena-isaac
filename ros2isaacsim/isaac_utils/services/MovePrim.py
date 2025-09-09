from isaac_utils.utils import geom
from isaac_utils.utils.path import world_path
from rclpy.qos import QoSProfile

from isaacsim_msgs.srv import MovePrim

from .utils import safe

profile = QoSProfile(depth=2000)


@safe()
def prim_mover(request, response):
    prim_path = world_path(request.name)
    position = request.pose.position
    orientation = request.pose.orientation

    geom.move(
        prim_path=prim_path,
        translation=geom.Translation.parse(position),
        rotation=geom.Rotation.parse(orientation),
    )

    response.ret = True
    return response


def move_prim(controller):
    service = controller.create_service(
        srv_type=MovePrim,
        qos_profile=profile,
        srv_name='isaac/move_prim',
        callback=prim_mover
    )
    return service
