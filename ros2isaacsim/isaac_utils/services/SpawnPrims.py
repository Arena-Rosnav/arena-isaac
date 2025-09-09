import numpy as np

from isaac_utils.utils import geom, prim
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Prim
from isaacsim_msgs.srv import SpawnPrims

from .utils import Service, on_exception


@on_exception(False)
def prim_importer(prim_msg: Prim) -> bool:
    name = prim_msg.name
    usd_path = prim_msg.usd_path
    prim.create_prim_safe(
        prim_path=world_path(name),
        position=np.array(geom.Translation.parse(prim_msg.pose.position).tuple()),
        orientation=np.array(geom.Rotation.parse(prim_msg.pose.orientation).quat()),
        usd_path=usd_path,
    )

    return True


def spawn_prims_callback(request: SpawnPrims.Request, response: SpawnPrims.Response):
    response.ret = list(map(prim_importer, request.prims))
    return response


spawn_prims_service = Service(
    srv_type=SpawnPrims,
    srv_name='isaac/SpawnPrims',
    callback=spawn_prims_callback
)


__all__ = ['spawn_prims_service']
