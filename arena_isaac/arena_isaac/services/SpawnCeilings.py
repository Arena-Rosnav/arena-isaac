from isaacsim_msgs.msg import Ceiling
from isaacsim_msgs.srv import SpawnCeilings
from pxr import UsdGeom

from isaac_utils.utils.geom import Scale, Translation
from isaac_utils.utils.mesh import create_plane
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import stage

from .utils import Service, on_exception


@on_exception(False)
def spawn_ceiling(ceiling: Ceiling) -> bool:
    prim_path = world_path(ceiling.name)
    pos = Translation.parse(ceiling.pos)

    # RTX has no one-way culling or per-camera visibility, so ceilings spawn hidden.
    create_plane(
        prim_path=prim_path,
        position=pos,
        scale=Scale(ceiling.x_length, ceiling.y_length, 1.0),
        collide=False,
    )
    UsdGeom.Imageable(stage.GetPrimAtPath(prim_path)).MakeInvisible()

    return True


def spawn_ceilings_callback(request: SpawnCeilings.Request, response: SpawnCeilings.Response) -> SpawnCeilings.Response:
    response.ret = list(map(spawn_ceiling, request.ceilings))
    return response


spawn_ceilings_service = Service(srv_type=SpawnCeilings, srv_name='isaac/SpawnCeilings', callback=spawn_ceilings_callback)

__all__ = ['spawn_ceilings_service']
