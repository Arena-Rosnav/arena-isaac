from geometry_msgs.msg import Pose
from isaac_utils.utils import geom
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Scale
from isaacsim_msgs.srv import EditPrims

from .utils import Service, on_exception


@on_exception(False)
def move_prim(name: str, pose: Pose) -> bool:
    geom.move(
        prim_path=world_path(name),
        translation=geom.Translation.parse(pose.position),
        rotation=geom.Rotation.parse(pose.orientation),
    )

    return True


@on_exception(False)
def scale_prim(name: str, scale: Scale) -> bool:
    prim_path = world_path(name)

    geom.rescale(
        prim_path=prim_path,
        scale=geom.Scale.parse(scale),
    )

    return True


def edit_prims_callback(request: EditPrims.Request, response: EditPrims.Response):
    results = (True for _ in request.prims)

    if request.pose:
        results = (a and b for a, b in zip(results, map(move_prim, (p.name for p in request.prims), (p.pose for p in request.prims))))

    if request.scale:
        results = (a and b for a, b in zip(results, map(scale_prim, (p.name for p in request.prims), (p.scale for p in request.prims))))

    response.ret = list(results)
    return response


edit_prims_service = Service(
    srv_type=EditPrims,
    srv_name='isaac/EditPrims',
    callback=edit_prims_callback
)

__all__ = ['edit_prims_service']
