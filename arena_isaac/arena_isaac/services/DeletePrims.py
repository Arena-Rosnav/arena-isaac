import carb
import omni.kit.commands as commands
import omni.usd
from isaacsim_msgs.srv import DeletePrims

from isaac_utils.managers import entity_lifecycle
from isaac_utils.utils import geom
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import resolve_paths

from .utils import Service, on_exception


@on_exception(False)
def delete_prim(name: str) -> bool:
    target = world_path(name)

    entity_lifecycle.destroy_under(target)

    paths = resolve_paths(target)
    stage = omni.usd.get_context().get_stage()
    for path in paths:
        geom.unregister_robot(path)
        try:
            commands.execute("DeletePrims", paths=[path])
        except Exception as error:
            carb.log_warn(f"DeletePrims: trailing destroy of {path} raised: {error}")
            return False
        stage.RemovePrim(path)
    return True


def delete_prims_callback(request: DeletePrims.Request, response: DeletePrims.Response) -> DeletePrims.Response:
    response.ret = list(map(delete_prim, request.names))
    return response


delete_prims_service = Service(srv_type=DeletePrims, srv_name='isaac/DeletePrims', callback=delete_prims_callback)

__all__ = ['delete_prims_service']
