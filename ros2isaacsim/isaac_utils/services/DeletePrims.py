import omni.kit.commands as commands

from isaac_utils.utils.path import world_path
from isaacsim_msgs.srv import DeletePrims

from .utils import Service, on_exception


@on_exception(False)
def delete_prim(name: str) -> bool:
    prim_path = world_path(name)
    commands.execute(
        "IsaacSimDestroyPrim",
        prim_path=prim_path,
    )
    return True


def delete_prims_callback(request: DeletePrims.Request, response: DeletePrims.Response):
    response.ret = list(map(delete_prim, request.names))
    return response


delete_prims_service = Service(
    srv_type=DeletePrims,
    srv_name='isaac/DeletePrims',
    callback=delete_prims_callback
)

__all__ = ['delete_prims_service']
