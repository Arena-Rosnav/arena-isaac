import omni.kit.commands as commands
import omni.usd
from isaac_utils.managers.door_manager import DoorManager
from isaac_utils.managers.elevator_manager import ElevatorManager
from isaac_utils.utils.path import world_path
from isaacsim.core.experimental.prims import Prim
from isaacsim_msgs.srv import ResetWorld

from .utils import Service, on_exception

_ENVIRONMENT_ROOTS = ('Walls', 'Doors', 'Floors', 'Elevators')


@on_exception(False)
def reset_world() -> bool:
    DoorManager.instance().reset_environment()
    ElevatorManager.instance().reset_environment()

    stage = omni.usd.get_context().get_stage()
    for name in _ENVIRONMENT_ROOTS:
        root = world_path(name)
        for target in Prim.resolve_paths([root])[0]:
            commands.execute("DeletePrims", paths=[target])
            stage.RemovePrim(target)
    return True


def reset_world_callback(request: ResetWorld.Request, response: ResetWorld.Response):
    del request
    response.ret = reset_world()
    return response


reset_world_service = Service(
    srv_type=ResetWorld,
    srv_name='isaac/ResetWorld',
    callback=reset_world_callback,
)

__all__ = ['reset_world_service']
