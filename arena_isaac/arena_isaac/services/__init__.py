import collections.abc

from .DeletePedestrians import delete_pedestrians_service
from .DeletePrims import delete_prims_service
from .EditPrims import edit_prims_service
from .GetPrims import get_prims_service
from .MovePedestrians import move_pedestrians_service
from .ResetWorld import reset_world_service
from .SpawnCeilings import spawn_ceilings_service
from .SpawnFloors import spawn_floors_service
from .SpawnPedestrians import spawn_pedestrians_service
from .SpawnPrims import spawn_prims_service
from .SpawnUrdf import spawn_urdf_service
from .SpawnUsd import spawn_usd_service
from .SpawnWalls import spawn_walls_service
from .UpdatePedestrians import update_pedestrians_service
from .utils import Service

services: collections.abc.Iterable[Service] = (
    delete_pedestrians_service,
    delete_prims_service,
    edit_prims_service,
    get_prims_service,
    move_pedestrians_service,
    reset_world_service,
    spawn_ceilings_service,
    spawn_floors_service,
    spawn_prims_service,
    spawn_pedestrians_service,
    update_pedestrians_service,
    spawn_urdf_service,
    spawn_usd_service,
    spawn_walls_service,
)

__all__ = ["services"]
