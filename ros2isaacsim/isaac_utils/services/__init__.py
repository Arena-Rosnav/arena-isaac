import collections.abc

from .DeleteAllPedestrians import delete_all_pedestrians_service
from .DeletePrims import delete_prims_service
from .EditPrims import edit_prims_service
from .GetPrims import get_prims_service
from .NavigatePedestrians import navigate_pedestrians_service
from .SpawnDoors import spawn_doors_service
from .SpawnFloors import spawn_floors_service
from .SpawnPrims import spawn_prims_service
from .SpawnPedestrians import spawn_pedestrians_service
from .SpawnUrdf import spawn_urdf_service
from .SpawnUsd import spawn_usd_service
from .SpawnWalls import spawn_walls_service
from .SpawnElevator import spawn_elevator_service
from .utils import Service

services: collections.abc.Iterable[Service] = (
    delete_all_pedestrians_service,
    delete_prims_service,
    edit_prims_service,
    get_prims_service,
    navigate_pedestrians_service,
    spawn_doors_service,
    spawn_floors_service,
    spawn_prims_service,
    spawn_pedestrians_service,
    spawn_urdf_service,
    spawn_usd_service,
    spawn_walls_service,
    spawn_elevator_service,
)

__all__ = ["services"]
