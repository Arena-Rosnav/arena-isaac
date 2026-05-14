"""Process-wide registry of arena_isaac-spawned entities for symmetric teardown."""
from __future__ import annotations

import threading
import typing
from collections.abc import Mapping

import attrs
import carb
import omni.kit.commands
import omni.usd
from isaac_utils.utils import geom
from isaacsim.core.experimental.prims import Prim

if typing.TYPE_CHECKING:
    from isaac_utils.graphs.sensors import SensorBase


@attrs.define
class RobotManifest:
    prim_path: str
    articulation_path: str
    graph_paths: list[str] = attrs.field(factory=list)
    sensors: list[SensorBase] = attrs.field(factory=list)


_robots: dict[str, RobotManifest] = {}
_doors: set[str] = set()
_elevators: dict[str, str] = {}
_lock = threading.RLock()


def _normalize(prim_path: str) -> str:
    return prim_path.rstrip('/') or '/'


def _is_under(child: str, parent: str) -> bool:
    parent = _normalize(parent)
    child = _normalize(child)
    if parent == '/':
        return True
    return child == parent or child.startswith(parent + '/')


def register_robot(prim_path: str, articulation_path: str) -> RobotManifest:
    key = _normalize(prim_path)
    manifest = RobotManifest(prim_path=key, articulation_path=_normalize(articulation_path))
    with _lock:
        _robots[key] = manifest
    return manifest


def register_door(prim_path: str) -> None:
    with _lock:
        _doors.add(_normalize(prim_path))


def register_elevator(name: str, prim_path: str) -> None:
    with _lock:
        _elevators[name] = _normalize(prim_path)


def _destroy_prim(prim_path: str) -> bool:
    paths, _ = Prim.resolve_paths([prim_path])
    if not paths:
        return False
    stage = omni.usd.get_context().get_stage()
    for path in paths:
        try:
            omni.kit.commands.execute("DeletePrims", paths=[path])
        except Exception as error:
            carb.log_warn(f"entity_lifecycle: DeletePrims({path}) raised: {error}")
            return False
        stage.RemovePrim(path)
    return True


def _destroy_robot(manifest: RobotManifest) -> int:
    from isaac_utils.managers.door_manager import DoorManager
    from isaac_utils.managers.elevator_manager import elevator_manager

    DoorManager.instance().remove_robot(manifest.prim_path)
    elevator_manager.remove_robot(manifest.prim_path)
    geom.unregister_robot(manifest.prim_path)

    # Release sensor-owned writers/render products before deleting their backing prims.
    for sensor in reversed(manifest.sensors):
        try:
            sensor.destroy()
        except Exception as error:
            carb.log_warn(f"entity_lifecycle: sensor.destroy raised: {error}")

    destroyed = 0
    for sensor in reversed(manifest.sensors):
        for sensor_path in reversed(sensor.paths()):
            if _destroy_prim(sensor_path):
                destroyed += 1

    for graph_path in reversed(manifest.graph_paths):
        if _destroy_prim(graph_path):
            destroyed += 1

    if _destroy_prim(manifest.prim_path):
        destroyed += 1

    carb.log_info(f"entity_lifecycle: robot teardown complete: {manifest.prim_path}")
    return destroyed


def _destroy_door(prim_path: str) -> int:
    from isaac_utils.managers.door_manager import DoorManager

    DoorManager.instance().remove_door(prim_path)
    return 1 if _destroy_prim(prim_path) else 0


def _destroy_elevator(name: str, prim_path: str) -> int:
    from isaac_utils.managers.elevator_manager import elevator_manager

    elevator_manager.remove_elevator(name)
    return 1 if _destroy_prim(prim_path) else 0


def destroy_under(parent_prim_path: str) -> int:
    """Destroy every tracked entity at or under parent_prim_path, one carb call each."""
    parent = _normalize(parent_prim_path)
    destroyed = 0

    with _lock:
        robot_keys = [key for key in _robots if _is_under(key, parent)]
        robot_manifests = [_robots.pop(key) for key in robot_keys]

        door_keys = [key for key in _doors if _is_under(key, parent)]
        for key in door_keys:
            _doors.discard(key)

        elevator_items = [
            (name, path) for name, path in _elevators.items() if _is_under(path, parent)
        ]
        for name, _ in elevator_items:
            _elevators.pop(name, None)

    for manifest in robot_manifests:
        destroyed += _destroy_robot(manifest)

    for door_path in door_keys:
        destroyed += _destroy_door(door_path)

    for name, path in elevator_items:
        destroyed += _destroy_elevator(name, path)

    return destroyed


def manifests_snapshot() -> Mapping[str, RobotManifest]:
    with _lock:
        return dict(_robots)


__all__: typing.Final[tuple[str, ...]] = (
    'RobotManifest',
    'register_robot',
    'register_door',
    'register_elevator',
    'destroy_under',
    'manifests_snapshot',
)
