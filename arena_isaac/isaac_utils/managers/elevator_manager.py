import time
from typing import Any, ClassVar, Literal

import attrs
import carb
from isaac_utils.utils import geom
from isaacsim_msgs.msg import Elevator


@attrs.define
class PlatformInfo:
    name: str
    position: Any
    size: Any


@attrs.define
class CooldownState:
    last_tp: float = 0.0
    can_tp: bool = True
    was_on: Literal['a', 'b', 'none'] = 'none'


@attrs.define
class ElevatorPair:
    a: PlatformInfo
    b: PlatformInfo
    cooldown: dict[str, CooldownState] = attrs.field(factory=dict)


class ElevatorManager:
    _instance: ClassVar["ElevatorManager | None"] = None

    @staticmethod
    def instance() -> "ElevatorManager":
        if ElevatorManager._instance is None:
            ElevatorManager._instance = ElevatorManager()
        return ElevatorManager._instance

    def __init__(self):
        self._elevators: dict[str, Elevator] = {}
        self._pairs: list[ElevatorPair] = []
        self._robots: dict[str, str] = {}

    def _has_pair(self, a_name: str, b_name: str) -> bool:
        for pair in self._pairs:
            if (pair.a.name == a_name and pair.b.name == b_name) or (pair.a.name == b_name and pair.b.name == a_name):
                return True
        return False

    def register_node(self, _controller: Any) -> None:
        return

    def add_elevator(self, elevator: Elevator, destination: str | None) -> None:
        self._elevators[elevator.name] = elevator
        # Subscribe to robot odometry for all registered robots
        # Pair elevators by destination
        if destination is None:
            carb.log_warn(f"Destination missing for elevator {elevator.name}")
            return
        dest = self._elevators.get(destination)
        if dest is None:
            carb.log_warn(f"Destination {destination} not found for elevator {elevator.name}")
            return
        if self._has_pair(elevator.name, dest.name):
            return
        self._pairs.append(ElevatorPair(
            a=PlatformInfo(name=elevator.name, position=elevator.position, size=elevator.size),
            b=PlatformInfo(name=dest.name, position=dest.position, size=dest.size),
        ))

    def add_robot(self, prim_path: str) -> None:
        robot_prim_path = prim_path.rstrip('/')
        robot_name = robot_prim_path.split("/")[-1]
        self._robots[robot_name] = robot_prim_path

    def remove_robot(self, prim_path: str) -> None:
        # TODO key _robots by prim_path so recycled envs with same robot name don't collide.
        robot_prim_path = prim_path.rstrip('/')
        robot_name = robot_prim_path.split("/")[-1]
        self._robots.pop(robot_name, None)
        for pair in self._pairs:
            pair.cooldown.pop(robot_prim_path, None)

    def remove_elevator(self, name: str) -> None:
        self._elevators.pop(name, None)
        self._pairs = [pair for pair in self._pairs if pair.a.name != name and pair.b.name != name]

    def reset_environment(self) -> None:
        self._elevators.clear()
        self._pairs.clear()

    def get_robot_pose(self, robot_prim_path: str) -> tuple[float, float, float] | None:
        position = geom.get_world_translation(robot_prim_path)
        if position is None:
            return None
        return position.x, position.y, position.z

    def get_robots(self) -> dict[str, str]:
        return self._robots

    def update(self) -> None:
        now = time.time()
        cooldown_sec = 5
        for pair in self._pairs:
            for robot_name, robot_prim_path in self._robots.items():
                robot_pose = self.get_robot_pose(robot_prim_path)
                if robot_pose is None:
                    continue
                state = pair.cooldown.get(robot_prim_path, CooldownState())
                last_tp = state.last_tp
                can_tp = state.can_tp
                was_on = state.was_on
                on_a = self._robot_on_platform(robot_pose, pair.a)
                on_b = self._robot_on_platform(robot_pose, pair.b)
                # Only allow teleport if robot is on a platform, was previously off both, and cooldown expired
                if can_tp and (on_a ^ on_b) and not (was_on == 'a' and on_a) and not (was_on == 'b' and on_b) and (now - last_tp > cooldown_sec):
                    if on_a:
                        self.teleport_robot(robot_prim_path, pair.b.position)
                        pair.cooldown[robot_prim_path] = CooldownState(last_tp=now, can_tp=False, was_on='a')
                    elif on_b:
                        self.teleport_robot(robot_prim_path, pair.a.position)
                        pair.cooldown[robot_prim_path] = CooldownState(last_tp=now, can_tp=False, was_on='b')
                # Reset teleport permission only when robot is fully off both platforms
                elif not on_a and not on_b:
                    pair.cooldown[robot_prim_path] = CooldownState(last_tp=last_tp, can_tp=True, was_on='none')
                else:
                    pair.cooldown[robot_prim_path] = CooldownState(
                        last_tp=last_tp,
                        can_tp=can_tp,
                        was_on='a' if on_a else 'b' if on_b else 'none',
                    )

    def _robot_on_platform(self, robot_pose: tuple[float, float, float], platform: PlatformInfo) -> bool:
        px = platform.position.x
        py = platform.position.y
        pz = platform.position.z
        sx = platform.size.x
        sy = platform.size.y
        sz = platform.size.z
        rx, ry, rz = robot_pose
        return (
            abs(rx - px) <= sx / 2 and
            abs(ry - py) <= sy / 2 and
            abs(rz - pz) <= max(sz / 2, 0.5)
        )

    def teleport_robot(self, robot_prim_path: str, position: Any) -> None:
        geom.move(
            prim_path=robot_prim_path,
            translation=geom.Translation(position.x, position.y, position.z),
        )


elevator_manager = ElevatorManager.instance()
