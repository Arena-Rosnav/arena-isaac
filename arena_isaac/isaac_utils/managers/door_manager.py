from __future__ import annotations

import time
import typing

import attrs
import carb
import numpy as np
import omni
import rclpy.node
import std_msgs.msg
from arena_people_msgs.msg import Pedestrians as ArenaPedestrians
from isaac_utils.utils import geom
from nav_msgs.msg import Odometry
from pxr import Gf, Usd, UsdGeom


@attrs.define
class XFormAnimation:
    duration: float

    T: tuple[geom.Translation, geom.Translation] | None = None
    R: tuple[geom.Rotation, geom.Rotation] | None = None
    S: tuple[geom.Scale, geom.Scale] | None = None

    animation_fn: typing.Callable[[float], float] = lambda x: x

    _progress: float = 0.0
    _time: float = attrs.field(init=False, factory=time.time)

    def _interp_T(self, progress: float):
        if self.T is None:
            return None

        start = self.T[0].Vec3d()
        end = self.T[1].Vec3d()
        result = start + (end - start) * progress
        return geom.Translation(*result)

    def _interp_R(self, progress: float):
        if self.R is None:
            return None
        start = self.R[0].Quatd()
        end = self.R[1].Quatd()
        result = Gf.Slerp(progress, start, end)
        return geom.Rotation(*result)

    def _interp_S(self, progress: float):
        if self.S is None:
            return None
        start = self.S[0].Vec3d()
        end = self.S[1].Vec3d()
        result = start + (end - start) * progress
        return geom.Scale(*result)

    def step(self, reverse: bool = False):
        now = time.time()
        dt = now - self._time
        self._time = now
        if reverse:
            dt *= -1
        previous = self._progress
        self._progress = max(min(self._progress + dt / self.duration, 1.), 0.)
        if previous == self._progress:
            return None, None, None
        point = self.animation_fn(self._progress)
        return self._interp_T(point), self._interp_R(point), self._interp_S(point)


@attrs.define
class Door:
    prim: Usd.Prim
    T: geom.Translation
    R: geom.Rotation
    S: geom.Scale
    move_prim_path: str
    axis: np.ndarray
    angle: float
    open: bool = False
    last_toggle_time: float = 0.0
    animation: XFormAnimation | None = None


class DoorManager:
    __instance: typing.ClassVar[DoorManager] = None  # type: ignore

    @classmethod
    def instance(cls, node: rclpy.node.Node | None = None) -> DoorManager:
        if cls.__instance is None:
            assert node is not None, "DoorManager not registered with any node yet; must provide node"
            cls.__instance = DoorManager(node)
        return cls.__instance

    def __init__(self, node: rclpy.node.Node):
        self._doors: dict[str, Door] = {}
        self._robot_poses: dict[str, np.ndarray] = {}
        self._pedestrian_poses: dict[str, np.ndarray] = {}
        # Distance thresholds (meters)
        self._door_open_distance = 3.0  # open when entity is closer than this
        self._door_close_margin = 0.5   # hysteresis margin; close when farther than open_distance + margin
        # Minimum seconds between toggles for a given door to avoid rapid spam
        # For instantaneous open/close set interval to 0.0
        self._door_min_toggle_interval = 0.0

        # Controller node (set by register_node)
        self._controller = node
        # robot subscriptions by registered prim path
        self._robot_subs: dict[str, object] = {}
        self._pedestrian_subs: dict[str, object] = {}

        self._debug_rate_limit = 1.0
        self._last_debug_time = 0.0

        # # instead of ped topic, use direct people manager access
        # self._pedestrian_topic_sub = self._controller.create_subscription(
        #     std_msgs.msg.String,
        #     '/isaac/add_pedestrians_topic',
        #     self._cb_pedestrian_topic,
        #     10
        # )

    def _cb_pedestrian_topic(self, msg: std_msgs.msg.String):
        self._pedestrian_subs[msg.data] = self._controller.create_subscription(
            ArenaPedestrians,
            msg.data,
            self._people_cb,
            10
        )

    def add_robot(self, prim_path: str, odom_topic: str | None = None):
        # TODO auto subscribe to task generator reset and update robots list
        # derive robot name from prim path: /World/<robot_name>/...
        try:

            topic = odom_topic or f'/task_generator_node/{prim_path.split("/")[2]}/odom'
            carb.log_error(f'Add robot in DoorManager {prim_path.split("/")[2]}')
            if Odometry is not None:
                try:
                    sub = self._controller.create_subscription(
                        Odometry,
                        topic,
                        lambda msg, p=prim_path: self._odom_cb(msg, p),
                        10
                    )
                    self._robot_subs[prim_path] = sub
                    carb.log_verbose(f'Subscribed to {topic} for robot {prim_path}')
                except Exception as e:
                    carb.log_warn(f'Failed to subscribe to {topic}: {e}')
            else:
                carb.log_warn('nav_msgs.Odometry not available; robot odom not subscribed')
        except Exception as e:
            carb.log_verbose(f'_ensure_robot_subscription error: {e}')

    def _odom_cb(self, msg: Odometry, prim_path: str): 
        try:
            pos = msg.pose.pose.position
            self._robot_poses[prim_path] = np.array([pos.x, pos.y, pos.z])
            carb.log_verbose(f'odom update for {prim_path}: {self._robot_poses[prim_path]}')
        except Exception as e:
            carb.log_verbose(f'odom_cb error: {e}')

    def _people_cb(self, msg: ArenaPedestrians):
        """Handle incoming pedestrian poses from ROS topic.
        """
        try:
            carb.log_verbose('people topic message received')
            # support both attribute names
            people = msg.pedestrians
            if not people:
                return

            for p in people:
                # identifier
                name = p.name
                xyz = np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])

                self._pedestrian_poses[name] = xyz
                carb.log_verbose(f'people update for {name}: {xyz}')

        except Exception as e:
            carb.log_verbose(f'people_cb error: {e}')

    def register_door(self, prim_path: str, kind: str, start: Gf.Vec3f, end: Gf.Vec3f):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            start_v = start
            end_v = end
            dx = end_v[0] - start_v[0]
            dy = end_v[1] - start_v[1]
            axis = np.array([dx, dy, 0])
            axis = axis / (np.linalg.norm(axis) or 1)
            angle = np.arctan2(axis[1], axis[0]) + np.pi / 2

            door = Door(
                prim=prim,
                move_prim_path=prim.GetPath().pathString,
                T=geom.Translation.parse(prim.GetAttribute('xformOp:translate').Get() or (0, 0, 0)),
                R=geom.Rotation.parse(prim.GetAttribute('xformOp:rotateXYZ').Get() or (0, 0, 0)),
                S=geom.Scale.parse(prim.GetAttribute('xformOp:scale').Get() or (1, 1, 1)),
                open=False,
                last_toggle_time=0.0,
                axis=axis,
                angle=angle,
                animation=None
            )

            if kind == 'sliding':
                door.animation = XFormAnimation(
                    duration=3.0,
                    T=(
                        geom.Translation(0., 0., 0.),
                        geom.Translation(axis[0] * door.S.x, axis[1] * door.S.x, 0.)
                    ),
                )
            elif kind == 'sliding_top':
                door.animation = XFormAnimation(
                    duration=3.0,
                    T=(
                        geom.Translation(0., 0., 0.),
                        geom.Translation(0., 0., door.S.z)
                    ),
                )

            self._doors[prim_path] = door
            carb.log_verbose(f"Added door to door manager: {prim_path}")
        else:
            carb.log_warn(f"Failed to add door - invalid prim: {prim_path}")

    def reset_peds(self):
        self._pedestrian_poses.clear()
        # Close all doors on reset
        for door in self._doors.values():
            self._close_door(door)

    def _update_peds_from_people_manager(self):
        from pedestrian.simulator.logic.people_manager import PeopleManager
        mgr = PeopleManager.get_people_manager()
        self._pedestrian_poses.clear()

        for name, person in mgr.people.items():
            self._pedestrian_poses[name] = person.position

    def update(self):
        if not self._doors:
            carb.log_verbose("No doors registered with DoorManager.")

        self._update_peds_from_people_manager()

        for door_path, door in self._doors.items():
            door_prim = door.prim
            if not door_prim.IsValid():
                carb.log_warn(f"Door prim invalid: {door_path}")
                continue

            poses = [*self._robot_poses.values(), *self._pedestrian_poses.values()]
            if poses:
                door_pos = door.T.Vec3d()
                distances = np.linalg.norm(np.array(poses) - np.array(door_pos), axis=1)
                min_distance = np.min(distances)

                # Hysteresis + cooldown: decide once per door using min_distance
                now = time.time()
                if min_distance < self._door_open_distance:
                    if (not door.open and (now - door.last_toggle_time) > self._door_min_toggle_interval):
                        self._open_door(door)
                elif min_distance > (self._door_open_distance + self._door_close_margin):
                    if (door.open and (now - door.last_toggle_time) > self._door_min_toggle_interval):
                        self._close_door(door)
                door.last_toggle_time = now

            if not door.animation:
                continue
            anim = door.animation
            dT, dR, dS = anim.step(not door.open)
            prim = door.move_prim_path
            if dT:
                geom.move(prim, translation=door.T + dT, local=True)
            if dR:
                geom.move(prim, rotation=door.R * dR, local=True)
            if dS:
                geom.rescale(prim, scale=door.S + dS)

    def _set_visibility(self, prim, visible: bool, recursive: bool = False):
        """Set Usd visibility on prim. visible=True -> 'inherited', False -> 'invisible'.
        If recursive=True, apply to all descendants as well.
        """
        try:
            if prim is None or not prim.IsValid():
                return False
            try:
                img = UsdGeom.Imageable(prim)
            except Exception:
                img = None
            if img is not None:
                val = 'inherited' if visible else 'invisible'
                try:
                    img.GetVisibilityAttr().Set(val)
                except Exception as e:
                    carb.log_verbose(f'Failed to set visibility on {prim.GetPath().pathString}: {e}')
            # optionally apply to children
            if recursive:
                for child in prim.GetAllChildren():
                    try:
                        self._set_visibility(child, visible, recursive=True)
                    except Exception:
                        pass
            carb.log_verbose(f"Set visibility {val} on {prim.GetPath().pathString}")
            return True
        except Exception as e:
            carb.log_verbose(f'_set_visibility error: {e}')
            return False

    def _open_door(self, door: Door):
        if not door.animation:
            self._set_visibility(door.prim, visible=False, recursive=True)
        door.open = True

    def _close_door(self, door: Door):
        if not door.animation:
            self._set_visibility(door.prim, visible=True, recursive=True)
        door.open = False
