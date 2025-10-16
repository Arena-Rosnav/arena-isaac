import typing
import omni
import numpy as np
from pxr import Gf, UsdGeom, Sdf, Usd
import time

# try to import rclpy and message types for subscribing to external pose topics
try:
    from rclpy.qos import QoSProfile
    from people_msgs.msg import People
    # support the new topic type used by task_generator
    from arena_people_msgs.msg import Pedestrians as ArenaPedestrians
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String as StdString
except Exception:
    rclpy = None
    People = None
    ArenaPedestrians = None
    Odometry = None
    QoSProfile = None
    StdString = None

# Prefer rclpy logger when available so messages appear in ros2/launch logs; fallback to print
try:
    from rclpy.logging import get_logger
    _LOGGER = get_logger('isaac_door_manager')
except Exception:
    _LOGGER = None


def _log_debug(msg: str):
    print(f"[DoorManager][DEBUG] {msg}")
    try:
        if _LOGGER:
            _LOGGER.debug(msg)
    except Exception:
        pass


def _log_info(msg: str):
    print(f"[DoorManager][INFO] {msg}")
    try:
        if _LOGGER:
            _LOGGER.info(msg)
    except Exception:
        pass


def _log_warn(msg: str):
    print(f"[DoorManager][WARN] {msg}")
    try:
        if _LOGGER:
            _LOGGER.warn(msg)
    except Exception:
        pass


def _log_error(msg: str):
    try:
        if _LOGGER:
            _LOGGER.error(msg)
            return
    except Exception:
        pass
    print(msg)


class DoorManager:
    def __init__(self):
        self._doors = {}
        self._robots = []
        self._pedestrians = []
        # Distance thresholds (meters)
        self._door_open_distance = 3.0  # open when entity is closer than this
        self._door_close_margin = 0.5   # hysteresis margin; close when farther than open_distance + margin
        # Minimum seconds between toggles for a given door to avoid rapid spam
        # For instantaneous open/close set interval to 0.0
        self._door_min_toggle_interval = 0.0

        # Controller node (set by register_node)
        self._controller = None
        # Cached entity poses published over ROS topics: prim_path -> np.array([x,y,z])
        self._entity_poses: dict[str, np.ndarray] = {}
        # robot subscriptions by registered prim path
        self._robot_subs: dict[str, object] = {}
        # control verbose per-tick logging (DOOR_POS and DISTANCE). Set to False to silence.
        self._log_every_tick = False
        # Optional list of substrings to filter per-entity logs. If set, DISTANCE
        # logs will only be printed when any substring matches entity_path.
        # Example: door_manager._log_entity_filter = ['gazebo_actor']
        # or door_manager._log_entity_filter = ['jackal']
        self._log_entity_filter: list[str] | None = None
        # If True, hide door geometry by setting visibility instead of translating/scale
        # Useful for debugging and avoids moving shared ancestors (walls).
        self._use_visibility_toggle: bool = True

        self._debug_rate_limit = 1.0
        self._last_debug_time = 0.0

    def _rate_limited_debug(self, msg: str):
        try:
            now = time.time()
            if now - self._last_debug_time >= float(self._debug_rate_limit):
                _log_debug(msg)
                self._last_debug_time = now
        except Exception:
            _log_debug(msg)

    def register_node(self, controller):
        """Attach rclpy subscriptions to the provided controller node so DoorManager
        receives live poses for pedestrians and robots published by the task_generator.
        """
        self._controller = controller
        # subscribe to legacy people topic if available
        if People is not None:
            try:
                qos_depth = 10
                controller.create_subscription(People, '/task_generator_node/people', self._people_cb, qos_depth)
                _log_info('Subscribed to /task_generator_node/people for pedestrian poses')
            except Exception as e:
                _log_warn(f'Failed to subscribe to people topic: {e}')
        else:
            _log_warn('people_msgs.People message type not available; pedestrian topic not subscribed')

        # subscribe to new arena people topic if available
        if ArenaPedestrians is not None:
            try:
                controller.create_subscription(ArenaPedestrians, '/task_generator_node/arena_peds', self._people_cb, 10)
                _log_info('Subscribed to /task_generator_node/arena_peds for pedestrian poses')
            except Exception as e:
                _log_warn(f'Failed to subscribe to arena_peds topic: {e}')
        else:
            _log_debug('arena_people_msgs.Pedestrians not available; /task_generator_node/arena_peds not subscribed')

        # Also subscribe to simple registration topic so external processes can register prims
        if StdString is not None:
            try:
                controller.create_subscription(StdString, '/isaac/register_entity', self._register_entity_cb, 10)
                _log_info('Subscribed to /isaac/register_entity for external entity registrations')
            except Exception as e:
                _log_warn(f'Failed to subscribe to /isaac/register_entity: {e}')
        else:
            _log_warn('std_msgs.String not available; registration topic not subscribed')

        # ensure robot subscriptions for already-registered robots
        for prim_path in list(self._robots):
            try:
                self._ensure_robot_subscription(prim_path)
            except Exception as e:
                _log_warn(f'Failed to ensure robot subscription for {prim_path}: {e}')

    def _ensure_robot_subscription(self, prim_path: str):
        if self._controller is None:
            return
        if prim_path in self._robot_subs:
            return
        # derive robot name from prim path: /World/<robot_name>/...
        try:
            parts = prim_path.split('/')
            if len(parts) >= 3 and parts[1] == 'World':
                robot_name = parts[2]
                topic = f'/task_generator_node/{robot_name}/odom'
                if Odometry is not None:
                    try:
                        sub = self._controller.create_subscription(
                            Odometry,
                            topic,
                            lambda msg, p=prim_path: self._odom_cb(msg, p),
                            10
                        )
                        self._robot_subs[prim_path] = sub
                        _log_info(f'Subscribed to {topic} for robot {robot_name}')
                    except Exception as e:
                        _log_warn(f'Failed to subscribe to {topic}: {e}')
                else:
                    _log_warn('nav_msgs.Odometry not available; robot odom not subscribed')
        except Exception as e:
            _log_debug(f'_ensure_robot_subscription error: {e}')

    def _odom_cb(self, msg, prim_path: str):
        try:
            pose = getattr(msg, 'pose', None)
            if pose is None:
                return
            pb = getattr(pose, 'pose', pose)  # handle Odometry vs nested
            pos = getattr(pb, 'position', None)
            if pos is None:
                return
            self._entity_poses[prim_path] = np.array([pos.x, pos.y, pos.z])
            _log_debug(f'odom update for {prim_path}: {self._entity_poses[prim_path]}')
        except Exception as e:
            _log_debug(f'odom_cb error: {e}')

    def _people_cb(self, msg):
        """Unified people callback supporting multiple message types.

        Handles:
          - people_msgs/People (msg.people list)
          - arena_people_msgs/Pedestrians (msg.pedestrians list with nested position.position)
        """
        try:
            _log_debug('people topic message received')
            # support both attribute names
            people = getattr(msg, 'people', None) or getattr(msg, 'pedestrians', None)
            if not people:
                return

            def _extract_xyz_from_field(field) -> np.ndarray | None:
                if field is None:
                    return None
                # common direct x,y,z
                if hasattr(field, 'x') and hasattr(field, 'y') and hasattr(field, 'z'):
                    return np.array([float(getattr(field, 'x', 0.0)),
                                     float(getattr(field, 'y', 0.0)),
                                     float(getattr(field, 'z', 0.0))])
                # nested .position (e.g. arena_people_msgs -> position.position)
                inner = getattr(field, 'position', None)
                if inner and hasattr(inner, 'x'):
                    return np.array([float(getattr(inner, 'x', 0.0)),
                                     float(getattr(inner, 'y', 0.0)),
                                     float(getattr(inner, 'z', 0.0))])
                # nested .pose.position
                pose = getattr(field, 'pose', None)
                if pose:
                    inner2 = getattr(pose, 'position', None)
                    if inner2 and hasattr(inner2, 'x'):
                        return np.array([float(getattr(inner2, 'x', 0.0)),
                                         float(getattr(inner2, 'y', 0.0)),
                                         float(getattr(inner2, 'z', 0.0))])
                return None

            for p in people:
                # identifier: prefer stage_prefix, then name, then id
                stage_prefix = getattr(p, 'stage_prefix', None) or getattr(p, 'name', None) or getattr(p, 'id', None)

                # fields vary: try common ones
                pose_field = getattr(p, 'pose', None) or getattr(p, 'position', None)
                xyz = _extract_xyz_from_field(pose_field)
                if xyz is None:
                    # some messages have nested structure: p.position.position
                    alt = getattr(p, 'position', None)
                    xyz = _extract_xyz_from_field(alt)

                if xyz is None:
                    _log_debug(f'Could not extract position for pedestrian entry: {stage_prefix}')
                    continue

                # Map to a registered pedestrian prim path if any contains the name/id
                matched_prim = None
                if stage_prefix is not None:
                    for reg in self._pedestrians:
                        if stage_prefix in reg or reg in str(stage_prefix):
                            matched_prim = reg
                            break
                # fallback to use stage_prefix as prim path if it looks like a usd path
                if matched_prim is None and isinstance(stage_prefix, str) and stage_prefix.startswith('/'):
                    matched_prim = stage_prefix

                # if still None, use a name-based synthetic key so distance logic can still see it
                prim_key = matched_prim or f"ped|{stage_prefix}"

                self._entity_poses[prim_key] = xyz
                _log_debug(f'people update for {prim_key}: {xyz}')

        except Exception as e:
            _log_debug(f'people_cb error: {e}')

    def add_door(self, prim_path: str, kind: typing.Literal['sliding'] = 'sliding'):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            start_v, end_v = None, None
            try:
                start_attr = prim.GetAttribute('door_start')
                end_attr = prim.GetAttribute('door_end')
                if start_attr and end_attr:
                    start_v = start_attr.Get()
                    end_v = end_attr.Get()
            except Exception:
                pass
            if not start_v or not end_v:
                start_v = [0, 0, 0]
                end_v = [1, 0, 0]
            dx = end_v[0] - start_v[0]
            dy = end_v[1] - start_v[1]
            axis = np.array([dx, dy, 0])
            axis = axis / (np.linalg.norm(axis) + 1e-8)
            angle = np.arctan2(axis[1], axis[0])
            self._doors[prim_path] = {
                "prim": prim,
                "kind": kind,
                "initial_scale": Gf.Vec3f(1, 1, 1),
                "initial_translate": Gf.Vec3f(0, 0, 0),
                "move_prim_path": prim_path,
                "open": False,
                "last_toggle_time": 0.0,
                "axis": axis,
                "angle": angle,
                "anim": None
            }
            # Store initial scale
            scale_attr = prim.GetAttribute('xformOp:scale')
            if scale_attr:
                self._doors[prim_path]["initial_scale"] = scale_attr.Get()
            else:
                # If no scale attribute exists, try to get it from the xform
                xformable = UsdGeom.Xformable(prim)
                transform = xformable.GetLocalTransformation()
                # Extract scale from transformation matrix
                scale_x = transform.GetRow(0).GetLength()
                scale_y = transform.GetRow(1).GetLength()
                scale_z = transform.GetRow(2).GetLength()
                self._doors[prim_path]["initial_scale"] = Gf.Vec3f(scale_x, scale_y, scale_z)

            # Store initial local translation if present (fallback to local xform translation)
            translate_attr = prim.GetAttribute('xformOp:translate')
            if translate_attr and translate_attr.Get() is not None:
                self._doors[prim_path]["initial_translate"] = translate_attr.Get()
            else:
                # fallback: use local xform translation (preferred over world position)
                try:
                    xformable = UsdGeom.Xformable(prim)
                    transform = xformable.GetLocalTransformation()
                    tx = transform.GetRow(3)[0]
                    ty = transform.GetRow(3)[1]
                    tz = transform.GetRow(3)[2]
                    self._doors[prim_path]["initial_translate"] = Gf.Vec3f(float(tx), float(ty), float(tz))
                except Exception:
                    wp = self._get_prim_position(prim)
                    self._doors[prim_path]["initial_translate"] = Gf.Vec3f(float(wp[0]), float(wp[1]), float(wp[2]))
            try:
                xformable = UsdGeom.Xformable(prim)
                ops = xformable.GetOrderedXformOps()
                rot_quat = [0, 0, 0, 1]
                for op in ops:
                    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                        rot = op.Get()
                        from scipy.spatial.transform import Rotation as R
                        rot_quat = R.from_euler('xyz', rot, degrees=True).as_quat()
                        break
                    elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                        rot_quat = op.Get()
                        break
                self._doors[prim_path]["initial_rotation"] = rot_quat
            except Exception:
                self._doors[prim_path]["initial_rotation"] = [0, 0, 0, 1]

            try:
                for child in prim.GetAllChildren():
                    if not child or not child.IsValid():
                        continue
                    try:
                        _ = UsdGeom.Xformable(child)
                        self._doors[prim_path]["move_prim_path"] = child.GetPath().pathString
                        break
                    except Exception:
                        continue
            except Exception:
                pass

            _log_debug(f"Added door to door manager: {prim_path}")
        else:
            _log_warn(f"Failed to add door - invalid prim: {prim_path}")

    def reset(self):
        self._pedestrians.clear()
        # Close all doors on reset
        for door_path, door_data in self._doors.items():
            self._close_door(door_data)

    def update(self):
        stage = omni.usd.get_context().get_stage()
        entities_to_check = self._robots + self._pedestrians

        if not self._doors:
            _log_debug("No doors registered with DoorManager.")

        # Always print something each tick. If there are no entities, still print door positions.
        for door_path, door_data in self._doors.items():
            door_prim = door_data["prim"]
            if not door_prim.IsValid():
                _log_warn(f"Door prim invalid: {door_path}")
                continue

            door_pos = self._get_prim_position(door_prim)
            if self._log_every_tick:
                self._rate_limited_debug(f"[DOOR_POS] {door_path} -> {door_pos.tolist()}")

            if entities_to_check:
                # Compute positions for all entities and their distances to the door
                distances = []
                entity_positions = {}
                for entity_path in entities_to_check:
                    try:
                        if entity_path in self._entity_poses:
                            entity_pos = self._entity_poses[entity_path]
                        else:
                            entity_prim = stage.GetPrimAtPath(entity_path)
                            if not (entity_prim and entity_prim.IsValid()):
                                resolved_path = self._resolve_entity_prim(entity_path)
                                entity_prim = stage.GetPrimAtPath(resolved_path)
                                if entity_prim and entity_prim.IsValid():
                                    _log_debug(f"Resolved entity prim for distance checks: {entity_path} -> {resolved_path}")
                                    entity_path = resolved_path
                        if not (entity_prim and entity_prim.IsValid()):
                            _log_warn(f"Entity prim invalid or missing: {entity_path}")
                            continue
                        entity_pos = self._get_prim_position(entity_prim)
                        dist = float(np.linalg.norm(door_pos - entity_pos))
                        distances.append((dist, entity_path))
                        entity_positions[entity_path] = entity_pos
                    except Exception as e:
                        _log_debug(f'Error computing distance for {entity_path}: {e}')

                if not distances:
                    if self._log_every_tick:
                        _log_info(f"DISTANCE: Door {door_path} -> (no valid entities found)")
                    continue

                # Use the minimum distance across all entities to decide open/close
                distances.sort(key=lambda x: x[0])
                min_distance, closest_entity = distances[0]

                # Optionally print per-entity distance for the closest entity only
                if self._log_every_tick:
                    if (self._log_entity_filter is None or any(substr in closest_entity for substr in self._log_entity_filter)):
                        _log_info(f"DISTANCE: Door {door_path} -> Entity {closest_entity} = {min_distance:.3f} m (open={door_data['open']})")

                # Hysteresis + cooldown: decide once per door using min_distance
                now = time.time()
                if min_distance < self._door_open_distance:
                    if (not door_data["open"] and (now - door_data.get("last_toggle_time", 0.0) > self._door_min_toggle_interval)):
                        _log_info(f"Opening door {door_path} (closest entity {closest_entity} within {self._door_open_distance}m)")
                        self._open_door(door_data)
                        door_data["last_toggle_time"] = now
                elif min_distance > (self._door_open_distance + self._door_close_margin):
                    if (door_data["open"] and (now - door_data.get("last_toggle_time", 0.0) > self._door_min_toggle_interval)):
                        _log_info(f"Closing door {door_path} (no entities within {self._door_open_distance}m)")
                        self._close_door(door_data)
                        door_data["last_toggle_time"] = now
            else:
                # No entities registered — optionally print a placeholder distance message
                if self._log_every_tick:
                    _log_info(f"DISTANCE: Door {door_path} -> (no entities registered)")

        for door_path, door_data in self._doors.items():
            if door_data.get("anim"):
                anim = door_data["anim"]
                now = time.time()
                t = min((now - anim["start_time"]) / anim["duration"], 1.0)
                interp_pos = tuple(
                    anim["start"][i] + (anim["end"][i] - anim["start"][i]) * t
                    for i in range(3)
                )
                from isaac_utils.utils import geom
                rot_quat = door_data.get("initial_rotation", [0, 0, 0, 1])
                if hasattr(rot_quat, "GetReal") and hasattr(rot_quat, "GetImaginary"):
                    w = rot_quat.GetReal()
                    x, y, z = rot_quat.GetImaginary()
                elif isinstance(rot_quat, (list, tuple, np.ndarray)) and len(rot_quat) == 4:
                    x, y, z, w = rot_quat
                else:
                    x, y, z, w = 0, 0, 0, 1
                geom.move(
                    prim_path=door_data.get("move_prim_path", door_data["prim"].GetPath().pathString),
                    translation=geom.Translation(*interp_pos),
                    rotation=geom.Rotation(w=w, x=x, y=y, z=z),
                )
                if t >= 1.0:
                    door_data["anim"] = None

    def _get_prim_position(self, prim):
        try:
            xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            matrix = xform_cache.GetLocalToWorldTransform(prim)
            tx = matrix[3][0]
            ty = matrix[3][1]
            tz = matrix[3][2]
            return np.array([tx, ty, tz])
        except Exception:
            xformable = UsdGeom.Xformable(prim)
            translation = xformable.GetLocalTransformation().GetRow(3)
            return np.array([translation[0], translation[1], translation[2]])

    def _dump_prim_transform_info(self, prim):
        """Diagnostic: log transform ops and attributes for prim, parents and children."""
        try:
            if not prim or not prim.IsValid():
                _log_info('Diagnostic: prim invalid or not found')
                return
            _log_info(f'Diagnostic for prim: {prim.GetPath().pathString}')
            try:
                _log_info(f'  typeName={prim.GetTypeName()}, IsInstance={prim.IsInstance()}')
            except Exception:
                pass
            # Xform ops for the prim
            try:
                xf = UsdGeom.Xformable(prim)
                ops = xf.GetOrderedXformOps()
                _log_info('  ordered xform ops: ' + str([o.GetOpName() for o in ops]))
            except Exception as e:
                _log_info(f'  prim not xformable: {e}')

            # common transform attributes
            for name in ('xformOp:translate', 'xformOp:scale', 'xformOp:transform'):
                try:
                    a = prim.GetAttribute(name)
                    if a and a.HasAuthoredValue():
                        _log_info(f'  {name} = {a.Get()}')
                    else:
                        _log_info(f'  {name} = <not authored>')
                except Exception as e:
                    _log_debug(f'  reading {name} failed: {e}')

            # walk parents up to a few levels
            parent = prim.GetParent()
            depth = 0
            while parent and parent.IsValid() and depth < 5:
                try:
                    _log_info(f'  parent: {parent.GetPath().pathString} type={parent.GetTypeName()} IsInstance={parent.IsInstance()}')
                    try:
                        pxf = UsdGeom.Xformable(parent)
                        pops = pxf.GetOrderedXformOps()
                        _log_info('    ordered xform ops: ' + str([o.GetOpName() for o in pops]))
                    except Exception:
                        _log_info('    parent not xformable')
                    for name in ('xformOp:translate', 'xformOp:scale', 'xformOp:transform'):
                        try:
                            a = parent.GetAttribute(name)
                            if a and a.HasAuthoredValue():
                                _log_info(f'    {name} = {a.Get()}')
                            else:
                                _log_info(f'    {name} = <not authored>')
                        except Exception:
                            pass
                except Exception as e:
                    _log_debug(f'  parent info failed: {e}')
                parent = parent.GetParent()
                depth += 1

            # list children attributes
            for child in prim.GetAllChildren():
                try:
                    _log_info(f'  child: {child.GetPath().pathString} type={child.GetTypeName()} IsInstance={child.IsInstance()}')
                    try:
                        cxf = UsdGeom.Xformable(child)
                        cops = cxf.GetOrderedXformOps()
                        _log_info('    ordered xform ops: ' + str([o.GetOpName() for o in cops]))
                    except Exception:
                        _log_info('    child not xformable')
                    for name in ('xformOp:translate', 'xformOp:scale', 'xformOp:transform'):
                        try:
                            a = child.GetAttribute(name)
                            if a and a.HasAuthoredValue():
                                _log_info(f'    {name} = {a.Get()}')
                            else:
                                _log_info(f'    {name} = <not authored>')
                        except Exception:
                            pass
                except Exception as e:
                    _log_debug(f'  child info failed: {e}')
        except Exception as e:
            _log_debug(f'_dump_prim_transform_info error: {e}')

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
                    _log_debug(f'Failed to set visibility on {prim.GetPath().pathString}: {e}')
            # optionally apply to children
            if recursive:
                for child in prim.GetAllChildren():
                    try:
                        self._set_visibility(child, visible, recursive=True)
                    except Exception:
                        pass
            _log_debug(f"Set visibility {val} on {prim.GetPath().pathString}")
            return True
        except Exception as e:
            _log_debug(f'_set_visibility error: {e}')
            return False

    def _open_door(self, door_data):
        kind = door_data["kind"]
        prim_path = door_data.get("move_prim_path", door_data["prim"].GetPath().pathString)
        init_t = door_data.get("initial_translate", Gf.Vec3f(0, 0, 0))
        init_scale = door_data.get("initial_scale", Gf.Vec3f(1, 1, 1))
        axis = door_data.get("axis", np.array([1, 0, 0]))
        angle = door_data.get("angle", 0.0)

        if kind == 'sliding':
            slide_dir = np.array([-axis[1], axis[0], 0])
            try:
                size_x = float(init_scale[0])
            except Exception:
                size_x = 1.0
            opening_offset = max(0.5, size_x * 0.9)
            target_t = (
                float(init_t[0]) + opening_offset * slide_dir[0],
                float(init_t[1]) + opening_offset * slide_dir[1],
                float(init_t[2])
            )
            door_data["anim"] = {
                "start": tuple(init_t),
                "end": target_t,
                "angle": angle,
                "start_time": time.time(),
                "duration": 3.0,
                "opening": True
            }
            door_data["open"] = True

        elif kind == 'sliding_top':
            # Move up in Z by the door's height (or use scale[2])
            try:
                size_z = float(init_scale[2])
            except Exception:
                size_z = 2.0
            opening_offset = max(0.5, size_z * 0.9)
            target_t = (
                float(init_t[0]),
                float(init_t[1]),
                float(init_t[2]) + opening_offset
            )
            door_data["anim"] = {
                "start": tuple(init_t),
                "end": target_t,
                "angle": angle,
                "start_time": time.time(),
                "duration": 3.0,
                "opening": True
            }
            door_data["open"] = True

        else:
            prim = door_data["prim"]
            self._set_visibility(prim, visible=False, recursive=True)
            door_data["open"] = True

    def _close_door(self, door_data):
        kind = door_data["kind"]
        prim_path = door_data.get("move_prim_path", door_data["prim"].GetPath().pathString)
        init_t = door_data.get("initial_translate", Gf.Vec3f(0, 0, 0))
        init_scale = door_data.get("initial_scale", Gf.Vec3f(1, 1, 1))
        axis = door_data.get("axis", np.array([1, 0, 0]))
        angle = door_data.get("angle", 0.0)

        if kind == 'sliding':
            slide_dir = np.array([-axis[1], axis[0], 0])
            try:
                size_x = float(init_scale[0])
            except Exception:
                size_x = 1.0
            opening_offset = max(0.5, size_x * 0.9)
            open_t = (
                float(init_t[0]) + opening_offset * slide_dir[0],
                float(init_t[1]) + opening_offset * slide_dir[1],
                float(init_t[2])
            )
            door_data["anim"] = {
                "start": open_t,
                "end": tuple(init_t),
                "angle": angle,
                "start_time": time.time(),
                "duration": 3.0,
                "opening": False
            }
            door_data["open"] = False

        elif kind == 'sliding_top':
            try:
                size_z = float(init_scale[2])
            except Exception:
                size_z = 2.0
            opening_offset = max(0.5, size_z * 0.9)
            open_t = (
                float(init_t[0]),
                float(init_t[1]),
                float(init_t[2]) + opening_offset
            )
            door_data["anim"] = {
                "start": open_t,
                "end": tuple(init_t),
                "angle": angle,
                "start_time": time.time(),
                "duration": 3.0,
                "opening": False
            }
            door_data["open"] = False

        else:
            prim = door_data["prim"]
            self._set_visibility(prim, visible=True, recursive=True)
            door_data["open"] = False

    def _resolve_entity_prim(self, prim_path: str) -> str:
        """Try to resolve a dynamic/moving sub-prim for a given USD prim path.
        If the given prim is valid return it, otherwise attempt common suffixes
        and a shallow child-name search. Always return a string path (fallback
        to the original prim_path).
        """
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                return prim_path

            # Common candidate suffixes for robots/actors
            candidates = [
                'base_link',
                'base_footprint',
                'base',
                'man_root',
                'ManRoot',
                'root',
                'actor',
            ]
            for c in candidates:
                p = prim_path.rstrip('/') + '/' + c
                pr = stage.GetPrimAtPath(p)
                if pr and pr.IsValid():
                    return p

            # Shallow search children for likely moving prims
            if prim and prim.IsValid():
                for child in prim.GetChildren():
                    name = child.GetName().lower()
                    if any(k in name for k in ('base', 'root', 'man', 'actor')):
                        return child.GetPath().pathString
        except Exception as e:
            _log_debug(f'_resolve_entity_prim error: {e}')
        return prim_path

    def add_robot(self, prim_path: str):
        """Register a robot prim for distance checks. Resolves to a moving
        sub-prim if possible and ensures an odom subscription is created when
        a controller node is registered.
        """
        resolved = self._resolve_entity_prim(prim_path)
        if resolved not in self._robots:
            self._robots.append(resolved)
        _log_debug(f"Added robot to door manager: {prim_path} -> resolved: {resolved}")
        try:
            self._ensure_robot_subscription(resolved)
        except Exception as e:
            _log_warn(f'Failed to ensure robot subscription on add: {e}')

    def add_pedestrian(self, prim_path: str):
        """Register a pedestrian prim for distance checks. Poses for
        pedestrians are primarily updated via the people topic; this method
        just records the prim path.
        """
        resolved = self._resolve_entity_prim(prim_path)
        if resolved not in self._pedestrians:
            self._pedestrians.append(resolved)
        _log_debug(f"Added pedestrian to door manager: {prim_path} -> resolved: {resolved}")

    def _register_entity_cb(self, msg):
        """Handle simple registration messages published on /isaac/register_entity.
        Expected payload: '<role>|<prim_path>' where role is 'robot' or 'pedestrian'.
        """
        try:
            data = getattr(msg, 'data', '')
            _log_info(f'REGISTER_ENTITY received: {data}')
            if not data:
                return
            parts = data.split('|', 1)
            if len(parts) != 2:
                _log_warn(f'Invalid register_entity payload: {data}')
                return
            role, prim_path = parts[0], parts[1]
            role = role.strip().lower()
            prim_path = prim_path.strip()
            if role == 'robot':
                self.add_robot(prim_path)
                _log_info(f'Registered robot: {prim_path}')
            elif role == 'pedestrian' or role == 'ped':
                self.add_pedestrian(prim_path)
                _log_info(f'Registered pedestrian: {prim_path}')
            else:
                _log_warn(f'Unknown role in register_entity: {role}')
            # show current registry
            _log_debug(f'Current robots: {self._robots}, pedestrians: {self._pedestrians}')
        except Exception as e:
            _log_debug(f'_register_entity_cb error: {e}')


door_manager = DoorManager()
