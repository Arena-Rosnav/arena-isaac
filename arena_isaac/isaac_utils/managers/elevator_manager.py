import time
import numpy as np

try:
    from rclpy.qos import QoSProfile
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String as StdString
except Exception:
    QoSProfile = None
    Odometry = None
    StdString = None

try:
    from rclpy.logging import get_logger
    _LOGGER = get_logger('isaac_elevator_manager')
except Exception:
    _LOGGER = None


def _log_info(msg: str):
    try:
        if _LOGGER:
            _LOGGER.info(msg)
            return
    except Exception:
        pass
        try:
            if _LOGGER:
                _LOGGER.info(msg)
        except Exception:
            pass


def _log_warn(msg: str):
    try:
        if _LOGGER:
            _LOGGER.warn(msg)
            return
    except Exception:
        pass
        try:
            if _LOGGER:
                _LOGGER.warn(msg)
        except Exception:
            pass


class ElevatorManager:
    # Biến lưu trữ instance duy nhất
    _instance = None

    @staticmethod
    def instance():
        """
        Phương thức tĩnh để truy cập instance duy nhất.
        """
        if ElevatorManager._instance is None:
            ElevatorManager._instance = ElevatorManager()
        return ElevatorManager._instance
        
    def __init__(self):
        self._elevators = {}
        self._pairs = []
        self._robots = []
        self._odom_cache = {}
        self._odom_subs = {}
        self._cooldowns = {}

        self._controller = None
        # robot subscriptions by registered prim path
        self._robot_subs: dict[str, object] = {}
    def register_node(self, controller):
        self._controller = controller
        # Subscribe to robot odometry for all registered robots
        for robot_name in self._robots:
            self._ensure_robot_subscription(robot_name)

    def add_elevator(self, elevator, destination):
        self._elevators[elevator.name] = elevator
        # Subscribe to robot odometry for all registered robots
        try:
            # Pair elevators by destination
            dest = self._elevators.get(destination)
            _LOGGER.info(str(elevator))
            self._pairs.append({
                'a': {'name': elevator.name, 'position': elevator.position, 'size': elevator.size},
                'b': {'name': dest.name, 'position': dest.position, 'size': dest.size},
                'cooldown': {},
            })
        except:
            pass

    def add_robot(self, prim_path):
        robot_name = prim_path.split("/")[-1]  # Get last part: /World/Robots/jackal -> jackal
        if robot_name not in self._robots:
            _LOGGER.info(f"Add {robot_name} into World")
            self._robots.append(robot_name)
            for robot in self._robots:
                _LOGGER.info(f"Robot {robot} in World")
            self._ensure_robot_subscription(prim_path)

    def _ensure_robot_subscription(self, prim_path):
        robot_name = prim_path.split("/")[-1]  # Get last part: /World/Robots/jackal -> jackal
        if self._controller is None or robot_name in self._odom_subs:
            return
        topic = f"/{robot_name}/odom"
        try:
            topic = f'/task_generator_node/{robot_name}/odom'
            if Odometry is not None:
                try:
                    sub = self._controller.create_subscription(
                        Odometry,
                        topic,
                        lambda msg, rn=robot_name: self.odom_cb(msg, rn),  # Capture robot_name correctly
                        10
                    )
                    self._odom_subs[robot_name] = sub
                    _log_info(f"Subscribed to {topic} for robot {robot_name}")
                except Exception as e:
                    _log_warn(f'Failed to subscribe to {topic}: {e}')
            else:
                _log_warn('nav_msgs.Odometry not available; robot odom not subscribed')
        except Exception as e:
            _LOGGER.info("Error")



    def odom_cb(self, msg, robot_name):
        try:
            pos = msg.pose.pose.position
            _LOGGER.info(f"Received odom for {robot_name}: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
            self._odom_cache[robot_name] = (pos.x, pos.y, pos.z)
        except Exception as e:
            _log_warn(f"odom_cb failed for {robot_name}: {e}")

    def get_robot_pose(self, robot_name):
        return self._odom_cache.get(robot_name, None)
    def get_robots(self):
        return self._robots
    def update(self):
        now = time.time()
        # _LOGGER.info(str(self._robots))
        cooldown_sec = 5
        if not self._pairs:
            _LOGGER.info("No elevator was add")
        for pair in self._pairs:
            for robot_name in self._robots:
                robot_pose = self.get_robot_pose(robot_name)
                if robot_pose is None:
                    _LOGGER.warn("No Robot pose was added")
                    continue
                state = pair['cooldown'].get(robot_name, {'last_tp': 0, 'can_tp': True, 'was_on': 'none'})
                last_tp = state.get('last_tp', 0)
                can_tp = state.get('can_tp', True)
                was_on = state.get('was_on', 'none')
                _LOGGER.info(str(pair['a']))
                _LOGGER.info(str(pair['b']))
                on_a = self._robot_on_platform(robot_pose, pair['a'])
                on_b = self._robot_on_platform(robot_pose, pair['b'])
                _LOGGER.info(f"Laying on a is: {str(on_a)} {was_on} {can_tp}")
                _LOGGER.info(f"Laying on b is: {str(on_b)}")
                # Only allow teleport if robot is on a platform, was previously off both, and cooldown expired
                if can_tp and (on_a ^ on_b) and not (was_on == 'a' and on_a) and not (was_on == 'b' and on_b) and (now - last_tp > cooldown_sec):
                    if on_a:
                        self.teleport_robot(robot_name, pair['b']['position'])
                        pair['cooldown'][robot_name] = {'last_tp': now, 'can_tp': False, 'was_on': 'a'}
                    elif on_b:
                        self.teleport_robot(robot_name, pair['a']['position'])
                        pair['cooldown'][robot_name] = {'last_tp': now, 'can_tp': False, 'was_on': 'b'}
                # Reset teleport permission only when robot is fully off both platforms
                elif not on_a and not on_b:
                    pair['cooldown'][robot_name] = {'Checklast_tp': last_tp, 'can_tp': True, 'was_on': 'none'}
                else:
                    pair['cooldown'][robot_name] = {'last_tp': last_tp, 'can_tp': can_tp, 'was_on': 'a' if on_a else 'b' if on_b else 'none'}

    def _robot_on_platform(self, robot_pose, platform):
        px = platform['position'].x
        py = platform['position'].y
        pz = platform['position'].z
        sx = platform['size'].x
        sy = platform['size'].y
        sz = platform['size'].z
        rx, ry, rz = robot_pose
        return (
            abs(rx - px) <= sx / 2 and
            abs(ry - py) <= sy / 2 and
            abs(rz - pz) <= max(sz / 2, 0.5)
        )

    def teleport_robot(self, robot_name, position):
        try:
            import omni.usd
            from omni.isaac.core.prims import XFormPrim
            
            stage = omni.usd.get_context().get_stage()
            
            # Try several possible prim paths
            # IMPORTANT: base_link (articulation root) moves with physics, parent xform does NOT
            # So we must teleport base_link, not the parent xform
            possible_paths = [
                f"/World/Robots/{robot_name}/base_link",  # Articulation root - this is what moves!
                f"/World/{robot_name}/base_link",
                f"/World/Robots/{robot_name}",  # Fallback to parent xform
                f"/World/{robot_name}",
            ]
            
            for prim_path in possible_paths:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and prim.IsValid():
                    _log_info(f"Found robot prim for {robot_name} at {prim_path}")
                    # Use XFormPrim wrapper to set world pose
                    xform = XFormPrim(prim_path)
                    # position should be (x, y, z)
                    pos = position
                    if hasattr(position, 'x'):  # geometry_msgs.Point
                        pos = [position.x, position.y, position.z]
                    xform.set_world_pose(position=np.array(pos))
                    _log_info(f"Teleported robot {robot_name} to {pos}")
                    return
                    
            _log_warn(f"Could not find prim for robot {robot_name} at any of: {possible_paths}")
        except Exception as e:
            import traceback
            _log_warn(f"Teleport failed for {robot_name}: {e}\n{traceback.format_exc()}")


elevator_manager = ElevatorManager.instance()
