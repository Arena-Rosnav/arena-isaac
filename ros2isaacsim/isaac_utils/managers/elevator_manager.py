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
    def __init__(self):
        self._elevators = {}
        self._pairs = []
        self._robots = []
        self._odom_cache = {}
        self._odom_subs = {}
        self._cooldowns = {}

        self._controller = None

    def register_node(self, controller):
        self._controller = controller
        # Subscribe to robot odometry for all registered robots
        for robot_name in self._robots:
            self._ensure_robot_subscription(robot_name)

    def add_elevator(self, elevator, destination):
        self._elevators[elevator.name] = elevator
        # Pair elevators by destination
        dest = self._elevators.get(destination)
        # Subscribe to robot odometry for all registered robots
        self._pairs.append({
            'a': {'name': elevator.name, 'position': elevator.position, 'size': elevator.size},
            'b': {'name': dest.name, 'position': dest.position, 'size': dest.size},
            'cooldown': {},
        })

    def add_robot(self, robot_name):
        if robot_name not in self._robots:
            self._robots.append(robot_name)
            self._ensure_robot_subscription(robot_name)

    def _ensure_robot_subscription(self, robot_name):
        if self._controller is None or robot_name in self._odom_subs:
            return
        topic = f"/{robot_name}/odom"

    def odom_cb(msg):
        try:
            pos = msg.pose.pose.position
            self._odom_cache[robot_name] = (pos.x, pos.y, getattr(pos, 'z', 0.0))
        except Exception as e:
            _log_warn(f"odom_cb failed for {robot_name}: {e}")
        sub = self._controller.create_subscription(Odometry, topic, odom_cb, 10)
        self._odom_subs[robot_name] = sub
        _log_info(f"Subscribed to odometry for robot {robot_name} on topic {topic}")

    def get_robot_pose(self, robot_name):
        return self._odom_cache.get(robot_name, None)

    def update(self):
        for robot_name in self._robots:
            robot_pose = self.get_robot_pose(robot_name)
            if robot_pose is None:
                continue
            state = pair['cooldown'].get(robot_name, {'last_tp': 0, 'can_tp': True, 'was_on': 'none'})
            last_tp = state.get('last_tp', 0)
        now = time.time()
        for pair in self._pairs:
            for robot_name in self._robots:
                robot_pose = self.get_robot_pose(robot_name)
                if robot_pose is None:
                    continue
                state = pair['cooldown'].get(robot_name, {'last_tp': 0, 'can_tp': True, 'was_on': 'none'})
                last_tp = state.get('last_tp', 0)
                can_tp = state.get('can_tp', True)
                was_on = state.get('was_on', 'none')
                on_a = self._robot_on_platform(robot_pose, pair['a'])
                on_b = self._robot_on_platform(robot_pose, pair['b'])
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
                    pair['cooldown'][robot_name] = {'last_tp': last_tp, 'can_tp': True, 'was_on': 'none'}
                else:
                    pair['cooldown'][robot_name] = {'last_tp': last_tp, 'can_tp': can_tp, 'was_on': 'a' if on_a else 'b' if on_b else 'none'}

    def _robot_on_platform(self, robot_pose, platform):
        px, py, pz = platform['position']
        sx, sy, sz = platform['size']
        rx, ry, rz = robot_pose
        return (
            abs(rx - px) <= sx / 2 and
            abs(ry - py) <= sy / 2 and
            abs(rz - pz) <= max(sz / 2, 0.5)
        )

    def teleport_robot(self, robot_name, position):
        try:
            import omni
            from omni.isaac.core import World
            world = World.instance()
            # Try several possible prim paths
            possible_paths = [
                f"/World/Robots/{robot_name}",
                f"/World/{robot_name}",
                f"/World/Robots/{robot_name}/base_link",
            ]
            prim = None
            for prim_path in possible_paths:
                prim = world.scene.get_object(prim_path)
                if prim:
                    _log_info(f"Found robot prim for {robot_name} at {prim_path}")
                    break
            if prim:
                prim.set_world_pose(np.array(position))
                _log_info(f"Teleported robot {robot_name} to {position}")
            else:
                _log_warn(f"Could not find prim for robot {robot_name} at any of: {possible_paths}")
        except Exception as e:
            _log_warn(f"Teleport failed for {robot_name}: {e}")


elevator_manager = ElevatorManager()
