import os
import xml.etree.ElementTree as ET

import arena_robots.Robot
import carb
import omni.usd
from pxr import UsdPhysics

from .differential import differential
from .joint_controller import joint_controller
from .mecanum import mecanum
from .topic_bridge import topic_bridge

_ARM_CONTROLLER_TYPES = frozenset({
    'joint_trajectory_controller/JointTrajectoryController',
    'position_controllers/JointGroupPositionController',
    'forward_command_controller/ForwardCommandController',
})

# Position-drive defaults for arm/lift joints. URDF import leaves stiffness at
# 0, so gravity wins and the chain sags; these values let the articulation
# actually track a position setpoint.
_ARM_DRIVE_STIFFNESS = 4.0e5
_ARM_DRIVE_DAMPING = 4.0e4


def _set_position_drive(joint_prim, stiffness: float, damping: float) -> None:
    for axis in ('angular', 'linear'):
        drive = UsdPhysics.DriveAPI(joint_prim, axis)
        stiffness_attr = drive.GetStiffnessAttr()
        if stiffness_attr.IsValid():
            stiffness_attr.Set(stiffness)
            drive.GetDampingAttr().Set(damping)


_BRIDGE_CONTROLLER_TYPES = frozenset({
    'joint_trajectory_controller/JointTrajectoryController',
    'position_controllers/JointGroupPositionController',
    'forward_command_controller/ForwardCommandController',
    'diff_drive_controller/DiffDriveController',
    'mecanum_drive_controller/MecanumDriveController',
    'joint_state_broadcaster/JointStateBroadcaster',
})


def _joints_from_urdf(urdf_path: str) -> tuple[list[str], list[str]]:
    """Return (velocity_joints, position_joints) declared in any <ros2_control>
    block of the URDF. The bridge graph reflects whatever the URDF says is a
    ros2_control command interface, regardless of which controller class
    claims it; this lets new controller types Just Work without per-type
    extractors on the Isaac side.
    """
    velocity: list[str] = []
    position: list[str] = []
    tree = ET.parse(urdf_path)
    for rc in tree.getroot().iter('ros2_control'):
        for joint in rc.iter('joint'):
            joint_name = joint.get('name')
            if not joint_name:
                continue
            for cmd in joint.iter('command_interface'):
                kind = cmd.get('name')
                if kind == 'velocity':
                    velocity.append(joint_name)
                elif kind == 'position':
                    position.append(joint_name)
    return velocity, position


class Control:
    def __init__(
        self,
        prim_path: str,
        target_prim_path: str,
        cmd_vel_topic: str,
        urdf_path: str,
        commands_topic: str | None = None,
        states_topic: str | None = None,
    ):
        self.prim_path: str = prim_path
        self.target_prim_path: str = target_prim_path
        self.cmd_vel_topic: str = cmd_vel_topic
        self.urdf_path: str = urdf_path

        # Derive bridge topics from cmd_vel_topic namespace when not given explicitly.
        ns = cmd_vel_topic.rsplit('/', 1)[0] if '/' in cmd_vel_topic else ''
        self.commands_topic: str = commands_topic if commands_topic is not None else f"{ns}/isaac/joint_commands"
        self.states_topic: str = states_topic if states_topic is not None else f"{ns}/isaac/joint_states"

    def parse(self, robot_model: str) -> bool:
        """
        Gets configuration for given robot type and spawns controllers.
        Native ros2_control controller types map to bespoke OmniGraphs; anything
        else triggers a generic topic_bridge graph that mirrors the URDF's
        ros2_control joints into a JointState command/state topic pair.
        """
        robot = arena_robots.Robot.RobotIdentifier(robot_model).resolve_sync()

        has_bridge_controller = False

        for controller_name, config in robot.control['controller_manager']['ros__parameters'].items():
            if not isinstance(config, dict):
                continue
            ctype = config.get('type')
            if ctype == 'diff_drive_controller/DiffDriveController':
                if not self._parse_differential(controller_name, robot.control[controller_name]['ros__parameters']):
                    return False
            elif ctype == 'mecanum_drive_controller/MecanumDriveController':
                if not self._parse_mecanum(controller_name, robot.control[controller_name]['ros__parameters']):
                    return False
            elif ctype in _ARM_CONTROLLER_TYPES:
                arm_config = robot.control[controller_name]['ros__parameters']
                if not self._parse_arm(controller_name, arm_config):
                    return False
            elif ctype is not None and ctype not in _BRIDGE_CONTROLLER_TYPES:
                has_bridge_controller = True

        if has_bridge_controller:
            if not self._dispatch_bridge():
                return False

        return True

    def _parse_differential(
        self,
        controller_name: str,
        diff_drive_config: dict,
    ):
        wheel_distance = diff_drive_config['wheel_separation']
        wheel_radius = diff_drive_config['wheel_radius']
        max_linear_speed = diff_drive_config.get('linear.x.max_velocity', 1.0)
        min_linear_speed = diff_drive_config.get('linear.x.min_velocity', -max_linear_speed)
        max_angular_speed = diff_drive_config.get('angular.z.max_velocity', 1.0)
        min_angular_speed = diff_drive_config.get('angular.z.min_velocity', -max_angular_speed)

        left_wheels = diff_drive_config['left_wheel_names']
        right_wheels = diff_drive_config['right_wheel_names']

        for i, (left_wheel, right_wheel) in enumerate(zip(left_wheels, right_wheels)):
            if not differential(
                graph_path=os.path.join(self.prim_path, f'{controller_name}_{i}'),
                prim_path=self.target_prim_path,
                cmd_vel_topic=self.cmd_vel_topic,
                joint_names=[left_wheel, right_wheel],
                wheel_distance=wheel_distance,
                wheel_radius=wheel_radius,
                max_linear_speed=max_linear_speed,
                min_linear_speed=min_linear_speed,
                max_angular_speed=max_angular_speed,
                min_angular_speed=min_angular_speed,
            ):
                return False
        return True

    def _parse_arm(
        self,
        controller_name: str,
        arm_config: dict,
    ):
        # Single JointState command topic per controller; the Isaac graph
        # forwards inbound joint names verbatim to the articulation, so users
        # publish only the joints they want to drive.
        stage = omni.usd.get_context().get_stage()
        for joint_name in arm_config.get('joints', []):
            joint_prim = stage.GetPrimAtPath(f"{self.prim_path}/joints/{joint_name}")
            if joint_prim.IsValid():
                _set_position_drive(joint_prim, _ARM_DRIVE_STIFFNESS, _ARM_DRIVE_DAMPING)

        return joint_controller(
            graph_path=os.path.join(self.prim_path, controller_name),
            prim_path=self.target_prim_path,
            state_topic=f"{controller_name}/joint_states",
            command_topic=f"{controller_name}/joint_state_command",
        )

    def _parse_mecanum(
        self,
        controller_name: str,
        mecanum_config: dict,
    ):
        # The ros2_control mecanum_drive_controller exposes per-corner joints
        # by explicit name keys; prefer those so wheel order is unambiguous.
        joint_names = [
            mecanum_config['front_left_wheel_command_joint_name'],
            mecanum_config['rear_left_wheel_command_joint_name'],
            mecanum_config['rear_right_wheel_command_joint_name'],
            mecanum_config['front_right_wheel_command_joint_name'],
        ]
        wheel_radius = mecanum_config['kinematics.wheels_radius']
        wheel_base_sum = mecanum_config['kinematics.sum_of_robot_center_projection_on_X_Y_axis']
        max_linear_speed = mecanum_config['linear.x.max_velocity']
        max_lateral_speed = mecanum_config['linear.y.max_velocity']
        max_angular_speed = mecanum_config['angular.z.max_velocity']

        return mecanum(
            graph_path=os.path.join(self.prim_path, controller_name),
            prim_path=self.target_prim_path,
            cmd_vel_topic=self.cmd_vel_topic,
            joint_names=joint_names,
            wheel_radius=wheel_radius,
            wheel_base_sum=wheel_base_sum,
            max_linear_speed=max_linear_speed,
            max_lateral_speed=max_lateral_speed,
            max_angular_speed=max_angular_speed,
        )

    def _dispatch_bridge(self) -> bool:
        """Build a generic topic_bridge graph claiming every ros2_control
        joint declared in the URDF, partitioned by command_interface name.
        """
        joints_velocity, joints_position = _joints_from_urdf(self.urdf_path)
        if not joints_velocity and not joints_position:
            carb.log_error(
                f"topic_bridge: no ros2_control joints found in URDF '{self.urdf_path}'"
            )
            return False

        return topic_bridge(
            graph_path=os.path.join(self.prim_path, 'topic_bridge'),
            prim_path=self.target_prim_path,
            joints_velocity=joints_velocity,
            joints_position=joints_position,
            commands_topic=self.commands_topic,
            states_topic=self.states_topic,
        )


__all__ = ['Control']
