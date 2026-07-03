import math
import os
import xml.etree.ElementTree as ET

import carb
import omni.usd
from pxr import Usd, UsdPhysics

from isaac_utils.graphs import physics_engine, register_rebuilder

from .topic_bridge import topic_bridge

_RAD2DEG = 180.0 / math.pi

# Drive defaults. URDF import leaves stiffness/damping at 0, so position drives
# can't hold setpoints (gravity wins) and velocity drives can't apply force.
# These values let the articulation actually track commands fed via TopicBasedSystem.
_POSITION_DRIVE_STIFFNESS = 4.0e5
_POSITION_DRIVE_DAMPING = 4.0e4
_VELOCITY_DRIVE_DAMPING = 1.0e4
# mjwarp applies velocity-actuator feedback per step, the physx-scale gain
# (5.7e5 N m s/rad after deg conversion) explodes wheel DOFs to NaN within
# frames of ground contact, and an unclamped drive at any gain delivers the
# full velocity-error torque at once (hundreds of N m, backflips the base)
_VELOCITY_DRIVE_DAMPING_NEWTON = 0.35
_VELOCITY_DRIVE_MAX_FORCE_NEWTON = 16.0
# physx-scale position gains saturate mjwarp's implicit integrator, newton
# derives per-joint gains from the urdf effort limit instead: kp = 4*effort
# N m/rad, kd = kp/20, torque clamped at the effort limit
_POSITION_DRIVE_KP_PER_EFFORT_NEWTON = 4.0
_POSITION_DRIVE_KP_KD_RATIO_NEWTON = 20.0
_POSITION_DRIVE_EFFORT_FALLBACK_NEWTON = 25.0


def _set_drive(joint_prim, stiffness: float, damping: float, max_force: float | None = None) -> None:
    for axis in ('angular', 'linear'):
        drive = UsdPhysics.DriveAPI(joint_prim, axis)
        stiffness_attr = drive.GetStiffnessAttr()
        if stiffness_attr.IsValid():
            stiffness_attr.Set(stiffness)
            drive.GetDampingAttr().Set(damping)
            if max_force is not None:
                drive.CreateMaxForceAttr(max_force)


def _joints_from_urdf(urdf_path: str) -> tuple[list[str], list[str], dict[str, float]]:
    """Return (velocity_joints, position_joints, effort_limits) declared in the
    URDF. Command interfaces come from any <ros2_control> block; the bridge
    graph reflects whatever the URDF says is a ros2_control command interface,
    regardless of which controller class claims it; this lets any controller
    type Just Work without per-type handling on the Isaac side. Effort limits
    come from the <limit> element of the plain <joint> definitions.
    """
    velocity: list[str] = []
    position: list[str] = []
    efforts: dict[str, float] = {}
    root = ET.parse(urdf_path).getroot()
    for joint in root.iter('joint'):
        joint_name = joint.get('name')
        limit = joint.find('limit')
        if joint_name and limit is not None and (effort := limit.get('effort')) is not None:
            efforts[joint_name] = float(effort)
    for rc in root.iter('ros2_control'):
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
    return velocity, position, efforts


class Control:
    def __init__(
        self,
        prim_path: str,
        target_prim_path: str,
        cmd_vel_topic: str,
        urdf_path: str,
        velocity_commands_topic: str | None = None,
        position_commands_topic: str | None = None,
        states_topic: str | None = None,
    ):
        self.prim_path: str = prim_path
        self.target_prim_path: str = target_prim_path
        self.cmd_vel_topic: str = cmd_vel_topic
        self.urdf_path: str = urdf_path

        ns = cmd_vel_topic.rsplit('/', 1)[0] if '/' in cmd_vel_topic else ''
        self.velocity_commands_topic: str = velocity_commands_topic if velocity_commands_topic is not None else f"{ns}/isaac/joint_commands_velocity"
        self.position_commands_topic: str = position_commands_topic if position_commands_topic is not None else f"{ns}/isaac/joint_commands_position"
        self.states_topic: str = states_topic if states_topic is not None else f"{ns}/isaac/joint_states"

    def parse(self, robot_model: str) -> bool:
        """Mirror every ros2_control joint declared in the URDF into the
        topic_bridge graph. All controllers run in the external CM; Isaac just
        exposes joint states/commands over per-kind JointState topics.
        """
        del robot_model  # control plane is URDF-driven; controller types are decided in the CM
        return self._dispatch_bridge()

    def _dispatch_bridge(self) -> bool:
        joints_velocity, joints_position, effort_limits = _joints_from_urdf(self.urdf_path)
        if not joints_velocity and not joints_position:
            carb.log_error(
                f"topic_bridge: no ros2_control joints found in URDF '{self.urdf_path}'"
            )
            return False

        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.prim_path)
        # The URDF importer nests joints under `<prim_path>/Physics/<name>`, so
        # index them by name from the subtree rather than a fixed path.
        joint_prims = {
            prim.GetName(): prim
            for prim in Usd.PrimRange(root)
            if prim.IsA(UsdPhysics.Joint)
        } if root.IsValid() else {}

        newton = physics_engine() == 'newton'
        velocity_damping = _VELOCITY_DRIVE_DAMPING_NEWTON if newton else _VELOCITY_DRIVE_DAMPING
        velocity_max_force = _VELOCITY_DRIVE_MAX_FORCE_NEWTON if newton else None

        for joint_name in joints_position:
            joint_prim = joint_prims.get(joint_name)
            if joint_prim is not None:
                if newton:
                    effort = effort_limits.get(joint_name, _POSITION_DRIVE_EFFORT_FALLBACK_NEWTON)
                    kp = _POSITION_DRIVE_KP_PER_EFFORT_NEWTON * effort / _RAD2DEG
                    _set_drive(joint_prim, kp, kp / _POSITION_DRIVE_KP_KD_RATIO_NEWTON, max_force=effort)
                else:
                    _set_drive(joint_prim, _POSITION_DRIVE_STIFFNESS, _POSITION_DRIVE_DAMPING)
            else:
                carb.log_warn(f"topic_bridge: joint prim not found for position drive: {joint_name}")
        for joint_name in joints_velocity:
            joint_prim = joint_prims.get(joint_name)
            if joint_prim is not None:
                _set_drive(joint_prim, 0.0, velocity_damping, max_force=velocity_max_force)
            else:
                carb.log_warn(f"topic_bridge: joint prim not found for velocity drive: {joint_name}")

        graph_path = os.path.join(self.prim_path, 'topic_bridge')
        register_rebuilder(graph_path, self._dispatch_bridge)
        return topic_bridge(
            graph_path=graph_path,
            prim_path=self.target_prim_path,
            joints_velocity=joints_velocity,
            joints_position=joints_position,
            velocity_commands_topic=self.velocity_commands_topic,
            position_commands_topic=self.position_commands_topic,
            states_topic=self.states_topic,
        )


__all__ = ['Control']
