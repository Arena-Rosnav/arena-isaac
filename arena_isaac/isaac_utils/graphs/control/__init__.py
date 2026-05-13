import os
import xml.etree.ElementTree as ET

import carb
import omni.usd
from pxr import UsdPhysics

from .topic_bridge import topic_bridge

# Drive defaults. URDF import leaves stiffness/damping at 0, so position drives
# can't hold setpoints (gravity wins) and velocity drives can't apply force.
# These values let the articulation actually track commands fed via TopicBasedSystem.
_POSITION_DRIVE_STIFFNESS = 4.0e5
_POSITION_DRIVE_DAMPING = 4.0e4
_VELOCITY_DRIVE_DAMPING = 1.0e4


def _set_drive(joint_prim, stiffness: float, damping: float) -> None:
    for axis in ('angular', 'linear'):
        drive = UsdPhysics.DriveAPI(joint_prim, axis)
        stiffness_attr = drive.GetStiffnessAttr()
        if stiffness_attr.IsValid():
            stiffness_attr.Set(stiffness)
            drive.GetDampingAttr().Set(damping)


def _joints_from_urdf(urdf_path: str) -> tuple[list[str], list[str]]:
    """Return (velocity_joints, position_joints) declared in any <ros2_control>
    block of the URDF. The bridge graph reflects whatever the URDF says is a
    ros2_control command interface, regardless of which controller class
    claims it; this lets any controller type Just Work without per-type
    handling on the Isaac side.
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
        joints_velocity, joints_position = _joints_from_urdf(self.urdf_path)
        if not joints_velocity and not joints_position:
            carb.log_error(
                f"topic_bridge: no ros2_control joints found in URDF '{self.urdf_path}'"
            )
            return False

        stage = omni.usd.get_context().get_stage()
        for joint_name in joints_position:
            path = f"{self.prim_path}/joints/{joint_name}"
            joint_prim = stage.GetPrimAtPath(path)
            if joint_prim.IsValid():
                _set_drive(joint_prim, _POSITION_DRIVE_STIFFNESS, _POSITION_DRIVE_DAMPING)
            else:
                carb.log_warn(f"topic_bridge: joint prim not found for position drive: {path}")
        for joint_name in joints_velocity:
            path = f"{self.prim_path}/joints/{joint_name}"
            joint_prim = stage.GetPrimAtPath(path)
            if joint_prim.IsValid():
                _set_drive(joint_prim, 0.0, _VELOCITY_DRIVE_DAMPING)
            else:
                carb.log_warn(f"topic_bridge: joint prim not found for velocity drive: {path}")

        return topic_bridge(
            graph_path=os.path.join(self.prim_path, 'topic_bridge'),
            prim_path=self.target_prim_path,
            joints_velocity=joints_velocity,
            joints_position=joints_position,
            velocity_commands_topic=self.velocity_commands_topic,
            position_commands_topic=self.position_commands_topic,
            states_topic=self.states_topic,
        )


__all__ = ['Control']
