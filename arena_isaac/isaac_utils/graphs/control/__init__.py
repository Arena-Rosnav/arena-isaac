import math
import os
import xml.etree.ElementTree as ET

import carb
import omni.usd
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

from isaac_utils.graphs import physics_engine, register_rebuilder
from isaac_utils.utils.drives import JointDrive, command_interfaces, joint_drives
from isaac_utils.utils.initial_joints import initial_positions

from . import release
from .topic_bridge import topic_bridge

_RAD2DEG = 180.0 / math.pi

# Drive defaults. URDF import leaves stiffness/damping at 0, so position drives
# can't hold setpoints (gravity wins) and velocity drives can't apply force.
# These values let the articulation actually track commands fed via TopicBasedSystem.
_POSITION_DRIVE_STIFFNESS = 4.0e5
_POSITION_DRIVE_DAMPING = 4.0e3
_VELOCITY_DRIVE_DAMPING = 1.0e4
# mjwarp applies velocity-actuator feedback per step, the physx-scale gain
# (5.7e5 N m s/rad after deg conversion) explodes wheel DOFs to NaN within
# frames of ground contact, and an unclamped drive at any gain delivers the
# full velocity-error torque at once (hundreds of N m, backflips the base)
_VELOCITY_DRIVE_DAMPING_NEWTON = 0.35
_VELOCITY_DRIVE_MAX_FORCE_NEWTON = 16.0
_POSITION_DRIVE_GAIN_SCALE_NEWTON = 0.08
_POSITION_DRIVE_ARMATURE_NEWTON = 0.01


def _set_drive(joint_prim: Usd.Prim, stiffness: float, damping: float, max_force: float | None = None) -> None:
    for axis in ('angular', 'linear'):
        drive = UsdPhysics.DriveAPI(joint_prim, axis)
        stiffness_attr = drive.GetStiffnessAttr()
        if stiffness_attr.IsValid():
            stiffness_attr.Set(stiffness)
            drive.GetDampingAttr().Set(damping)
            if max_force is not None:
                drive.CreateMaxForceAttr(max_force)


def _set_initial(joint_prim: Usd.Prim, position: float) -> None:
    axis, value = ('angular', math.degrees(position)) if joint_prim.IsA(UsdPhysics.RevoluteJoint) else ('linear', position)
    UsdPhysics.DriveAPI(joint_prim, axis).CreateTargetPositionAttr(value)
    PhysxSchema.JointStateAPI.Apply(joint_prim, axis).CreatePositionAttr(value)


def _joints_from_urdf(urdf_path: str) -> tuple[list[str], list[str], list[str], dict[str, JointDrive]]:
    """Return (velocity_joints, position_joints, effort_joints, drives) declared by the URDF's ros2_control block."""
    root = ET.parse(urdf_path).getroot()
    velocity, position, effort = command_interfaces(root)
    return velocity, position, effort, joint_drives(root)


class Control:
    def __init__(
        self,
        prim_path: str,
        target_prim_path: str,
        cmd_vel_topic: str,
        urdf_path: str,
        velocity_commands_topic: str | None = None,
        position_commands_topic: str | None = None,
        effort_commands_topic: str | None = None,
        states_topic: str | None = None,
    ):
        self.prim_path: str = prim_path
        self.target_prim_path: str = target_prim_path
        self.cmd_vel_topic: str = cmd_vel_topic
        self.urdf_path: str = urdf_path

        ns = cmd_vel_topic.rsplit('/', 1)[0] if '/' in cmd_vel_topic else ''
        self.velocity_commands_topic: str = velocity_commands_topic if velocity_commands_topic is not None else f"{ns}/isaac/joint_commands_velocity"
        self.position_commands_topic: str = position_commands_topic if position_commands_topic is not None else f"{ns}/isaac/joint_commands_position"
        self.effort_commands_topic: str = effort_commands_topic if effort_commands_topic is not None else f"{ns}/isaac/joint_commands_effort"
        self.states_topic: str = states_topic if states_topic is not None else f"{ns}/isaac/joint_states"

    def parse(self, robot_model: str) -> bool:
        """Mirror every ros2_control joint declared in the URDF into the
        topic_bridge graph. All controllers run in the external CM; Isaac just
        exposes joint states/commands over per-kind JointState topics.
        """
        del robot_model  # control plane is URDF-driven; controller types are decided in the CM
        joint_prims = self._joint_prims()
        for joint_name, position in initial_positions(self.urdf_path).items():
            if (joint_prim := joint_prims.get(joint_name)) is not None:
                _set_initial(joint_prim, position)
        return self._dispatch_bridge()

    def _joint_prims(self) -> dict[str, Usd.Prim]:
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.prim_path)
        # The URDF importer nests joints under `<prim_path>/Physics/<name>`, so
        # index them by name from the subtree rather than a fixed path.
        return {prim.GetName(): prim for prim in Usd.PrimRange(root) if prim.IsA(UsdPhysics.Joint)} if root.IsValid() else {}

    def _dispatch_bridge(self) -> bool:
        joints_velocity, joints_position, joints_effort, drives = _joints_from_urdf(self.urdf_path)
        if not joints_velocity and not joints_position and not joints_effort:
            carb.log_error(f"topic_bridge: no ros2_control joints found in URDF '{self.urdf_path}'")
            return False

        joint_prims = self._joint_prims()

        newton = physics_engine() == 'newton'
        velocity_damping = _VELOCITY_DRIVE_DAMPING_NEWTON if newton else _VELOCITY_DRIVE_DAMPING
        velocity_max_force = _VELOCITY_DRIVE_MAX_FORCE_NEWTON if newton else None

        for joint_name in joints_position:
            joint_prim = joint_prims.get(joint_name)
            if joint_prim is not None:
                drive = drives.get(joint_name)
                if drive is None:
                    _set_drive(joint_prim, _POSITION_DRIVE_STIFFNESS, _POSITION_DRIVE_DAMPING)
                elif newton and drive.angular:
                    scale = _POSITION_DRIVE_GAIN_SCALE_NEWTON / _RAD2DEG
                    _set_drive(joint_prim, drive.stiffness * scale, drive.damping * scale, max_force=drive.max_force)
                    joint_prim.CreateAttribute('newton:armature', Sdf.ValueTypeNames.Float).Set(_POSITION_DRIVE_ARMATURE_NEWTON)
                else:
                    scale = 1.0 / _RAD2DEG if drive.angular else 1.0
                    _set_drive(joint_prim, drive.stiffness * scale, drive.damping * scale, max_force=drive.max_force)
                if drive is not None:
                    carb.log_info(f"topic_bridge: position drive {joint_name} kp={drive.stiffness:.4g} kd={drive.damping:.4g} max_force={drive.max_force:.4g} ({'angular' if drive.angular else 'linear'})")
            else:
                carb.log_warn(f"topic_bridge: joint prim not found for position drive: {joint_name}")
        for joint_name in joints_velocity:
            joint_prim = joint_prims.get(joint_name)
            if joint_prim is not None:
                _set_drive(joint_prim, 0.0, velocity_damping, max_force=velocity_max_force)
            else:
                carb.log_warn(f"topic_bridge: joint prim not found for velocity drive: {joint_name}")
        held = not release.released(self.target_prim_path)
        for joint_name in joints_effort:
            joint_prim = joint_prims.get(joint_name)
            drive = drives.get(joint_name)
            if joint_prim is not None and drive is not None:
                scale = 1.0 / _RAD2DEG if drive.angular and held else 1.0
                _set_drive(joint_prim, drive.stiffness * scale if held else 0.0, drive.damping * scale if held else 0.0, max_force=drive.max_force)
            else:
                carb.log_warn(f"topic_bridge: joint prim or effort params not found for effort drive: {joint_name}")
        if joints_effort and held:
            release.hold(self.effort_commands_topic, self.target_prim_path, joints_effort)

        graph_path = os.path.join(self.prim_path, 'topic_bridge')
        register_rebuilder(graph_path, self._dispatch_bridge)
        return topic_bridge(
            graph_path=graph_path,
            prim_path=self.target_prim_path,
            joints_velocity=joints_velocity,
            joints_position=joints_position,
            joints_effort=joints_effort,
            velocity_commands_topic=self.velocity_commands_topic,
            position_commands_topic=self.position_commands_topic,
            effort_commands_topic=self.effort_commands_topic,
            states_topic=self.states_topic,
        )


__all__ = ['Control']
