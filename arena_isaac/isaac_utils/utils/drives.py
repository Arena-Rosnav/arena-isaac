"""Per-joint drive gains and effort limits read from the URDF's ros2_control joint params."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass(frozen=True)
class JointDrive:
    """SI gains: N m/rad and N m s/rad for angular joints, N/m and N s/m for prismatic."""

    stiffness: float
    damping: float
    max_force: float
    angular: bool
    kind: str


def command_interfaces(root: ET.Element) -> tuple[list[str], list[str], list[str]]:
    """Joint names with a velocity, a position and an effort command interface, in URDF order."""
    velocity: list[str] = []
    position: list[str] = []
    effort: list[str] = []
    for rc in root.iter('ros2_control'):
        for joint in rc.iter('joint'):
            name = joint.get('name')
            if not name:
                continue
            for cmd in joint.iter('command_interface'):
                kind = cmd.get('name')
                if kind == 'velocity':
                    velocity.append(name)
                elif kind == 'position':
                    position.append(name)
                elif kind == 'effort':
                    effort.append(name)
    return velocity, position, effort


def joint_drives(root: ET.Element) -> dict[str, JointDrive]:
    """Drive of every movable joint that ros2_control commands by position or effort and that carries kp, kd and effort params."""
    types = {j.get('name'): j.get('type') for j in root.findall('joint')}
    drives: dict[str, JointDrive] = {}
    for rc in root.iter('ros2_control'):
        for joint in rc.findall('joint'):
            name = joint.get('name')
            interfaces = {cmd.get('name') for cmd in joint.findall('command_interface')}
            kind = 'effort' if 'effort' in interfaces else 'position' if 'position' in interfaces else None
            params = {p.get('name'): (p.text or '').strip() for p in joint.findall('param')}
            if kind is None or types.get(name) not in ('revolute', 'continuous', 'prismatic') or not {'kp', 'kd', 'effort'} <= params.keys():
                continue
            drives[name] = JointDrive(float(params['kp']), float(params['kd']), float(params['effort']), types[name] != 'prismatic', kind)
    return drives


def strip_dynamics_damping(root: ET.Element, joint_names: list[str]) -> None:
    """Remove `<dynamics damping>` from the named joints so it cannot resist the drive."""
    wanted = set(joint_names)
    for joint in root.findall('joint'):
        dynamics = joint.find('dynamics')
        if joint.get('name') in wanted and dynamics is not None:
            dynamics.attrib.pop('damping', None)


__all__ = ['JointDrive', 'command_interfaces', 'joint_drives', 'strip_dynamics_damping']
