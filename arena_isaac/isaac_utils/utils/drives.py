"""Per-joint drive gains for ros2_control joints, sized from URDF limits."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

GRAVITY = 9.81
STIFFNESS_PER_EFFORT_REVOLUTE = 100.0
STIFFNESS_PER_EFFORT_PRISMATIC = 8.0
MAX_GRAVITY_SAG = 0.002
DAMPING_RATIO = 1.0
STIFFNESS_DAMPING_RATIO = 20.0
EFFORT_FALLBACK = 25.0


@dataclass(frozen=True)
class PositionDrive:
    """SI gains: N m/rad and N m s/rad for revolute joints, N/m and N s/m for prismatic."""

    stiffness: float
    damping: float
    max_force: float
    angular: bool


def command_interfaces(root: ET.Element) -> tuple[list[str], list[str]]:
    """Joint names with a velocity command interface and with a position one, in URDF order."""
    velocity: list[str] = []
    position: list[str] = []
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
    return velocity, position


def _link_mass(link: ET.Element) -> float:
    mass = link.find('inertial/mass')
    return float(mass.get('value', 0.0)) if mass is not None else 0.0


def carried_mass(root: ET.Element, joint_name: str) -> float:
    """Total mass of the child link of a joint and of every link below it."""
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}
    children: dict[str, list[str]] = {}
    for joint in joints.values():
        parent, child = joint.find('parent'), joint.find('child')
        if parent is not None and child is not None:
            children.setdefault(parent.get('link', ''), []).append(child.get('link', ''))
    joint = joints.get(joint_name)
    child = joint.find('child') if joint is not None else None
    if child is None:
        return 0.0
    total = 0.0
    stack = [child.get('link', '')]
    seen: set[str] = set()
    while stack:
        name = stack.pop()
        if name in seen:
            continue
        seen.add(name)
        if name in links:
            total += _link_mass(links[name])
        stack.extend(children.get(name, []))
    return total


def position_drive(root: ET.Element, joint_name: str) -> PositionDrive | None:
    """Drive for a position-commanded joint: stiff enough to hold its carried load, damped to critical, never above its effort limit."""
    joint = next((j for j in root.findall('joint') if j.get('name') == joint_name), None)
    if joint is None:
        return None
    kind = joint.get('type')
    if kind not in ('revolute', 'continuous', 'prismatic'):
        return None
    angular = kind != 'prismatic'
    limit = joint.find('limit')
    effort = float(limit.get('effort')) if limit is not None and limit.get('effort') else EFFORT_FALLBACK
    velocity = float(limit.get('velocity', 0.0)) if limit is not None else 0.0
    mass = carried_mass(root, joint_name)

    if angular:
        stiffness = STIFFNESS_PER_EFFORT_REVOLUTE * effort
        damping = stiffness / STIFFNESS_DAMPING_RATIO
    else:
        stiffness = STIFFNESS_PER_EFFORT_PRISMATIC * effort
        if mass > 0.0:
            stiffness = max(stiffness, mass * GRAVITY / MAX_GRAVITY_SAG)
        damping = 2.0 * DAMPING_RATIO * math.sqrt(stiffness * mass) if mass > 0.0 else stiffness / STIFFNESS_DAMPING_RATIO
    if velocity > 0.0:
        damping = min(damping, effort / velocity)
    return PositionDrive(stiffness=stiffness, damping=damping, max_force=effort, angular=angular)


def position_drives(root: ET.Element) -> dict[str, PositionDrive]:
    """Drive per joint that ros2_control commands by position."""
    _, position = command_interfaces(root)
    return {name: drive for name in position if (drive := position_drive(root, name)) is not None}


def strip_dynamics_damping(root: ET.Element, joint_names: list[str]) -> None:
    """Remove `<dynamics damping>` from the named joints so it cannot resist the drive."""
    wanted = set(joint_names)
    for joint in root.findall('joint'):
        dynamics = joint.find('dynamics')
        if joint.get('name') in wanted and dynamics is not None:
            dynamics.attrib.pop('damping', None)


__all__ = ['PositionDrive', 'carried_mass', 'command_interfaces', 'position_drive', 'position_drives', 'strip_dynamics_damping']
