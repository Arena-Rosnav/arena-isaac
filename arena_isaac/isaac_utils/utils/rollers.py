"""Passive rollers for URDF wheels that declare grip along one ground direction only."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

_GZ_EXPRESSED_IN = '{http://gazebosim.org/schema}expressed_in'

_ROLLERS = 12
_SPHERES = 2
_CENTER_RADIUS_SHARE = 0.28
_MASS_SHARE = 0.25
_INERTIA_ATTRS = ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')


def _rpy_matrix(rpy: str) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return np.array(
        [
            [c + x * x * k, x * y * k - z * s, x * z * k + y * s],
            [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
            [z * x * k - y * s, z * y * k + x * s, c + z * z * k],
        ]
    )


def _link_rotations(root: ET.Element) -> dict[str, np.ndarray]:
    """Rotation of every link frame against the root link with all joints at zero."""
    parents: dict[str, tuple[str, np.ndarray]] = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or child is None:
            continue
        origin = joint.find('origin')
        rpy = '0 0 0' if origin is None else origin.attrib.get('rpy', '0 0 0')
        parents[child.attrib['link']] = (parent.attrib['link'], _rpy_matrix(rpy))

    rotations: dict[str, np.ndarray] = {}

    def resolve(link: str) -> np.ndarray:
        if link not in rotations:
            if link in parents:
                parent, local = parents[link]
                rotations[link] = resolve(parent) @ local
            else:
                rotations[link] = np.eye(3)
        return rotations[link]

    for link in root.findall('link'):
        resolve(link.attrib['name'])
    return rotations


def _vector(element: ET.Element) -> np.ndarray | None:
    raw = element.attrib.get('value') or element.text or ''
    try:
        values = [float(v) for v in raw.split()]
    except ValueError:
        return None
    if len(values) != 3 or not any(values):
        return None
    vector = np.array(values)
    return vector / np.linalg.norm(vector)


def _coefficient(gazebo: ET.Element, *tags: str) -> float | None:
    for tag in tags:
        for element in gazebo.iter(tag):
            raw = element.attrib.get('value') or element.text or ''
            try:
                return float(raw)
            except ValueError:
                continue
    return None


def roller_layout(
    radius: float,
    axle: np.ndarray,
    down: np.ndarray,
    grip: np.ndarray,
) -> list[tuple[np.ndarray, np.ndarray, list[tuple[float, float]]]]:
    """Per roller its hinge origin, hinge axis and (offset along the axis, radius) of each sphere, all touching the wheel circle."""
    axle = axle / np.linalg.norm(axle)
    down = down - (down @ axle) * axle
    down = down / np.linalg.norm(down)
    forward = np.cross(axle, down)
    engaged = math.pi / _ROLLERS
    along_axle = float(grip @ axle)
    along_forward = float(grip @ forward) * engaged / math.sin(engaged)
    norm = math.hypot(along_axle, along_forward)
    along_axle /= norm
    along_forward /= norm
    bottom_axis = along_axle * axle + along_forward * forward
    pitch = radius * (1.0 - _CENTER_RADIUS_SHARE)
    step = 2.0 * math.pi / (_ROLLERS * _SPHERES)
    spheres: list[tuple[float, float]] = []
    for index in range(_SPHERES):
        angle = (index - (_SPHERES - 1) / 2.0) * step
        offset = pitch * math.tan(angle) / along_forward
        spheres.append((offset, radius - pitch / math.cos(angle)))
    layout = []
    for index in range(_ROLLERS):
        turn = _axis_rotation(axle, 2.0 * math.pi * index / _ROLLERS)
        layout.append((turn @ (pitch * down), turn @ bottom_axis, spheres))
    return layout


def _wheel_radius(link: ET.Element) -> float | None:
    radii: list[float] = []
    for collision in link.findall('collision'):
        shape = collision.find('geometry/sphere')
        if shape is None:
            shape = collision.find('geometry/cylinder')
        if shape is None:
            return None
        radii.append(float(shape.attrib['radius']))
    return max(radii) if radii else None


def _triple(vector: np.ndarray) -> str:
    return ' '.join(f'{value:.9g}' for value in vector)


def _lighten(link: ET.Element) -> float:
    """Take the rollers' mass share out of the wheel link, return the mass of one roller."""
    mass = link.find('inertial/mass')
    inertia = link.find('inertial/inertia')
    if mass is None or inertia is None:
        return 0.0
    total = float(mass.attrib['value'])
    mass.attrib['value'] = f'{total * (1.0 - _MASS_SHARE):.9g}'
    for attr in _INERTIA_ATTRS:
        if attr in inertia.attrib:
            inertia.attrib[attr] = f'{float(inertia.attrib[attr]) * (1.0 - _MASS_SHARE):.9g}'
    return total * _MASS_SHARE / _ROLLERS


def expand_roller_wheels(root: ET.Element) -> list[str]:
    """Rebuild every link whose <gazebo> friction grips along one direction only as a wheel with free-spinning rollers, return the roller joints."""
    rotations = _link_rotations(root)
    links = {link.attrib['name']: link for link in root.findall('link')}
    joints = {child.attrib['link']: joint for joint in root.findall('joint') if (child := joint.find('child')) is not None}
    hinges: list[str] = []
    for gazebo in root.findall('gazebo'):
        name = gazebo.attrib.get('reference')
        direction = next(gazebo.iter('fdir1'), None)
        if name not in links or name not in joints or direction is None:
            continue
        along = _vector(direction)
        mu = _coefficient(gazebo, 'mu', 'mu1')
        mu2 = _coefficient(gazebo, 'mu2')
        frame = direction.attrib.get(_GZ_EXPRESSED_IN, name)
        if along is None or mu is None or mu2 is None or (mu > 0.0) == (mu2 > 0.0) or frame not in rotations:
            continue
        link = links[name]
        joint = joints[name]
        radius = _wheel_radius(link)
        axis = joint.find('axis')
        axle = np.array([float(v) for v in (axis.attrib['xyz'] if axis is not None else '1 0 0').split()])
        axle /= np.linalg.norm(axle)
        to_link = rotations[name].T @ rotations[frame]
        down = to_link @ np.array([0.0, 0.0, -1.0])
        grip = to_link @ along
        if mu2 > 0.0:
            grip = np.cross(down, grip)
        if radius is None or joint.attrib.get('type') not in ('continuous', 'revolute') or abs(float(grip @ np.cross(axle, down))) < 1e-3:
            continue

        for collision in link.findall('collision'):
            link.remove(collision)
        roller_mass = _lighten(link)
        root.remove(gazebo)
        friction = max(mu, mu2)
        for index, (origin, roller_axis, spheres) in enumerate(roller_layout(radius, axle, down, grip)):
            roller_name = f'{name}_roller_{index}'
            roller = ET.SubElement(root, 'link', {'name': roller_name})
            if roller_mass > 0.0:
                inertial = ET.SubElement(roller, 'inertial')
                ET.SubElement(inertial, 'mass', {'value': f'{roller_mass:.9g}'})
                moment = 0.4 * roller_mass * max(sphere_radius for _, sphere_radius in spheres) ** 2
                ET.SubElement(inertial, 'inertia', {attr: f'{moment:.9g}' if attr in ('ixx', 'iyy', 'izz') else '0' for attr in _INERTIA_ATTRS})
            for offset, sphere_radius in spheres:
                collision = ET.SubElement(roller, 'collision')
                ET.SubElement(collision, 'origin', {'xyz': _triple(offset * roller_axis), 'rpy': '0 0 0'})
                ET.SubElement(ET.SubElement(collision, 'geometry'), 'sphere', {'radius': f'{sphere_radius:.9g}'})
            hinges.append(f'{roller_name}_joint')
            hinge = ET.SubElement(root, 'joint', {'name': hinges[-1], 'type': 'continuous'})
            ET.SubElement(hinge, 'parent', {'link': name})
            ET.SubElement(hinge, 'child', {'link': roller_name})
            ET.SubElement(hinge, 'origin', {'xyz': _triple(origin), 'rpy': '0 0 0'})
            ET.SubElement(hinge, 'axis', {'xyz': _triple(roller_axis)})
            grip_tags = ET.SubElement(root, 'gazebo', {'reference': roller_name})
            ET.SubElement(grip_tags, 'mu1').text = f'{friction:.9g}'
            ET.SubElement(grip_tags, 'mu2').text = f'{friction:.9g}'
    return hinges


__all__ = ['expand_roller_wheels', 'roller_layout']
