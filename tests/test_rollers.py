from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from isaac_utils.utils.rollers import expand_roller_wheels, roller_layout

_RADIUS = 0.127


def _robot(mu: float = 1.0, mu2: float = 0.0, fdir1: str = '0.70710678 -0.70710678 0', wheel_yaw: float = 0.0, shape: str = f'<sphere radius="{_RADIUS}"/>') -> ET.Element:
    return ET.fromstring(
        '<robot name="r" xmlns:gz="http://gazebosim.org/schema">'
        '<link name="base_footprint"/>'
        '<link name="base_link"><inertial><mass value="80"/><inertia ixx="3" iyy="5" izz="6" ixy="0" ixz="0" iyz="0"/></inertial></link>'
        '<joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/></joint>'
        '<link name="wheel_link"><inertial><mass value="6"/><inertia ixx="0.03" iyy="0.05" izz="0.03" ixy="0" ixz="0" iyz="0"/></inertial>'
        f'<collision><geometry>{shape}</geometry></collision></link>'
        '<joint name="wheel_joint" type="continuous"><parent link="base_link"/><child link="wheel_link"/>'
        f'<origin xyz="0.2 0.25 0" rpy="0 0 {wheel_yaw}"/><axis xyz="{math.sin(wheel_yaw)} {math.cos(wheel_yaw)} 0"/></joint>'
        f'<gazebo reference="wheel_link"><collision><surface><friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2>'
        f'<fdir1 gz:expressed_in="base_footprint">{fdir1}</fdir1></ode></friction></surface></collision></gazebo>'
        '</robot>'
    )


def _floats(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()])


def _rollers(root: ET.Element) -> list[tuple[np.ndarray, np.ndarray, ET.Element]]:
    links = {link.attrib['name']: link for link in root.findall('link')}
    return [(_floats(joint.find('origin').attrib['xyz']), _floats(joint.find('axis').attrib['xyz']), links[joint.find('child').attrib['link']]) for joint in root.findall('joint') if '_roller_' in joint.attrib['name']]


def test_every_roller_sphere_touches_the_wheel_circle_from_inside():
    root = _robot()
    assert expand_roller_wheels(root) == [f'wheel_link_roller_{index}_joint' for index in range(12)]
    rollers = _rollers(root)
    assert len(rollers) == 12
    for origin, axis, link in rollers:
        assert np.linalg.norm(axis) == pytest.approx(1.0)
        for collision in link.findall('collision'):
            center = origin + _floats(collision.find('origin').attrib['xyz'])
            reach = math.hypot(center[0], center[2]) + float(collision.find('geometry/sphere').attrib['radius'])
            assert reach == pytest.approx(_RADIUS, abs=1e-6)
            assert np.linalg.norm(np.cross(center - origin, axis)) == pytest.approx(0.0, abs=1e-9)


def test_wheel_contact_points_are_spread_evenly_around_the_rim():
    root = _robot()
    expand_roller_wheels(root)
    angles = sorted(math.atan2(center[0], -center[2]) % (2 * math.pi) for origin, _, link in _rollers(root) for center in [origin + _floats(c.find('origin').attrib['xyz']) for c in link.findall('collision')])
    assert np.diff(angles) == pytest.approx(2 * math.pi / len(angles), abs=1e-6)


def test_bottom_roller_spins_about_the_grip_direction():
    root = _robot()
    expand_roller_wheels(root)
    origin, axis, _ = min(_rollers(root), key=lambda roller: roller[0][2])
    assert origin[2] < 0.0 and abs(origin[0]) < 1e-9
    assert axis[2] == pytest.approx(0.0, abs=1e-9)
    assert math.degrees(math.atan2(-axis[1], axis[0])) == pytest.approx(45.0, abs=1.0)


def test_grip_across_fdir1_spins_the_bottom_roller_about_the_other_diagonal():
    root = _robot(mu=0.0, mu2=1.0)
    expand_roller_wheels(root)
    _, axis, _ = min(_rollers(root), key=lambda roller: roller[0][2])
    assert math.degrees(math.atan2(axis[1], axis[0])) % 180.0 == pytest.approx(45.0, abs=1.0)


def test_grip_direction_is_read_in_the_frame_it_is_expressed_in():
    root = _robot(wheel_yaw=math.pi / 2)
    expand_roller_wheels(root)
    _, axis, _ = min(_rollers(root), key=lambda roller: roller[0][2])
    in_base = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]) @ axis
    assert math.degrees(math.atan2(-in_base[1], in_base[0])) % 180.0 == pytest.approx(45.0, abs=1.0)


def test_rollers_take_over_collision_grip_and_a_share_of_the_mass():
    root = _robot()
    expand_roller_wheels(root)
    wheel = next(link for link in root.findall('link') if link.attrib['name'] == 'wheel_link')
    assert wheel.find('collision') is None
    masses = [float(link.find('inertial/mass').attrib['value']) for _, _, link in _rollers(root)]
    assert float(wheel.find('inertial/mass').attrib['value']) + sum(masses) == pytest.approx(6.0)
    grips = {gazebo.attrib['reference']: (gazebo.findtext('mu1'), gazebo.findtext('mu2')) for gazebo in root.findall('gazebo')}
    assert set(grips) == {link.attrib['name'] for _, _, link in _rollers(root)}
    assert set(grips.values()) == {('1', '1')}


@pytest.mark.parametrize(
    'robot',
    [
        _robot(mu=1.0, mu2=1.0),
        _robot(mu=0.0, mu2=0.0),
        _robot(fdir1='0 0 0'),
        _robot(fdir1='0 1 0'),
        _robot(shape='<box size="0.2 0.1 0.2"/>'),
    ],
    ids=['equal-grip', 'no-grip', 'no-direction', 'grip-along-the-axle', 'box-wheel'],
)
def test_wheels_that_rollers_cannot_stand_in_for_stay_untouched(robot: ET.Element):
    before = ET.tostring(robot)
    assert expand_roller_wheels(robot) == []
    assert ET.tostring(robot) == before


def test_roller_axis_is_corrected_for_its_tilt_while_engaged():
    layout = roller_layout(_RADIUS, np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0]), np.array([1.0, -1.0, 0.0]) / math.sqrt(2.0))
    _, axis, _ = layout[0]
    engaged = math.pi / len(layout)
    assert abs(axis[1] / axis[0]) == pytest.approx(math.sin(engaged) / engaged)
