from __future__ import annotations

import xml.etree.ElementTree as ET

from isaac_utils.utils.inertials import prune_inertials

_ZERO = '<inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial>'
_TINY = '<inertial><mass value="1e-09"/><inertia ixx="3.6e-08" iyy="3.6e-08" izz="6e-08" ixy="0" ixz="0" iyz="0"/></inertial>'
_WHEEL = '<inertial><mass value="0.3"/><inertia ixx="2e-04" iyy="2e-04" izz="3e-04" ixy="0" ixz="0" iyz="0"/></inertial>'


def _robot(root_inertial: str) -> ET.Element:
    wheels = ''.join(f'<link name="wheel_{i}">{_WHEEL}<collision><geometry><cylinder radius="0.03" length="0.01"/></geometry></collision></link><joint name="wheel{i}_speed" type="continuous"><parent link="base"/><child link="wheel_{i}"/><axis xyz="1 0 0"/></joint>' for i in range(3))
    return ET.fromstring(
        f'<robot name="r"><link name="base">{root_inertial}</link>{wheels}<link name="marker">{_ZERO}</link><link name="pin">{_TINY}</link><joint name="pin_fixed" type="fixed"><parent link="base"/><child link="pin"/></joint><joint name="marker_fixed" type="fixed"><parent link="base"/><child link="marker"/></joint></robot>'
    )


def _mass(root: ET.Element, link: str) -> float | None:
    mass = root.find(f"link[@name='{link}']/inertial/mass")
    return None if mass is None else float(mass.attrib['value'])


def test_a_root_with_a_zero_inertial_keeps_a_valid_one_and_stays_a_body():
    root = _robot(_ZERO)
    prune_inertials(root)
    assert _mass(root, 'base') == 0.01
    assert [float(root.find("link[@name='base']/inertial/inertia").attrib[key]) for key in ('ixx', 'iyy', 'izz')] == [1e-4, 1e-4, 1e-4]


def test_a_zero_child_inertial_is_dropped_and_a_small_valid_one_kept():
    root = _robot(_ZERO)
    prune_inertials(root)
    assert _mass(root, 'marker') is None
    assert _mass(root, 'wheel_0') == 0.3
    assert _mass(root, 'pin') == 1e-09


def test_a_root_without_an_inertial_stays_without_one():
    root = _robot('')
    prune_inertials(root)
    assert _mass(root, 'base') is None
