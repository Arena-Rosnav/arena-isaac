from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from isaac_utils.utils.drives import carried_mass, command_interfaces, position_drive, position_drives, strip_dynamics_damping

_URDF = """
<robot name="lifter">
  <link name="base"><inertial><mass value="50"/></inertial></link>
  <link name="column"><inertial><mass value="10"/></inertial></link>
  <link name="arm"><inertial><mass value="5"/></inertial></link>
  <link name="hand"><inertial><mass value="2"/></inertial></link>
  <link name="wheel"><inertial><mass value="1"/></inertial></link>
  <link name="marker"/>
  <joint name="lift" type="prismatic">
    <parent link="base"/><child link="column"/><axis xyz="0 0 1"/>
    <limit effort="2000" lower="0" upper="0.35" velocity="0.07"/>
    <dynamics damping="1000" friction="1.0"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="column"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit effort="43" lower="0" upper="2.7" velocity="1.95"/>
    <dynamics damping="40.0"/>
  </joint>
  <joint name="wrist" type="continuous">
    <parent link="arm"/><child link="hand"/><axis xyz="0 1 0"/>
    <limit effort="6.6" velocity="3.0"/>
  </joint>
  <joint name="wheel_joint" type="continuous">
    <parent link="base"/><child link="wheel"/><axis xyz="0 1 0"/>
    <limit effort="10" velocity="20"/>
    <dynamics damping="0.5"/>
  </joint>
  <joint name="marker_joint" type="fixed"><parent link="base"/><child link="marker"/></joint>
  <ros2_control name="system" type="system">
    <joint name="wheel_joint"><command_interface name="velocity"/><state_interface name="position"/></joint>
    <joint name="lift"><command_interface name="position"/><state_interface name="position"/></joint>
    <joint name="shoulder"><command_interface name="position"/></joint>
    <joint name="wrist"><command_interface name="position"/></joint>
    <joint name="marker_joint"><command_interface name="position"/></joint>
  </ros2_control>
</robot>
"""


def _root() -> ET.Element:
    return ET.fromstring(_URDF)


def test_command_interfaces_split_velocity_from_position_joints():
    velocity, position = command_interfaces(_root())
    assert velocity == ['wheel_joint']
    assert position == ['lift', 'shoulder', 'wrist', 'marker_joint']


def test_carried_mass_sums_the_subtree_below_the_joint():
    root = _root()
    assert carried_mass(root, 'lift') == 17.0
    assert carried_mass(root, 'shoulder') == 7.0
    assert carried_mass(root, 'wrist') == 2.0
    assert carried_mass(root, 'missing') == 0.0


def test_only_position_commanded_movable_joints_get_a_drive():
    assert set(position_drives(_root())) == {'lift', 'shoulder', 'wrist'}


def test_the_lift_holds_its_load_within_the_sag_budget_and_effort():
    drive = position_drive(_root(), 'lift')
    assert not drive.angular
    assert drive.max_force == 2000.0
    assert drive.stiffness * 0.002 >= 17.0 * 9.81 - 1e-6
    assert drive.stiffness * 0.002 < drive.max_force


def test_the_lift_is_damped_critically_without_exceeding_effort_at_max_speed():
    drive = position_drive(_root(), 'lift')
    assert drive.damping == pytest.approx(2.0 * math.sqrt(drive.stiffness * 17.0))
    assert drive.damping * 0.07 <= drive.max_force


def test_a_revolute_joint_scales_with_its_effort():
    shoulder, wrist = position_drive(_root(), 'shoulder'), position_drive(_root(), 'wrist')
    assert shoulder.angular and wrist.angular
    assert shoulder.stiffness / wrist.stiffness == pytest.approx(43.0 / 6.6)
    assert shoulder.max_force == 43.0
    assert shoulder.damping * 1.95 <= shoulder.max_force + 1e-9


def test_a_joint_without_a_limit_falls_back_to_a_default_effort():
    root = ET.fromstring('<robot name="r"><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint></robot>')
    assert position_drive(root, 'j').max_force == 25.0


def test_dynamics_damping_is_stripped_from_the_named_joints_only():
    root = _root()
    strip_dynamics_damping(root, ['lift', 'shoulder'])
    assert 'damping' not in root.find("joint[@name='lift']/dynamics").attrib
    assert root.find("joint[@name='lift']/dynamics").attrib['friction'] == '1.0'
    assert 'damping' not in root.find("joint[@name='shoulder']/dynamics").attrib
    assert root.find("joint[@name='wheel_joint']/dynamics").attrib['damping'] == '0.5'
