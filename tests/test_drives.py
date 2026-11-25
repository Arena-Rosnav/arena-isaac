from __future__ import annotations

import xml.etree.ElementTree as ET

from isaac_utils.utils.drives import command_interfaces, joint_drives, strip_dynamics_damping

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
    <joint name="lift"><command_interface name="position"/><state_interface name="position"/><param name="kp">83385.0</param><param name="kd">2381.2</param><param name="effort">2000.0</param></joint>
    <joint name="shoulder"><command_interface name="position"/><param name="kp">4300.0</param><param name="kd">215.0</param><param name="effort">43.0</param></joint>
    <joint name="wrist"><command_interface name="effort"/><param name="kp">660.0</param><param name="kd">33.0</param><param name="effort">6.6</param></joint>
    <joint name="marker_joint"><command_interface name="position"/><param name="kp">1.0</param><param name="kd">1.0</param><param name="effort">1.0</param></joint>
    <joint name="bare"><command_interface name="position"/></joint>
  </ros2_control>
</robot>
"""


def _root() -> ET.Element:
    return ET.fromstring(_URDF)


def test_command_interfaces_split_velocity_position_and_effort_joints():
    velocity, position, effort = command_interfaces(_root())
    assert velocity == ['wheel_joint']
    assert position == ['lift', 'shoulder', 'marker_joint', 'bare']
    assert effort == ['wrist']


def test_only_movable_joints_with_params_get_a_drive():
    assert set(joint_drives(_root())) == {'lift', 'shoulder', 'wrist'}


def test_a_drive_carries_the_params_the_joint_type_and_the_command_kind():
    drives = joint_drives(_root())
    assert (drives['lift'].stiffness, drives['lift'].damping, drives['lift'].max_force) == (83385.0, 2381.2, 2000.0)
    assert not drives['lift'].angular
    assert drives['shoulder'].angular
    assert [drives[name].kind for name in ('lift', 'shoulder', 'wrist')] == ['position', 'position', 'effort']


def test_dynamics_damping_is_stripped_from_the_named_joints_only():
    root = _root()
    strip_dynamics_damping(root, ['lift', 'shoulder'])
    assert 'damping' not in root.find("joint[@name='lift']/dynamics").attrib
    assert root.find("joint[@name='lift']/dynamics").attrib['friction'] == '1.0'
    assert 'damping' not in root.find("joint[@name='shoulder']/dynamics").attrib
    assert root.find("joint[@name='wheel_joint']/dynamics").attrib['damping'] == '0.5'
