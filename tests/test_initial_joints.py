from pathlib import Path

import pytest

from isaac_utils.utils.initial_joints import initial_positions

_URDF = """<robot name="r">
  <ros2_control name="bridge_position" type="system">
    <hardware><plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin></hardware>
    <joint name="FR_thigh_joint">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.85</param></state_interface>
      <state_interface name="velocity"><param name="initial_value">0.0</param></state_interface>
    </joint>
    <joint name="FR_calf_joint">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">-1.7</param></state_interface>
    </joint>
    <joint name="FR_hip_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""


def test_position_initial_values_become_the_spawn_pose(tmp_path: Path):
    urdf = tmp_path / 'r.urdf'
    urdf.write_text(_URDF)
    assert initial_positions(str(urdf)) == pytest.approx({'FR_thigh_joint': 0.85, 'FR_calf_joint': -1.7})
