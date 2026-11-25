from __future__ import annotations

import numpy as np
import pytest

from isaac_utils.utils.pose_math import compose, relative

_YAW_90 = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
_LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


def test_an_object_beside_a_yawed_base_sits_on_the_base_x_axis_in_its_frame():
    position, orientation = relative(np.array([1.0, 2.0, 0.0]), _YAW_90, np.array([1.0, 3.0, 0.5]), _LEVEL)
    assert position == pytest.approx([1.0, 0.0, 0.5], abs=1e-9)
    assert orientation == pytest.approx([np.sqrt(0.5), 0.0, 0.0, -np.sqrt(0.5)], abs=1e-9)


def test_composing_the_relative_pose_onto_the_base_restores_the_world_pose():
    base_position = np.array([0.4, -1.2, 0.9])
    base_orientation = np.array([0.8, 0.1, -0.5, 0.3])
    position = np.array([0.7, -0.9, 1.1])
    orientation = np.array([0.3, 0.6, 0.2, -0.7])
    orientation = orientation / np.linalg.norm(orientation)
    offset = relative(base_position, base_orientation, position, orientation)
    restored_position, restored_orientation = compose(base_position, base_orientation, *offset)
    assert restored_position == pytest.approx(position, abs=1e-9)
    assert restored_orientation == pytest.approx(orientation, abs=1e-9)


def test_a_moved_base_carries_the_offset_with_it():
    offset = (np.array([0.0, 0.0, 0.2]), _LEVEL)
    position, _ = compose(np.array([3.0, 0.0, 1.0]), _YAW_90, *offset)
    assert position == pytest.approx([3.0, 0.0, 1.2], abs=1e-9)
