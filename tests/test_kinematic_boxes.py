from __future__ import annotations

import numpy as np
import pytest

from isaac_utils.utils.kinematic import KinematicBoxes

_DT = 1.0 / 60.0
_LEVEL = (1.0, 0.0, 0.0, 0.0)
_YAW_90 = (np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))


def _boxes() -> KinematicBoxes:
    boxes = KinematicBoxes()
    boxes.add('/World/stair/tread_0', (0.0, 0.0, 0.0), _LEVEL, (0.5, 0.0, 0.25))
    boxes.add('/World/stair/tread_1', (0.4, 0.0, 0.2), _YAW_90, (0.5, 0.0, 0.25))
    return boxes


def test_a_step_advances_every_box_by_its_velocity_times_dt():
    boxes = _boxes()
    for _ in range(60):
        boxes.step(_DT)
    rows = boxes.transforms(boxes.paths)
    assert rows[:, :3] == pytest.approx(np.array([[0.5, 0.0, 0.25], [0.9, 0.0, 0.45]]), abs=1e-6)


def test_transforms_are_xyz_then_xyzw_quaternion_in_the_requested_order():
    boxes = _boxes()
    rows = boxes.transforms(('/World/stair/tread_1', '/World/stair/tread_0'))
    assert rows.dtype == np.float32
    assert rows.shape == (2, 7)
    assert rows[0] == pytest.approx([0.4, 0.0, 0.2, 0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
    assert rows[1] == pytest.approx([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])


def test_transforms_of_an_unknown_path_raise():
    with pytest.raises(KeyError):
        _boxes().transforms(('/World/stair/tread_7',))


def test_teleport_resets_the_pose_and_later_steps_advance_from_there():
    boxes = _boxes()
    boxes.step(_DT)
    assert boxes.teleport('/World/stair/tread_0', (0.0, 0.0, 100.0), _YAW_90)
    boxes.step(_DT)
    assert boxes.transforms(('/World/stair/tread_0',))[0] == pytest.approx([0.5 * _DT, 0.0, 100.0 + 0.25 * _DT, 0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])


def test_velocity_change_applies_from_the_next_step_and_zero_holds_the_box():
    boxes = _boxes()
    boxes.step(_DT)
    assert boxes.set_velocity('/World/stair/tread_0', (0.0, 0.0, 0.0))
    for _ in range(10):
        boxes.step(_DT)
    assert boxes.transforms(('/World/stair/tread_0',))[0, :3] == pytest.approx([0.5 * _DT, 0.0, 0.25 * _DT])


def test_unknown_paths_are_refused_without_touching_the_registry():
    boxes = _boxes()
    revision = boxes.revision
    assert not boxes.teleport('/World/wall', (0.0, 0.0, 0.0), _LEVEL)
    assert not boxes.set_velocity('/World/wall', (1.0, 0.0, 0.0))
    assert not boxes.forget_under('/World/wall')
    assert boxes.revision == revision
    assert len(boxes) == 2


def test_forget_under_drops_the_subtree_only_and_bumps_the_revision():
    boxes = _boxes()
    boxes.add('/World/stairless/tread_0', (0.0, 0.0, 0.0), _LEVEL, (1.0, 0.0, 0.0))
    revision = boxes.revision
    assert boxes.forget_under('/World/stair')
    assert boxes.paths == ('/World/stairless/tread_0',)
    assert boxes.revision == revision + 1
    assert '/World/stair/tread_0' not in boxes
