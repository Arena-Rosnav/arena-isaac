from __future__ import annotations

import numpy as np

from peds.providers.buffer import StampedPoseBuffer


def _frame(joints: int, yaw: float, x: float) -> tuple[np.ndarray, np.ndarray]:
    rotations = np.tile(np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]), (joints, 1))
    translations = np.tile(np.array([x, 0.0, 0.0]), (joints, 1))
    return rotations, translations


def test_evaluate_empty_buffer_returns_none() -> None:
    buf = StampedPoseBuffer()
    assert buf.evaluate(0.0) is None


def test_evaluate_interpolates_between_two_frames() -> None:
    buf = StampedPoseBuffer()
    r0, t0 = _frame(2, 0.0, 0.0)
    r1, t1 = _frame(2, np.pi / 2.0, 2.0)
    buf.append(0.0, r0, t0)
    buf.append(1.0, r1, t1)

    pose = buf.evaluate(0.5)
    assert pose is not None
    np.testing.assert_allclose(pose.translations, np.tile([1.0, 0.0, 0.0], (2, 1)), atol=1e-9)
    expected_rot = np.tile([0.0, 0.0, np.sin(np.pi / 8.0), np.cos(np.pi / 8.0)], (2, 1))
    np.testing.assert_allclose(pose.rotations, expected_rot, atol=1e-9)


def test_evaluate_clamps_beyond_newest() -> None:
    buf = StampedPoseBuffer()
    r0, t0 = _frame(1, 0.0, 0.0)
    r1, t1 = _frame(1, np.pi / 2.0, 2.0)
    buf.append(0.0, r0, t0)
    buf.append(1.0, r1, t1)

    pose = buf.evaluate(5.0)
    assert pose is not None
    np.testing.assert_allclose(pose.translations, t1, atol=1e-9)
    np.testing.assert_allclose(pose.rotations, r1, atol=1e-9)


def test_evaluate_clamps_before_oldest() -> None:
    buf = StampedPoseBuffer()
    r0, t0 = _frame(1, 0.0, 0.0)
    r1, t1 = _frame(1, np.pi / 2.0, 2.0)
    buf.append(1.0, r0, t0)
    buf.append(2.0, r1, t1)

    pose = buf.evaluate(0.0)
    assert pose is not None
    np.testing.assert_allclose(pose.translations, t0, atol=1e-9)


def test_newest_stamp() -> None:
    buf = StampedPoseBuffer()
    assert buf.newest_stamp is None
    r0, t0 = _frame(1, 0.0, 0.0)
    buf.append(3.0, r0, t0)
    assert buf.newest_stamp == 3.0


def test_staleness_zero_when_within_range() -> None:
    buf = StampedPoseBuffer()
    r0, t0 = _frame(1, 0.0, 0.0)
    buf.append(1.0, r0, t0)
    assert buf.staleness(0.5) == 0.0


def test_staleness_positive_when_stale() -> None:
    buf = StampedPoseBuffer()
    r0, t0 = _frame(1, 0.0, 0.0)
    buf.append(1.0, r0, t0)
    assert abs(buf.staleness(2.5) - 1.5) < 1e-9


def test_staleness_empty_buffer_is_zero() -> None:
    buf = StampedPoseBuffer()
    assert buf.staleness(10.0) == 0.0


def test_maxlen_evicts_oldest_frame() -> None:
    buf = StampedPoseBuffer(maxlen=2)
    r, t = _frame(1, 0.0, 0.0)
    buf.append(0.0, r, t)
    buf.append(1.0, r, t)
    buf.append(2.0, r, t)
    # the frame stamped 0.0 was evicted, evaluating before it now clamps to the oldest survivor
    pose = buf.evaluate(-1.0)
    assert pose is not None
    assert buf.newest_stamp == 2.0
