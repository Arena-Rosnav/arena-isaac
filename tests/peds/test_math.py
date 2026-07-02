from __future__ import annotations

import numpy as np

from peds.providers.math import quat_normalize, quat_slerp


def _tile(quat: np.ndarray, j: int) -> np.ndarray:
    return np.tile(np.asarray(quat, dtype=float), (j, 1))


def test_slerp_at_t0_returns_a() -> None:
    a = _tile([0.0, 0.0, 0.0, 1.0], 3)
    b = _tile([0.0, 0.0, 1.0, 0.0], 3)
    result = quat_slerp(a, b, 0.0)
    np.testing.assert_allclose(result, quat_normalize(a), atol=1e-9)


def test_slerp_at_t1_returns_b() -> None:
    a = _tile([0.0, 0.0, 0.0, 1.0], 2)
    b = _tile([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)], 2)
    result = quat_slerp(a, b, 1.0)
    np.testing.assert_allclose(result, quat_normalize(b), atol=1e-9)


def test_slerp_identity_when_a_equals_b() -> None:
    a = _tile([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)], 4)
    result = quat_slerp(a, a, 0.37)
    np.testing.assert_allclose(result, quat_normalize(a), atol=1e-6)


def test_slerp_midpoint_known_quats() -> None:
    # identity -> 90 deg about z, midpoint must be the 45 deg about z quaternion
    a = np.array([[0.0, 0.0, 0.0, 1.0]])
    b = np.array([[0.0, 0.0, np.sin(np.pi / 4.0), np.cos(np.pi / 4.0)]])
    expected = np.array([[0.0, 0.0, np.sin(np.pi / 8.0), np.cos(np.pi / 8.0)]])
    result = quat_slerp(a, b, 0.5)
    np.testing.assert_allclose(result, expected, atol=1e-9)


def test_slerp_antipodal_takes_shortest_path() -> None:
    a = np.array([[0.0, 0.0, 0.0, 1.0]])
    ninety = np.array([[0.0, 0.0, np.sin(np.pi / 4.0), np.cos(np.pi / 4.0)]])
    antipodal_b = -ninety
    expected = np.array([[0.0, 0.0, np.sin(np.pi / 8.0), np.cos(np.pi / 8.0)]])
    result = quat_slerp(a, antipodal_b, 0.5)
    np.testing.assert_allclose(result, expected, atol=1e-9)


def test_slerp_output_is_normalized() -> None:
    a = np.array([[1.0, 2.0, 3.0, 4.0]])
    b = np.array([[4.0, 3.0, 2.0, 1.0]])
    result = quat_slerp(a, b, 0.3)
    np.testing.assert_allclose(np.linalg.norm(result, axis=-1), 1.0, atol=1e-9)


def test_slerp_tiny_angle_falls_back_to_nlerp_and_stays_finite() -> None:
    a = np.array([[0.0, 0.0, 0.0, 1.0]])
    tiny = 1e-8
    b = np.array([[0.0, 0.0, np.sin(tiny), np.cos(tiny)]])
    result = quat_slerp(a, b, 0.5)
    assert np.all(np.isfinite(result))
    expected = quat_normalize(0.5 * a + 0.5 * b)
    np.testing.assert_allclose(result, expected, atol=1e-6)


def test_quat_normalize_unit_length() -> None:
    q = np.array([[2.0, 0.0, 0.0, 0.0], [0.0, 3.0, 4.0, 0.0]])
    result = quat_normalize(q)
    np.testing.assert_allclose(np.linalg.norm(result, axis=-1), 1.0, atol=1e-9)
