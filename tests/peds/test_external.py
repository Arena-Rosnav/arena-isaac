from __future__ import annotations

import math

import numpy as np
import pytest
from peds.providers.base import JointPose
from peds.providers.bone_map import BONE_MAP
from peds.providers.external import ExternalPoseProvider, _AngleRingBuffer, axis_angle_quat, quat_multiply
from peds.providers.math import quat_normalize, quat_slerp


def _expected_delta(joint: str, angle: float) -> np.ndarray:
    """The bone-local delta external.py builds for a pushed wire angle."""
    target = BONE_MAP[joint]
    assert target is not None
    return axis_angle_quat(target.axis, target.sign * angle)


class _FakeGait:
    """Constant-pose PoseProvider stub for testing ExternalPoseProvider, ignores sim_time/dt."""

    def __init__(self, rotations: np.ndarray, translations: np.ndarray) -> None:
        self._rotations = rotations
        self._translations = translations

    def evaluate(self, sim_time: float, dt: float) -> JointPose:
        return JointPose(rotations=self._rotations.copy(), translations=self._translations.copy())


def test_mapped_bone_at_weight_one_independent_of_fallback_sample() -> None:
    joint_order = ("Hips", "LeftLeg")  # Hips unmapped, LeftLeg is the l_knee target
    translations = np.zeros((2, 3))
    angle = math.pi / 3.0

    rot_a = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (2, 1))
    rot_b = rot_a.copy()
    rot_b[1] = axis_angle_quat((1.0, 0.0, 0.0), 0.9)  # a different sampled LeftLeg rotation

    provider_a = ExternalPoseProvider(_FakeGait(rot_a, translations), joint_order)
    provider_a.push(0.0, ["l_knee"], [angle])
    pose_a = provider_a.evaluate(sim_time=0.0, dt=1.0)  # dt >> blend_s snaps weight to 1.0

    provider_b = ExternalPoseProvider(_FakeGait(rot_b, translations), joint_order)
    provider_b.push(0.0, ["l_knee"], [angle])
    pose_b = provider_b.evaluate(sim_time=0.0, dt=1.0)

    np.testing.assert_allclose(pose_a.rotations[1], pose_b.rotations[1], atol=1e-9)
    np.testing.assert_allclose(pose_a.rotations[1], _expected_delta("l_knee", angle), atol=1e-9)


def test_full_path_joint_order_matches_bone_map_leaves() -> None:
    joint_order = ("Hips", "Hips/LeftUpLeg", "Hips/LeftUpLeg/LeftLeg")  # converter authors full paths
    rotations = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (3, 1))
    provider = ExternalPoseProvider(_FakeGait(rotations, np.zeros((3, 3))), joint_order)

    angle = math.pi / 4.0
    provider.push(0.0, ["l_knee"], [angle])
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    np.testing.assert_allclose(pose.rotations[2], _expected_delta("l_knee", angle), atol=1e-9)


def test_unmapped_bone_passes_fallback_through() -> None:
    joint_order = ("Hips", "LeftLeg")  # Hips is not a BONE_MAP target
    fallback_hips = axis_angle_quat((0.0, 0.0, 1.0), 0.4)
    rotations = np.stack([fallback_hips, np.array([0.0, 0.0, 0.0, 1.0])])
    translations = np.zeros((2, 3))
    provider = ExternalPoseProvider(_FakeGait(rotations, translations), joint_order)

    provider.push(0.0, ["l_knee"], [math.pi / 2.0])
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    np.testing.assert_allclose(pose.rotations[0], fallback_hips, atol=1e-9)
    np.testing.assert_allclose(pose.rotations[1], _expected_delta("l_knee", math.pi / 2.0), atol=1e-9)
    np.testing.assert_allclose(pose.translations, translations, atol=1e-9)


def test_mapped_bone_seeds_from_explicit_neutral_pose() -> None:
    joint_order = ("LeftLeg",)
    fallback = _FakeGait(np.array([[0.0, 0.0, 0.0, 1.0]]), np.zeros((1, 3)))
    neutral = np.array([axis_angle_quat((0.0, 1.0, 0.0), 0.6)])  # non-identity neutral pose
    provider = ExternalPoseProvider(fallback, joint_order, neutral_rotations=neutral)

    angle = math.pi / 4.0
    provider.push(0.0, ["l_knee"], [angle])
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    delta = _expected_delta("l_knee", angle)
    override = quat_normalize(quat_multiply(neutral[0], delta))
    np.testing.assert_allclose(pose.rotations[0], override, atol=1e-9)
    assert not np.allclose(override, delta, atol=1e-3)  # distinct from an identity (fallback-relative) seed


def test_angle_ring_buffer_interpolates_at_midpoint() -> None:
    buf = _AngleRingBuffer(maxlen=8)
    buf.append(0.0, np.array([0.0, 1.0]))
    buf.append(1.0, np.array([2.0, 3.0]))

    result = buf.evaluate(0.5)
    assert result is not None
    np.testing.assert_allclose(result, [1.0, 2.0], atol=1e-9)


def test_angle_ring_buffer_clamps_beyond_newest() -> None:
    buf = _AngleRingBuffer(maxlen=8)
    buf.append(0.0, np.array([0.0, 1.0]))
    buf.append(1.0, np.array([2.0, 3.0]))

    result = buf.evaluate(5.0)
    assert result is not None
    np.testing.assert_allclose(result, [2.0, 3.0], atol=1e-9)


def test_staleness_decays_to_fallback_then_ramps_back_on_fresh_push() -> None:
    joint_order = ("LeftLeg",)
    rotations = np.array([[0.0, 0.0, 0.0, 1.0]])
    translations = np.zeros((1, 3))
    fallback = _FakeGait(rotations, translations)
    provider = ExternalPoseProvider(fallback, joint_order, staleness_s=0.5, blend_s=0.3)

    angle = math.pi / 2.0
    delta = _expected_delta("l_knee", angle)
    identity = np.array([0.0, 0.0, 0.0, 1.0])

    provider.push(0.0, ["l_knee"], [angle])
    pose = provider.evaluate(sim_time=0.0, dt=1.0)  # snap weight 0.0 -> 1.0, fresh
    np.testing.assert_allclose(pose.rotations[0], delta, atol=1e-9)

    pose = provider.evaluate(sim_time=0.6, dt=0.15)  # stale (0.6 - 0.0 > 0.5), weight ramps 1.0 -> 0.5
    expected = quat_slerp(identity[None, :], delta[None, :], 0.5)[0]
    np.testing.assert_allclose(pose.rotations[0], expected, atol=1e-9)

    pose = provider.evaluate(sim_time=0.75, dt=0.15)  # weight ramps 0.5 -> 0.0, pure fallback now
    np.testing.assert_allclose(pose.rotations[0], identity, atol=1e-9)

    provider.push(0.75, ["l_knee"], [angle])  # fresh push
    pose = provider.evaluate(sim_time=0.75, dt=0.15)  # fresh again, weight ramps 0.0 -> 0.5, no pop
    expected = quat_slerp(identity[None, :], delta[None, :], 0.5)[0]
    np.testing.assert_allclose(pose.rotations[0], expected, atol=1e-9)

    pose = provider.evaluate(sim_time=0.9, dt=0.15)  # weight ramps 0.5 -> 1.0
    np.testing.assert_allclose(pose.rotations[0], delta, atol=1e-9)


def test_unknown_names_ignored_without_error_or_output(capsys: pytest.CaptureFixture[str]) -> None:
    joint_order = ("LeftLeg",)
    rotations = np.array([[0.0, 0.0, 0.0, 1.0]])
    translations = np.zeros((1, 3))
    fallback = _FakeGait(rotations, translations)
    provider = ExternalPoseProvider(fallback, joint_order)

    provider.push(0.0, ["l_knee", "not_a_real_joint"], [0.1, 0.2])
    provider.push(0.05, ["not_a_real_joint"], [0.3])  # repeated unknown name, still no raise

    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_axis_angle_quat_90_degrees_about_z() -> None:
    q = axis_angle_quat((0.0, 0.0, 1.0), math.pi / 2.0)
    expected = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])
    np.testing.assert_allclose(q, expected, atol=1e-9)


def test_quat_multiply_identity_is_neutral() -> None:
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    q = axis_angle_quat((0.0, 0.0, 1.0), math.pi / 2.0)
    np.testing.assert_allclose(quat_multiply(identity, q), q, atol=1e-9)
    np.testing.assert_allclose(quat_multiply(q, identity), q, atol=1e-9)


def test_quat_multiply_composes_two_quarter_turns_into_half_turn() -> None:
    q = axis_angle_quat((0.0, 0.0, 1.0), math.pi / 2.0)
    composed = quat_multiply(q, q)
    expected = axis_angle_quat((0.0, 0.0, 1.0), math.pi)
    np.testing.assert_allclose(composed, expected, atol=1e-9)


def test_blend_hemisphere_continuity_across_weight_ramp() -> None:
    joint_order = ("LeftArm",)
    rotations = np.array([[0.0, 0.0, 0.0, 1.0]])
    translations = np.zeros((1, 3))
    fallback = _FakeGait(rotations, translations)
    provider = ExternalPoseProvider(fallback, joint_order, staleness_s=0.5, blend_s=0.3)

    angle = 3.3  # near the l_p_shoulder joint limit, half-angle exceeds pi/2 so the raw delta has w < 0
    delta = _expected_delta("l_p_shoulder", angle)
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    assert delta[3] < 0.0  # sanity: this scenario really exercises antipodal correction
    assert float(np.dot(identity, delta)) < 0.0  # fallback and raw override land in opposite hemispheres

    provider.push(0.0, ["l_p_shoulder"], [angle])

    quats = [provider.evaluate(sim_time=0.0, dt=0.05).rotations[0].copy() for _ in range(6)]

    for a, b in zip(quats, quats[1:], strict=False):
        assert np.dot(a, b) > 0.9  # no hemisphere flip between consecutive ramp steps

    for k, q in enumerate(quats):
        weight = min(1.0, (k + 1) * 0.05 / 0.3)
        expected = quat_slerp(identity[None, :], delta[None, :], weight)[0]
        np.testing.assert_allclose(q, expected, atol=1e-9)
