from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
import pytest
from arena_simulation_setup.testing import make_rig
from peds.providers.external import ExternalPoseProvider, _Frame, _FrameBuffer, axis_angle_quat, quat_multiply
from peds.providers.math import quat_slerp

ORDER = (
    "Hips",
    "Hips/Helper",
    "Hips/Helper/Thigh",
    "Hips/Spine",
    "Hips/Spine/Neck",
    "Hips/Spine/Neck/Neck1",
    "Hips/Spine/Neck/Neck1/Finger",
    "Hips/WheelL",
)
HIPS, HELPER, THIGH, SPINE, NECK, NECK1, FINGER, WHEEL = range(8)

IDENTITY = np.array([0.0, 0.0, 0.0, 1.0])


def _q(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    return axis_angle_quat(axis, angle)


X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)

OFFSETS = {"Hips": _q(X, 0.1), "Thigh": _q(Y, -0.4), "Spine": _q(Z, 0.2), "Neck": _q(X, 0.3), "Neck1": _q(Y, 0.25)}
JOINTS: dict[str, tuple[str | None, list[tuple[str, float, Sequence[float]]]]] = {
    "pelvis": (None, [("Hips", 1.0, OFFSETS["Hips"].tolist())]),
    "left_hip": ("pelvis", [("Thigh", 1.0, OFFSETS["Thigh"].tolist())]),
    "spine1": ("pelvis", [("Spine", 1.0, OFFSETS["Spine"].tolist())]),
    "neck": ("spine1", [("Neck", 0.5, OFFSETS["Neck"].tolist()), ("Neck1", 0.5, OFFSETS["Neck1"].tolist())]),
}
STANCE = {"pelvis": _q(Z, 0.1), "left_hip": _q(Y, 0.7), "spine1": _q(Z, 0.3), "neck": _q(X, -0.2)}
DRIVERS: dict[str, dict[str, Any]] = {"l_wheel": {"bone": "WheelL", "from": "distance", "axis": [0.0, 1.0, 0.0]}}

CLIP = np.tile(IDENTITY, (len(ORDER), 1))
CLIP[HELPER] = _q(Z, 0.3)
CLIP[FINGER] = _q(X, 0.5)
CLIP[WHEEL] = _q(X, 0.2)
TRANSLATIONS = np.arange(len(ORDER) * 3, dtype=float).reshape(-1, 3) * 0.01


def _provider(
    joints: Mapping[str, tuple[str | None, list[tuple[str, float, Sequence[float]]]]] | None = None,
    stance: Mapping[str, Sequence[float]] | None = None,
    drivers: Mapping[str, Mapping[str, Any]] | None = None,
    order: tuple[str, ...] = ORDER,
    **kwargs: Any,
) -> ExternalPoseProvider:
    rig = make_rig(JOINTS if joints is None else joints, STANCE if stance is None else stance, drivers=DRIVERS if drivers is None else drivers)
    return ExternalPoseProvider(order, CLIP[: len(order)], TRANSLATIONS[: len(order)], rig, **kwargs)


def _push(
    provider: ExternalPoseProvider,
    stamp: float,
    pose: Mapping[str, np.ndarray],
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    drivers: Mapping[str, float] | None = None,
) -> None:
    drivers = drivers or {}
    provider.push(stamp, list(pose), [q.tolist() for q in pose.values()], offset, list(drivers), list(drivers.values()))


def _worlds(rotations: np.ndarray) -> list[np.ndarray]:
    worlds: list[np.ndarray] = []
    for i, path in enumerate(ORDER[: len(rotations)]):
        worlds.append(rotations[i] if "/" not in path else quat_multiply(worlds[ORDER.index(path.rsplit("/", 1)[0])], rotations[i]))
    return worlds


def _same(actual: np.ndarray, expected: np.ndarray) -> None:
    assert abs(float(np.dot(actual, expected))) == pytest.approx(1.0, abs=1e-9)


def _same_rows(actual: np.ndarray, expected: np.ndarray) -> None:
    for a, e in zip(actual, expected, strict=True):
        _same(a, e)


def _snap(provider: ExternalPoseProvider) -> np.ndarray:
    return provider.evaluate(sim_time=0.0, dt=1.0).rotations


def test_mapped_bone_world_is_joint_global_times_offset() -> None:
    provider = _provider()
    pelvis, hip = _q(Z, 0.5), _q(X, 0.3)
    _push(provider, 0.0, {"pelvis": pelvis, "left_hip": hip})
    worlds = _worlds(_snap(provider))

    _same(worlds[HIPS], quat_multiply(pelvis, OFFSETS["Hips"]))
    _same(worlds[THIGH], quat_multiply(quat_multiply(pelvis, hip), OFFSETS["Thigh"]))


def test_joint_missing_from_the_wire_keeps_its_stance() -> None:
    held = _q(Y, 0.6)
    provider = _provider(stance={"pelvis": _q(Z, 0.2), "spine1": held})
    pelvis = _q(Z, 0.5)
    _push(provider, 0.0, {"pelvis": pelvis})
    worlds = _worlds(_snap(provider))

    _same(worlds[SPINE], quat_multiply(quat_multiply(pelvis, quat_multiply(_q(Z, -0.2), held)), OFFSETS["Spine"]))


def test_spread_joint_splits_its_angle_by_weight() -> None:
    provider = _provider()
    pelvis, spine, neck = _q(X, 0.2), _q(Y, 0.3), _q(Z, 0.8)
    _push(provider, 0.0, {"pelvis": pelvis, "spine1": spine, "neck": neck})
    worlds = _worlds(_snap(provider))

    spine_global = quat_multiply(pelvis, spine)
    _same(worlds[NECK], quat_multiply(quat_multiply(spine_global, _q(Z, 0.4)), OFFSETS["Neck"]))
    _same(worlds[NECK1], quat_multiply(quat_multiply(spine_global, neck), OFFSETS["Neck1"]))


def test_spread_joint_children_follow_the_full_rotation() -> None:
    provider = _provider(joints={**JOINTS, "head": ("neck", [("Finger", 1.0, [0.0, 0.0, 0.0, 1.0])])})
    neck, head = _q(Z, 0.8), _q(X, 0.1)
    _push(provider, 0.0, {"pelvis": _q(Z, 0.0), "spine1": _q(Z, 0.0), "neck": neck, "head": head})
    worlds = _worlds(_snap(provider))

    _same(worlds[FINGER], quat_multiply(neck, head))


def test_unmapped_bones_hold_neutral_rotation_and_translation() -> None:
    provider = _provider()
    _push(provider, 0.0, {"pelvis": _q(Z, 0.5), "left_hip": _q(X, 0.3)})
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    _same(pose.rotations[HELPER], CLIP[HELPER])
    _same(pose.rotations[FINGER], CLIP[FINGER])
    np.testing.assert_allclose(pose.translations, TRANSLATIONS, atol=1e-9)


def test_root_offset_moves_only_the_top_bone() -> None:
    provider = _provider()
    _push(provider, 0.0, {"pelvis": IDENTITY}, offset=(0.1, -0.2, 0.3))
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    expected = TRANSLATIONS.copy()
    expected[HIPS] += [0.1, -0.2, 0.3]
    np.testing.assert_allclose(pose.translations, expected, atol=1e-9)


def test_no_push_yields_the_stance_world_rotations() -> None:
    provider = _provider()
    pose = provider.evaluate(sim_time=3.0, dt=0.05)
    worlds = _worlds(pose.rotations)

    _same(worlds[HIPS], quat_multiply(STANCE["pelvis"], OFFSETS["Hips"]))
    _same(worlds[THIGH], quat_multiply(STANCE["left_hip"], OFFSETS["Thigh"]))
    _same(worlds[SPINE], quat_multiply(STANCE["spine1"], OFFSETS["Spine"]))
    _same(worlds[NECK1], quat_multiply(STANCE["neck"], OFFSETS["Neck1"]))
    _same(pose.rotations[HELPER], CLIP[HELPER])
    np.testing.assert_allclose(pose.translations, TRANSLATIONS, atol=1e-9)


def test_a_rig_listing_a_joint_before_its_parent_is_rejected() -> None:
    with pytest.raises(ValueError, match="parents first"):
        _provider(joints=dict(reversed(list(JOINTS.items()))))


def test_driver_bone_turns_about_its_axis_from_the_neutral_rotation() -> None:
    provider = _provider()
    angle = 3.0 * math.tau + 0.5
    _push(provider, 0.0, {"pelvis": IDENTITY}, drivers={"l_wheel": angle})
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    _same(pose.rotations[WHEEL], quat_multiply(CLIP[WHEEL], _q(Y, 0.5)))
    _same(pose.rotations[HIPS], OFFSETS["Hips"])


def test_driver_without_a_bone_in_the_skeleton_is_ignored() -> None:
    provider = _provider(order=ORDER[:-1])
    _push(provider, 0.0, {"pelvis": IDENTITY}, drivers={"l_wheel": 4.0})
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    assert len(pose.rotations) == len(ORDER) - 1
    _same_rows(pose.rotations[[HELPER, FINGER]], CLIP[[HELPER, FINGER]])


def test_rig_without_joints_is_not_driven() -> None:
    provider = _provider(joints={}, stance={}, drivers={})
    _push(provider, 0.0, {"pelvis": _q(Z, 1.0), "left_hip": _q(X, 1.0)})
    pose = provider.evaluate(sim_time=0.0, dt=1.0)

    _same_rows(pose.rotations, CLIP)
    np.testing.assert_allclose(pose.translations, TRANSLATIONS, atol=1e-9)


def test_frame_buffer_interpolates_at_midpoint() -> None:
    buf = _FrameBuffer(maxlen=8)
    buf.append(0.0, _Frame(np.array([IDENTITY]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 1.0])))
    buf.append(1.0, _Frame(np.array([_q(Z, 1.0)]), np.array([0.0, 3.0, 0.0]), np.array([2.0, 3.0])))

    result = buf.evaluate(0.5)
    assert result is not None
    _same(result.rotations[0], _q(Z, 0.5))
    np.testing.assert_allclose(result.offset, [0.0, 2.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(result.drivers, [1.0, 2.0], atol=1e-9)


def test_frame_buffer_aligns_antipodal_rotations() -> None:
    buf = _FrameBuffer(maxlen=8)
    buf.append(0.0, _Frame(np.array([_q(Z, 0.4)]), np.zeros(3), np.zeros(0)))
    buf.append(1.0, _Frame(np.array([-_q(Z, 0.8)]), np.zeros(3), np.zeros(0)))

    result = buf.evaluate(0.5)
    assert result is not None
    _same(result.rotations[0], _q(Z, 0.6))


def test_frame_buffer_clamps_beyond_newest() -> None:
    buf = _FrameBuffer(maxlen=8)
    buf.append(0.0, _Frame(np.array([IDENTITY]), np.zeros(3), np.array([0.0])))
    buf.append(1.0, _Frame(np.array([_q(Z, 1.0)]), np.zeros(3), np.array([2.0])))

    result = buf.evaluate(5.0)
    assert result is not None
    _same(result.rotations[0], _q(Z, 1.0))
    np.testing.assert_allclose(result.drivers, [2.0], atol=1e-9)


def test_staleness_decays_to_rest_then_ramps_back_on_fresh_push() -> None:
    provider = _provider(staleness_s=0.5, blend_s=0.3)
    pose = {"pelvis": _q(Z, 0.5), "left_hip": _q(X, 1.0)}

    rest = provider.evaluate(sim_time=0.0, dt=0.05).rotations.copy()
    _push(provider, 0.0, pose)
    full = _snap(provider).copy()
    assert not np.allclose(full[THIGH], rest[THIGH], atol=1e-3)

    half = provider.evaluate(sim_time=0.6, dt=0.15).rotations
    np.testing.assert_allclose(half, quat_slerp(rest, full, 0.5), atol=1e-9)

    np.testing.assert_allclose(provider.evaluate(sim_time=0.75, dt=0.15).rotations, rest, atol=1e-9)

    _push(provider, 0.75, pose)
    np.testing.assert_allclose(provider.evaluate(sim_time=0.75, dt=0.15).rotations, quat_slerp(rest, full, 0.5), atol=1e-9)
    np.testing.assert_allclose(provider.evaluate(sim_time=0.9, dt=0.15).rotations, full, atol=1e-9)


def test_blend_ramp_stays_in_one_hemisphere() -> None:
    provider = _provider(staleness_s=0.5, blend_s=0.3)
    pose = {"left_hip": _q(X, 3.3)}
    rest = provider.evaluate(sim_time=0.0, dt=0.05).rotations.copy()
    snap = _provider()
    _push(snap, 0.0, pose)
    full = _snap(snap).copy()

    _push(provider, 0.0, pose)
    quats = [provider.evaluate(sim_time=0.0, dt=0.05).rotations[THIGH].copy() for _ in range(6)]

    for a, b in zip(quats, quats[1:], strict=False):
        assert np.dot(a, b) > 0.9
    for k, q in enumerate(quats):
        weight = min(1.0, (k + 1) * 0.05 / 0.3)
        np.testing.assert_allclose(q, quat_slerp(rest, full, weight)[THIGH], atol=1e-9)


def test_rest_pose_shows_without_wire_and_wire_replaces_it() -> None:
    reference = _provider()
    neutral = reference.evaluate(sim_time=0.0, dt=0.05).rotations.copy()
    rest = neutral.copy()
    rest[THIGH] = _q(X, -1.2)
    rest_translations = TRANSLATIONS + 0.5
    provider = _provider(staleness_s=0.5, blend_s=0.3, rest_rotations=rest, rest_translations=rest_translations)

    pose = provider.evaluate(sim_time=0.0, dt=0.05)
    np.testing.assert_allclose(pose.rotations, rest, atol=1e-9)

    wire = {"pelvis": _q(Z, 0.4), "left_hip": _q(X, 0.5)}
    _push(reference, 0.0, wire)
    full = _snap(reference).copy()
    _push(provider, 0.0, wire)
    pose = provider.evaluate(sim_time=0.0, dt=1.0)
    _same_rows(pose.rotations, full)

    pose = provider.evaluate(sim_time=0.6, dt=0.15)
    np.testing.assert_allclose(pose.rotations[THIGH], quat_slerp(rest, full, 0.5)[THIGH], atol=1e-9)
    pose = provider.evaluate(sim_time=0.75, dt=0.15)
    np.testing.assert_allclose(pose.rotations, rest, atol=1e-9)
    np.testing.assert_allclose(pose.translations, rest_translations, atol=1e-9)


def test_rest_pose_blends_bones_the_wire_does_not_touch() -> None:
    neutral = _provider().evaluate(sim_time=0.0, dt=0.05).rotations.copy()
    rest = neutral.copy()
    rest[FINGER] = _q(X, -1.2)
    provider = _provider(blend_s=0.3, rest_rotations=rest)

    _push(provider, 0.0, {"pelvis": IDENTITY})
    pose = provider.evaluate(sim_time=0.0, dt=0.15)
    np.testing.assert_allclose(pose.rotations[FINGER], quat_slerp(rest, neutral, 0.5)[FINGER], atol=1e-9)
    pose = provider.evaluate(sim_time=0.05, dt=0.15)
    _same(pose.rotations[FINGER], neutral[FINGER])


def test_unknown_names_ignored_without_error_or_output(capsys: pytest.CaptureFixture[str]) -> None:
    provider = _provider()

    _push(provider, 0.0, {"pelvis": IDENTITY, "not_a_real_joint": IDENTITY}, drivers={"not_a_driver": 0.2})
    _push(provider, 0.05, {"not_a_real_joint": IDENTITY})

    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_reset_phase_returns_to_rest() -> None:
    provider = _provider()
    rest = provider.evaluate(sim_time=0.0, dt=0.05).rotations.copy()
    _push(provider, 0.0, {"left_hip": _q(X, 1.0)})
    _snap(provider)

    provider.reset_phase()
    np.testing.assert_allclose(provider.evaluate(sim_time=0.0, dt=1.0).rotations, rest, atol=1e-9)


def test_axis_angle_quat_90_degrees_about_z() -> None:
    q = axis_angle_quat((0.0, 0.0, 1.0), math.pi / 2.0)
    expected = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])
    np.testing.assert_allclose(q, expected, atol=1e-9)


def test_quat_multiply_identity_is_neutral() -> None:
    q = axis_angle_quat((0.0, 0.0, 1.0), math.pi / 2.0)
    np.testing.assert_allclose(quat_multiply(IDENTITY, q), q, atol=1e-9)
    np.testing.assert_allclose(quat_multiply(q, IDENTITY), q, atol=1e-9)


def test_quat_multiply_composes_two_quarter_turns_into_half_turn() -> None:
    q = axis_angle_quat((0.0, 0.0, 1.0), math.pi / 2.0)
    composed = quat_multiply(q, q)
    expected = axis_angle_quat((0.0, 0.0, 1.0), math.pi)
    np.testing.assert_allclose(composed, expected, atol=1e-9)
