"""Sole contacts: picked from the bind mesh by the converter, planted per pose by Soles."""

from __future__ import annotations

import math

import numpy as np
import pytest

from peds.convert import JointNode, MeshData, SkinData, SkinnedMesh, _soles
from peds.ground import Soles
from peds.providers.base import JointPose


def _pose(rotations: list[list[float]], translations: list[list[float]]) -> JointPose:
    return JointPose(rotations=np.array(rotations, dtype=float), translations=np.array(translations, dtype=float))


def test_height_is_the_lowest_sole_point_through_the_chain() -> None:
    soles = Soles([-1, 0, 1], [2, 2], np.array([[0.1, 0.0, -0.1], [-0.1, 0.0, -0.05]]))
    still = _pose([[0, 0, 0, 1]] * 3, [[0, 0, 1.0], [0, 0, -0.4], [0, 0, -0.5]])
    assert soles.height(still) == pytest.approx(1.0 - 0.4 - 0.5 - 0.1)

    half = math.sin(math.pi / 4)
    pitched = _pose([[0, 0, 0, 1], [0, half, 0, half], [0, 0, 0, 1]], [[0, 0, 1.0], [0, 0, -0.4], [0, 0, -0.5]])
    assert soles.height(pitched) == pytest.approx(1.0 - 0.4 - 0.1)


def _rig(names: list[str], parents: list[int], binds: dict[str, list[float]], vertices: list[tuple[list[float], str]]) -> tuple[list[JointNode], list[SkinnedMesh]]:
    joints = [JointNode(name=name, path=name, parent=parent, rest=np.eye(4)) for name, parent in zip(names, parents, strict=True)]
    inv_bind = np.stack([np.eye(4) for _ in names])
    for k, name in enumerate(names):
        inv_bind[k, :3, 3] = -np.array(binds.get(name, [0.0, 0.0, 0.0]))
    skin = SkinData(joint_names=names, inv_bind=inv_bind, bind_shape=np.eye(4), influences=[[(names.index(bone), 1.0)] for _, bone in vertices])
    mesh = MeshData(points=np.array([point for point, _ in vertices]), face_counts=[], face_indices=[], normals=None, uvs=None, subsets={})
    return joints, [SkinnedMesh(name="body", skin=skin, mesh=mesh, bound_material=None)]


_FEET = ["Hips", "LeftFoot", "LeftToeBase", "RightFoot", "RightToeBase"]
_BINDS = {"LeftFoot": [0.0, 0.1, 0.1], "LeftToeBase": [0.15, 0.1, 0.03], "RightFoot": [0.0, -0.1, 0.1], "RightToeBase": [0.15, -0.1, 0.03]}
_SHOE = [
    ([-0.05, 0.1, 0.004], "LeftFoot"),
    ([-0.04, 0.1, 0.0], "LeftFoot"),
    ([0.0, 0.1, 0.02], "LeftFoot"),
    ([0.18, 0.1, 0.002], "LeftToeBase"),
    ([0.20, 0.1, 0.006], "LeftToeBase"),
    ([0.0, 0.0, 0.9], "Hips"),
    ([-0.04, -0.1, 0.0], "RightFoot"),
    ([0.19, -0.1, 0.001], "RightToeBase"),
]


def test_soles_take_the_lowest_rear_and_front_sole_vertex_of_each_foot_in_its_bone_frame() -> None:
    joints, skinned = _rig(_FEET, [-1, 0, 1, 0, 3], _BINDS, _SHOE)
    soles = _soles(joints, skinned)
    picked = [(joints[sole["joint"]].name, sole["point"]) for sole in soles]
    assert [name for name, _ in picked] == ["LeftFoot", "LeftToeBase", "RightFoot", "RightToeBase"]
    assert picked[0][1] == pytest.approx([-0.04, 0.0, -0.1])
    assert picked[1][1] == pytest.approx([0.03, 0.0, -0.028])
    assert picked[3][1] == pytest.approx([0.04, 0.0, -0.029])


def test_a_rig_on_wheels_has_no_soles() -> None:
    joints, skinned = _rig([*_FEET, "WheelL"], [-1, 0, 1, 0, 3, 0], _BINDS, _SHOE)
    assert _soles(joints, skinned) == []
