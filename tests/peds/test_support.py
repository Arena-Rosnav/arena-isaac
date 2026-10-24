from __future__ import annotations

import math

import numpy as np
import pytest

pytest.importorskip("pxr")

from peds.kinematics import Kinematics
from peds.ped import Ped
from peds.support import LIE_ON, SIT_ON, TYPICAL_CLEARANCE, PelvisClearance, Support, read_pelvis_clearance, supported_root
from peds.providers.base import JointPose
from peds.providers.external import axis_angle_quat
from pxr import Gf

IDENTITY = [0.0, 0.0, 0.0, 1.0]
CLEARANCE = PelvisClearance(sit=0.12, lie=0.11)


def _leg() -> tuple[Kinematics, JointPose]:
    """Hips 0.9 m above the root, a 0.45 m thigh down to the knee, a 0.45 m shin down to the foot."""
    joints = ("Hips", "Hips/Knee", "Hips/Knee/Foot")
    standing = JointPose(rotations=np.array([IDENTITY] * 3), translations=np.array([[0, 0, 0.9], [0, 0, -0.45], [0, 0, -0.45]]))
    return Kinematics(joints, standing), standing


def test_kinematics_reports_pelvis_and_foot_lift() -> None:
    kin, standing = _leg()
    assert kin.heights(standing) == pytest.approx((0.9, 0.0))
    bent = JointPose(rotations=np.array([axis_angle_quat((0, 1, 0), math.pi / 2), axis_angle_quat((0, 1, 0), -math.pi / 2), IDENTITY]), translations=standing.translations)
    pelvis, lowest = kin.heights(bent)  # thigh forward, shin down: a sitting leg
    assert (pelvis, kin.foot_lift(lowest)) == pytest.approx((0.9, 0.45))


def test_sitter_rests_its_pelvis_on_the_seat_once_the_legs_fold() -> None:
    floor, upright = np.array([1.0, 2.0, 0.0]), np.array(IDENTITY)

    def sit(height: float, foot_lift: float) -> np.ndarray:
        position, orientation = supported_root(Support(SIT_ON, height), CLEARANCE, floor, upright, 0.9, foot_lift)
        assert orientation == pytest.approx(upright)
        return position

    assert sit(0.45, 0.0)[2] == pytest.approx(0.0)  # legs still straight: stays on its feet
    assert sit(0.45, 0.45) == pytest.approx([1.0, 2.0, 0.45 + CLEARANCE.sit - 0.9])
    assert sit(0.7, 0.45)[2] + 0.9 == pytest.approx(0.7 + CLEARANCE.sit)  # high seat: the feet hang


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, -2.0])
def test_lier_faces_up_with_the_pelvis_over_its_slot(yaw: float) -> None:
    pelvis = 0.9
    pos, quat = supported_root(Support(LIE_ON, 0.6), CLEARANCE, np.zeros(3), axis_angle_quat((0.0, 0.0, 1.0), yaw), pelvis, 0.0)
    x, y, z, w = quat
    r = np.array(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(w, x, y, z)))).T  # Gf is row-vector: transpose to act on columns
    assert r @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)  # facing -> up
    heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    assert r @ np.array([0.0, 0.0, 1.0]) == pytest.approx(-heading, abs=1e-9)  # head -> opposite the yaw
    assert pos + r @ np.array([0.0, 0.0, pelvis]) == pytest.approx([0.0, 0.0, 0.6 + CLEARANCE.lie])  # pelvis on the slot


def test_ped_is_set_down_only_while_supported() -> None:
    kin, standing = _leg()
    ped = Ped(sim_path="p", prim_path="/p", provider=None, kinematics=kin, clearance=CLEARANCE)  # type: ignore[arg-type]
    ped.teleport(np.array([1.0, 2.0, 0.0]), np.array(IDENTITY))
    assert ped.root(0.1, standing)[0] == pytest.approx([1.0, 2.0, 0.0])
    ped.support = Support(LIE_ON, 0.6)
    assert ped.root(0.1, standing)[0][2] == pytest.approx(0.6 + CLEARANCE.lie)


def test_ped_without_a_measured_clearance_rests_on_a_typical_one() -> None:
    kin, standing = _leg()
    ped = Ped(sim_path="p", prim_path="/p", provider=None, kinematics=kin)  # type: ignore[arg-type]
    ped.teleport(np.array([1.0, 2.0, 0.0]), np.array(IDENTITY))
    ped.support = Support(LIE_ON, 0.6)
    assert ped.root(0.1, standing)[0][2] == pytest.approx(0.6 + TYPICAL_CLEARANCE.lie)


def test_clearance_comes_from_the_annotation_beside_the_sdf(tmp_path) -> None:
    sdf = tmp_path / "nurse.sdf"
    assert read_pelvis_clearance(str(sdf)) is None  # no annotation
    (tmp_path / "annotation.yaml").write_text("name: nurse\n")
    assert read_pelvis_clearance(str(sdf)) is None  # built before the measurement
    (tmp_path / "annotation.yaml").write_text("name: nurse\npelvis_clearance: {sit: 0.12, lie: 0.11}\n")
    assert read_pelvis_clearance(str(sdf)) == CLEARANCE
