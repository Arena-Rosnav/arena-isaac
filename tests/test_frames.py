from __future__ import annotations

import math

import pytest

pytest.importorskip("pxr")

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaac_utils.utils.frames import link, relative_pose


def _xform(stage: Usd.Stage, path: str, translate: tuple[float, float, float] = (0.0, 0.0, 0.0), yaw: float = 0.0, body: bool = False) -> Usd.Prim:
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
    if body:
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    return xform.GetPrim()


def _pose(translate: tuple[float, float, float], yaw: float) -> Gf.Matrix4d:
    return Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)), Gf.Vec3d(*translate))


def _compose(offset: tuple[list[float], list[float]], body: Gf.Matrix4d) -> Gf.Matrix4d:
    (tx, ty, tz), (qx, qy, qz, qw) = offset
    return Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(qw, qx, qy, qz)), Gf.Vec3d(tx, ty, tz)) * body


def _yaw(matrix: Gf.Matrix4d) -> float:
    x = matrix.TransformDir(Gf.Vec3d(1, 0, 0))
    return math.atan2(x[1], x[0])


def test_footprint_below_the_root_body_tracks_the_floor() -> None:
    stage = Usd.Stage.CreateInMemory()
    robot = _xform(stage, '/World/rbsummit')
    UsdGeom.Scope.Define(stage, '/World/rbsummit/Geometry')
    _xform(stage, '/World/rbsummit/Geometry/robot_base_footprint')
    base_link = _xform(stage, '/World/rbsummit/Geometry/robot_base_footprint/robot_base_link', (0.0, 0.0, 0.11), body=True)
    UsdPhysics.ArticulationRootAPI.Apply(base_link)
    _xform(stage, f'{base_link.GetPath()}/robot_front_right_wheel_link', (0.229, -0.235, 0.0), body=True)
    laser = _xform(stage, f'{base_link.GetPath()}/robot_top_3d_laser_link', (0.0, 0.0, 0.34), body=True)

    footprint = link(robot, 'robot_base_footprint')
    assert footprint is not None
    offset = relative_pose(laser, footprint)

    on_floor = _compose(offset, _pose((4.9, 12.0, 0.45), 1.0))
    assert tuple(on_floor.ExtractTranslation()) == pytest.approx((4.9, 12.0, 0.0), abs=1e-9)
    assert _yaw(on_floor) == pytest.approx(1.0, abs=1e-9)


def test_root_body_base_frame_follows_a_rotated_tracked_body() -> None:
    stage = Usd.Stage.CreateInMemory()
    robot = _xform(stage, '/World/jackal', (2.0, -1.0, 0.0635), 0.4)
    UsdGeom.Scope.Define(stage, '/World/jackal/Geometry')
    base_link = _xform(stage, '/World/jackal/Geometry/base_link', body=True)
    UsdPhysics.ArticulationRootAPI.Apply(base_link)
    imu = _xform(stage, '/World/jackal/Geometry/base_link/imu_link', (0.12, 0.03, 0.05), -0.7, body=True)

    base = link(robot, 'base_link')
    assert base is not None
    offset = relative_pose(imu, base)

    cache = UsdGeom.XformCache()
    authored = _compose(offset, cache.GetLocalToWorldTransform(imu))
    assert tuple(authored.ExtractTranslation()) == pytest.approx((2.0, -1.0, 0.0635), abs=1e-9)
    assert _yaw(authored) == pytest.approx(0.4, abs=1e-9)

    driven = _compose(offset, _pose((0.12, 0.03, 0.05), -0.7) * _pose((5.0, 3.0, 0.0635), 2.5))
    assert tuple(driven.ExtractTranslation()) == pytest.approx((5.0, 3.0, 0.0635), abs=1e-9)
    assert _yaw(driven) == pytest.approx(2.5, abs=1e-9)


def test_missing_link_resolves_to_none() -> None:
    stage = Usd.Stage.CreateInMemory()
    robot = _xform(stage, '/World/robot')
    _xform(stage, '/World/robot/base_link', body=True)
    assert link(robot, 'base_footprint') is None
