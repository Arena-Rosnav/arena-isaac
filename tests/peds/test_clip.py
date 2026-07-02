from __future__ import annotations

import pathlib

import numpy as np
import pytest

pytest.importorskip("pxr")

from pxr import Gf, Usd, UsdSkel

from peds.providers.clip import Clip, ClipSampler


@pytest.fixture()
def clip_path(tmp_path: pathlib.Path) -> str:
    path = str(tmp_path / "anim.usda")
    stage = Usd.Stage.CreateNew(path)
    stage.SetTimeCodesPerSecond(24.0)

    anim = UsdSkel.Animation.Define(stage, "/Anim")
    anim.CreateJointsAttr(["a", "a/b", "a/b/c"])

    rotations_attr = anim.CreateRotationsAttr()
    translations_attr = anim.CreateTranslationsAttr()

    identity = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
    quarter_turn_z = Gf.Quatf(float(np.cos(np.pi / 4.0)), Gf.Vec3f(0.0, 0.0, float(np.sin(np.pi / 4.0))))

    rotations_attr.Set([identity, identity, identity], 0.0)
    rotations_attr.Set([quarter_turn_z, quarter_turn_z, quarter_turn_z], 24.0)

    translations_attr.Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(0, 1, 0), Gf.Vec3f(0, 2, 0)], 0.0)
    translations_attr.Set([Gf.Vec3f(1, 0, 0), Gf.Vec3f(1, 1, 0), Gf.Vec3f(1, 2, 0)], 24.0)

    stage.GetRootLayer().Save()
    return path


def test_load_finds_skelanimation_by_traversal(clip_path: str) -> None:
    clip = Clip.load(clip_path)
    assert clip.joint_order == ("a", "a/b", "a/b/c")
    assert clip.times.shape == (2,)
    assert clip.rotations.shape == (2, 3, 4)
    assert clip.translations.shape == (2, 3, 3)
    assert clip.duration == pytest.approx(1.0)


def test_load_explicit_prim_path(clip_path: str) -> None:
    clip = Clip.load(clip_path, prim_path="/Anim")
    assert clip.duration == pytest.approx(1.0)


def test_load_missing_prim_path_raises(clip_path: str) -> None:
    with pytest.raises(ValueError, match="SkelAnimation"):
        Clip.load(clip_path, prim_path="/NotHere")


def test_sampler_bracket_midpoint(clip_path: str) -> None:
    clip = Clip.load(clip_path)
    sampler = ClipSampler()
    pose = sampler.sample(clip, 0.5, looping=False)
    np.testing.assert_allclose(pose.translations[0], [0.5, 0.0, 0.0], atol=1e-6)


def test_sampler_looping_wraps_phase(clip_path: str) -> None:
    clip = Clip.load(clip_path)
    sampler = ClipSampler()
    wrapped = sampler.sample(clip, 1.5, looping=True)
    direct = sampler.sample(clip, 0.5, looping=True)
    np.testing.assert_allclose(wrapped.translations, direct.translations, atol=1e-6)
    np.testing.assert_allclose(wrapped.rotations, direct.rotations, atol=1e-6)


def test_sampler_non_looping_clamps_to_last_frame(clip_path: str) -> None:
    clip = Clip.load(clip_path)
    sampler = ClipSampler()
    pose = sampler.sample(clip, 5.0, looping=False)
    np.testing.assert_allclose(pose.translations[0], [1.0, 0.0, 0.0], atol=1e-6)


def test_sampler_non_looping_clamps_to_first_frame(clip_path: str) -> None:
    clip = Clip.load(clip_path)
    sampler = ClipSampler()
    pose = sampler.sample(clip, -5.0, looping=False)
    np.testing.assert_allclose(pose.translations[0], [0.0, 0.0, 0.0], atol=1e-6)
