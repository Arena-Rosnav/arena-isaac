from __future__ import annotations

import numpy as np
import pytest

from peds.providers.clip import Clip
from peds.providers.gait import (
    CLIP_MAP,
    CURIOUS,
    IDLE,
    PANIC,
    RUNNING,
    SURPRISED,
    THREATENING,
    WALKING,
    GaitProvider,
)


def _make_clip(duration: float, offset: float) -> Clip:
    times = np.array([0.0, duration])
    rotations = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (2, 1, 1)).astype(float)
    translations = np.tile(np.array([offset, 0.0, 0.0]), (2, 1, 1)).astype(float)
    return Clip(joint_order=("j0",), times=times, rotations=rotations, translations=translations, duration=duration)


@pytest.fixture()
def clips() -> dict[str, Clip]:
    return {
        "idle": _make_clip(1.0, offset=0.0),
        "walk": _make_clip(1.0, offset=1.0),
        "run": _make_clip(0.5, offset=2.0),
    }


def test_clip_map_covers_all_states() -> None:
    for state in (IDLE, WALKING, RUNNING, PANIC, SURPRISED, CURIOUS, THREATENING):
        assert state in CLIP_MAP


def test_set_state_maps_walking(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(WALKING, speed=1.0)
    assert gp.clip_key == "walk"


def test_set_state_panic_falls_back_to_run(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(PANIC, speed=1.0)
    assert gp.clip_key == "run"


def test_set_state_surprised_falls_back_to_idle(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(WALKING, speed=1.0)
    gp.set_state(SURPRISED, speed=0.0)
    assert gp.clip_key == "idle"


def test_set_state_curious_falls_back_to_idle(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(CURIOUS, speed=0.0)
    assert gp.clip_key == "idle"


def test_set_state_threatening_falls_back_to_walk(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(THREATENING, speed=0.5)
    assert gp.clip_key == "walk"


def test_set_state_missing_clip_falls_back_to_idle() -> None:
    gp = GaitProvider({"idle": _make_clip(1.0, offset=0.0)}, stride_length=1.0)
    gp.set_state(RUNNING, speed=2.0)
    assert gp.clip_key == "idle"


def test_advance_walk_scales_by_distance_over_stride(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=2.0)
    gp.set_state(WALKING, speed=1.0)
    gp.advance(distance=1.0, dt=0.1)
    # walk clip duration 1.0, stride_length 2.0 -> phase += 1.0 * (1.0 / 2.0)
    assert gp.phase == pytest.approx(0.5)


def test_advance_idle_uses_dt_not_distance(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=2.0)
    gp.advance(distance=5.0, dt=0.2)
    assert gp.phase == pytest.approx(0.2)


def test_advance_run_scales_by_its_own_duration(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(RUNNING, speed=3.0)
    gp.advance(distance=1.0, dt=0.05)
    assert gp.phase == pytest.approx(0.5)


def test_evaluate_blends_from_previous_at_switch_start(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(WALKING, speed=1.0)  # idle -> walk, blend just started
    pose = gp.evaluate(sim_time=0.0, dt=0.0)
    np.testing.assert_allclose(pose.translations[0], [0.0, 0.0, 0.0], atol=1e-9)


def test_evaluate_blends_midway_through_window(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(WALKING, speed=1.0)  # idle -> walk
    gp.advance(distance=0.0, dt=0.125)  # half the 0.25s blend window
    pose = gp.evaluate(sim_time=0.0, dt=0.0)
    np.testing.assert_allclose(pose.translations[0], [0.5, 0.0, 0.0], atol=1e-6)


def test_evaluate_after_blend_window_is_pure_active_clip(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(WALKING, speed=1.0)  # idle -> walk
    gp.advance(distance=0.0, dt=0.3)  # past the 0.25s blend window
    pose = gp.evaluate(sim_time=0.0, dt=0.0)
    np.testing.assert_allclose(pose.translations[0], [1.0, 0.0, 0.0], atol=1e-6)


def test_reset_phase_clears_blend_state(clips: dict[str, Clip]) -> None:
    gp = GaitProvider(clips, stride_length=1.0)
    gp.set_state(WALKING, speed=1.0)
    gp.reset_phase()
    pose = gp.evaluate(sim_time=0.0, dt=0.0)
    np.testing.assert_allclose(pose.translations[0], [1.0, 0.0, 0.0], atol=1e-6)
