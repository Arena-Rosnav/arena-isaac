"""The viewport fov lands as a focal length that wins over the session-layer opinions Kit authors on its cameras."""

import math

import pytest
from pxr import Gf, Usd, UsdGeom

from isaac_utils.utils.camera import set_hfov

APERTURE = 20.955


def _camera(stage: Usd.Stage, layer: object) -> UsdGeom.Camera:
    with Usd.EditContext(stage, layer):
        camera = UsdGeom.Camera.Define(stage, "/OmniverseKit_Persp")
        camera.GetHorizontalApertureAttr().Set(APERTURE)
        camera.GetFocalLengthAttr().Set(18.147562)
    return camera


@pytest.mark.parametrize("authored_on", ["session", "root"])
def test_the_fov_sets_the_composed_focal_length(authored_on: str) -> None:
    stage = Usd.Stage.CreateInMemory()
    camera = _camera(stage, stage.GetSessionLayer() if authored_on == "session" else stage.GetRootLayer())
    set_hfov(camera, math.pi / 2)
    assert camera.GetFocalLengthAttr().Get() == pytest.approx(APERTURE / 2, rel=1e-6)
    assert camera.GetCamera().GetFieldOfView(Gf.Camera.FOVHorizontal) == pytest.approx(90.0, rel=1e-6)
