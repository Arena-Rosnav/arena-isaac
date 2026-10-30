"""Viewport camera attributes, authored on the session layer like the camera's pose."""

from __future__ import annotations

import math

from pxr import Usd, UsdGeom


def set_hfov(camera: UsdGeom.Camera, fov: float) -> None:
    """Set the horizontal field of view in radians as the focal length that gives it over the camera's aperture."""
    stage = camera.GetPrim().GetStage()
    aperture = float(camera.GetHorizontalApertureAttr().Get())
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera.GetFocalLengthAttr().Set(aperture / (2.0 * math.tan(fov / 2.0)))
