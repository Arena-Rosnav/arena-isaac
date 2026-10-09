from __future__ import annotations

import attrs
from pxr import UsdGeom

from isaac_utils.utils.prim import stage


@attrs.define
class _Registry:
    paths: set[str] = attrs.field(factory=set)
    forced: bool | None = None


_registry = _Registry()


def _apply() -> None:
    visible = bool(_registry.forced)
    for path in list(_registry.paths):
        prim = stage.GetPrimAtPath(path)
        if not prim:
            _registry.paths.discard(path)
            continue
        if visible:
            UsdGeom.Imageable(prim).MakeVisible()
        else:
            UsdGeom.Imageable(prim).MakeInvisible()


def register(path: str) -> None:
    """Track a ceiling prim, hidden unless ceilings are forced on."""
    _registry.paths.add(path)
    _apply()


def force(visible: bool | None) -> None:
    """Show or hide every ceiling, None restores the default (hidden)."""
    _registry.forced = visible
    _apply()
