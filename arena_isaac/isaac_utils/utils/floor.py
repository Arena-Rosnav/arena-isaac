"""Floor height under a point, for pedestrians and robots in worlds whose floor is not at z 0."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from pxr import Usd

GROUND_PATH = "/World/groundPlane"  # run_isaacsim's safety plane at z -3, a collision mesh but no floor
CELL = 0.2  # m, grid cell
FLAT = 0.85  # least upward share of a floor triangle's normal: slopes up to ~30 deg
STEP = 0.25  # m a walker steps up from its last floor: a stair riser
SLOPE = 1.0  # m a walker rises per m walked at most: stairs (~0.6) pass
CHECK = 10.0  # s between checks of the stage's collision meshes
MAX_SUB = 400  # subdivisions of one triangle at most


class FloorGrid:
    """Heights of the flat, upward facing world-frame triangles (n, 3, 3), per CELL square."""

    def __init__(self, triangles: np.ndarray) -> None:
        t = np.asarray(triangles, dtype=float).reshape(-1, 3, 3)
        normal = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        length = np.linalg.norm(normal, axis=1)
        t = t[(length > 1e-12) & (normal[:, 2] >= FLAT * length)]
        edge = np.linalg.norm(t - np.roll(t, 1, axis=1), axis=2).max(axis=1)
        subdivisions = np.clip(np.ceil(edge / (CELL / 2)), 1, MAX_SUB).astype(int)
        parts = [np.zeros((0, 3))]
        for n in np.unique(subdivisions):
            i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
            on = i + j <= n
            weights = np.stack([1 - (i[on] + j[on]) / n, i[on] / n, j[on] / n], axis=1)
            parts.append(np.einsum("kv,tvd->tkd", weights, t[subdivisions == n]).reshape(-1, 3))
        samples = np.concatenate(parts)
        keys = self._key(samples[:, 0], samples[:, 1])
        pairs = np.unique(np.stack([keys.astype(float), np.round(samples[:, 2], 2)], axis=1), axis=0)
        self._keys, self._z = pairs[:, 0].astype(np.int64), pairs[:, 1]

    @staticmethod
    def _key(x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray:
        return np.floor(np.asarray(x) / CELL).astype(np.int64) * 1_000_003 + np.floor(np.asarray(y) / CELL).astype(np.int64)

    def heights(self, x: float, y: float) -> np.ndarray:
        """Surface heights in the cell holding (x, y), ascending."""
        key = int(self._key(x, y))
        return self._z[np.searchsorted(self._keys, key, "left") : np.searchsorted(self._keys, key, "right")]


def floor_from(heights: np.ndarray, z: float, first: bool) -> float | None:
    """The highest height up to STEP above z, else for a first placement the lowest."""
    below = heights[heights <= z + STEP]
    if len(below):
        return float(below[-1])
    return float(heights[0]) if first and len(heights) else None


def climb(floor: float, found: float, moved: float) -> float:
    """The floor a walker steps onto, at most SLOPE times the distance moved above its last one."""
    return min(found, floor + SLOPE * moved)


_grid: FloorGrid | None = None
_fingerprint: tuple[str, ...] | None = None
_checked = -np.inf


def floor_under(x: float, y: float, z: float, *, first: bool = False) -> float | None:
    """Floor height under (x, y) for something last on floor z, None to keep its height."""
    global _grid, _fingerprint, _checked
    if time.monotonic() - _checked > CHECK:
        _checked = time.monotonic()
        prims = static_collision_meshes()
        fingerprint = tuple(str(prim.GetPath()) for prim in prims)
        if fingerprint != _fingerprint:
            triangles = [_triangles(prim) for prim in prims]
            _grid, _fingerprint = FloorGrid(np.concatenate(triangles) if triangles else np.zeros((0, 3, 3))), fingerprint
    return floor_from(_grid.heights(x, y), z, first) if _grid is not None else None


def static_collision_meshes(stage: Usd.Stage | None = None) -> list[Usd.Prim]:
    """Static collision mesh prims of the stage, the open one by default, outside robots and the safety plane."""
    from pxr import Sdf, UsdGeom, UsdPhysics

    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return []
    ground = Sdf.Path(GROUND_PATH)
    prims = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetPath().HasPrefix(ground):
            continue
        parent = prim
        while parent.IsValid() and not (parent.HasAPI(UsdPhysics.RigidBodyAPI) or parent.HasAPI(UsdPhysics.ArticulationRootAPI)):
            parent = parent.GetParent()
        if not parent.IsValid():
            prims.append(prim)
    return prims


def _triangles(prim: Usd.Prim) -> np.ndarray:
    """World-frame triangles (n, 3, 3) of a mesh prim."""
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh(prim)
    points, counts, indices = (np.asarray(attr.Get()) for attr in (mesh.GetPointsAttr(), mesh.GetFaceVertexCountsAttr(), mesh.GetFaceVertexIndicesAttr()))
    if not len(points) or not len(counts) or counts.max() < 3:
        return np.zeros((0, 3, 3))
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    fans = [np.stack([indices[starts[counts > k + 1]], indices[starts[counts > k + 1] + k], indices[starts[counts > k + 1] + k + 1]], axis=1) for k in range(1, int(counts.max()) - 1)]
    matrix = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
    return (points.astype(float) @ matrix[:3, :3].T + matrix[:3, 3])[np.concatenate(fans)]
