"""ROS4HRI semantic-joint to CMU-bone rotation map, read from arena_people_msgs' bone_map.json.

Each wire DOF (GaitGenerator.JOINT_NAMES) and each rig.yaml driver rotates its target
bones about a baked bone-local axis, scaled per target so one DOF can spread over a
chain. Axes are measured from the contract renderer and mapped through each bone's
neutral skeleton-global rotation, signs baked to +1.0. tests/peds/test_bone_map_parity.py
regenerates the JSON when the skeleton or the contract rig changes. Axes are measured at
the rest pose, so shoulder yaw and twist blend approximately under large flexion (exact
for gait, where both stay 0).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_prefix


@dataclass(frozen=True)
class BoneTarget:
    """One semantic joint's contribution to one CMU bone's local rotation."""

    bone: str  # CMU bone name (leaf of a token in the skeleton's joint_order)
    axis: tuple[float, float, float]  # unit rotation axis, in the bone's local rest frame
    sign: float  # +1.0 or -1.0, flips rotation direction (e.g. mirrored L/R bones)
    scale: float = 1.0  # share of the semantic angle applied to this bone


def _load(path: Path) -> dict[str, tuple[BoneTarget, ...]]:
    """Bone targets per wire DOF from a bone_map.json."""
    return {dof: tuple(BoneTarget(t["bone"], tuple(t["axis"]), t["sign"], t["scale"]) for t in targets) for dof, targets in json.loads(path.read_text()).items()}


def _workspace_bone_map() -> Path:
    """The bone_map.json installed beside this arena_isaac, never the Isaac image's message bridge copy."""
    return Path(get_package_prefix("arena_isaac")).parent / "arena_people_msgs" / "share" / "arena_people_msgs" / "bone_map.json"


BONE_MAP: dict[str, tuple[BoneTarget, ...]] = _load(_workspace_bone_map())
