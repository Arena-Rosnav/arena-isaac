"""Root placement of a ped supported by furniture: sitting on a seat or lying on a bed.

The wire carries a floor position, a yaw and joint angles. A sit clip folds the legs but keeps the hips at their
standing height above the root, and no clip tips a whole body onto its back. While a ped holds a sit or lie
interaction, the adapter reports how high the surface under it is (Pedestrian.support_height); `supported_root` moves
the root so the pelvis rests that character's measured clearance above the surface (annotation `pelvis_clearance`,
measured by the arena_assets human build). Pure numpy, importable without carb/omni/pxr.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from peds.providers.external import axis_angle_quat, quat_multiply

SIT_ON = 3  # humansim InteractionType values, as Pedestrian.interaction_type carries them
LIE_ON = 4
_SUPINE = axis_angle_quat((0.0, 1.0, 0.0), -math.pi / 2)  # local +x (facing) -> up, local +z (head) -> -x


@dataclass(frozen=True)
class Support:
    """Furniture a ped rests on: how (SIT_ON or LIE_ON), and the surface height above the ped's floor in m."""

    kind: int
    height: float


@dataclass(frozen=True)
class PelvisClearance:
    """Height of a character's pelvis joint above the surface it rests on, in m: seated, and lying on its back."""

    sit: float
    lie: float


# Median over the 83 characters the arena_assets human build measured (sit 0.093-0.160 m, lie 0.098-0.152 m). Stands in
# for a character whose asset was built before the measurement, within about 4 cm of its own clearance.
TYPICAL_CLEARANCE = PelvisClearance(sit=0.122, lie=0.117)


def read_pelvis_clearance(sdf_path: str) -> PelvisClearance | None:
    """The clearance in the annotation beside a character's SDF, None when the asset was built without it."""
    import yaml

    try:
        annotation = yaml.safe_load((Path(sdf_path).parent / "annotation.yaml").read_text()) or {}
    except OSError:
        return None
    clearance = annotation.get("pelvis_clearance")
    return PelvisClearance(sit=float(clearance["sit"]), lie=float(clearance["lie"])) if clearance else None


def supported_root(support: Support, clearance: PelvisClearance, position: np.ndarray, orientation: np.ndarray, pelvis: float, foot_lift: float) -> tuple[np.ndarray, np.ndarray]:
    """Root pose that sets a ped commanded to (position, orientation) on its supporting surface.

    pelvis is the pelvis height above the root in the evaluated pose, foot_lift how far that pose lifts the feet
    above where the standing pose has them (peds.kinematics).
    """
    if support.kind == LIE_ON:
        return _lie_root(position, orientation, support.height + clearance.lie, pelvis)
    return _sit_root(position, support.height + clearance.sit, pelvis, foot_lift), orientation


def _sit_root(position: np.ndarray, pelvis_z: float, pelvis: float, foot_lift: float) -> np.ndarray:
    """Pelvis `pelvis_z` m above the floor at position[2].

    The feet never go below the floor: while the clip is still folding the legs (foot_lift small) the body stays up,
    and it lowers onto the seat as the legs fold. On a seat higher than the folded legs reach, the feet hang.
    """
    out = np.array(position, dtype=float)
    floor = out[2]
    out[2] = max(floor + pelvis_z - pelvis, floor - foot_lift)
    return out


def _lie_root(position: np.ndarray, orientation: np.ndarray, pelvis_z: float, pelvis: float) -> tuple[np.ndarray, np.ndarray]:
    """On its back with the pelvis `pelvis_z` m above the floor, over `position`.

    The head points away from the commanded yaw, the feet toward it.
    """
    x, y, z, w = orientation
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    out = np.array(position, dtype=float)
    out[0] += pelvis * math.cos(yaw)  # tipped over, the root lies a pelvis height from the pelvis along the body
    out[1] += pelvis * math.sin(yaw)
    out[2] += pelvis_z
    return out, quat_multiply(np.asarray(orientation, dtype=float), _SUPINE)
