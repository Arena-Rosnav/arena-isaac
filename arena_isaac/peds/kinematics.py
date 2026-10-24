"""Forward kinematics of an evaluated pose: where the pelvis and the feet are relative to the root.

The wire carries joint angles only, the root stays where humansim commands it and the hips keep their standing height
in skeleton space: a sit clip folds the legs up under the hips. Placing a body on a seat or a bed needs the joint
heights those angles produce: the pelvis decides where the body rests, the lowest joint where the feet are.
Skeleton space is Z-up meters with the root between the feet on the floor.
"""

from __future__ import annotations

import numpy as np
from pxr import Gf, UsdSkel, Vt

from peds.providers.base import JointPose


class Kinematics:
    """Per-skeleton FK over joint-path tokens ("Hips/LHipJoint/..."), by UsdSkel."""

    def __init__(self, joint_order: tuple[str, ...], neutral: JointPose) -> None:
        self._topology = UsdSkel.Topology(list(joint_order))
        self._scales = Vt.Vec3hArray(len(joint_order), Gf.Vec3h(1.0, 1.0, 1.0))
        self._pelvis = next(i for i, name in enumerate(joint_order) if "/" not in name)  # the skeleton's top joint is the hips
        _, self._neutral_lowest = self.heights(neutral)

    def heights(self, pose: JointPose) -> tuple[float, float]:
        """(pelvis, lowest joint) heights of the pose in skeleton space."""
        local = UsdSkel.MakeTransforms(
            Vt.Vec3fArray.FromNumpy(np.asarray(pose.translations, dtype=np.float32)),
            Vt.QuatfArray.FromNumpy(np.asarray(pose.rotations, dtype=np.float32)),  # Gf quaternions are laid out xyzw too
            self._scales,
        )
        z = np.asarray(UsdSkel.ConcatJointTransforms(self._topology, local))[:, 3, 2]  # row-vector matrices: translation row
        return float(z[self._pelvis]), float(z.min())

    def foot_lift(self, lowest: float) -> float:
        """How far a pose with this lowest joint lifts the feet above where the standing pose has them."""
        return lowest - self._neutral_lowest
