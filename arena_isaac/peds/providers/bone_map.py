"""Static ROS4HRI semantic-joint to arenian CMU-bone rotation map.

The ROS4HRI `human_description` rig (task_generator's GaitGenerator.JOINT_NAMES, 20
scalar-angle joints) and the arenian DAE skeleton (31 CMU-BVH bones) are disjoint naming
universes, this table is the bridge between them. Each axis is the bone-local rotation axis
derived from the arenian skeleton at its neutral (idle clip frame 0) pose: the desired ros4hri
world axis (body frame X forward, Y left, Z up) mapped into the bone's local frame through its
neutral skeleton-global rotation transpose, direction baked in so every sign is +1.0.
Regenerate if the arenian skeleton source changes.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BoneTarget:
    """One semantic joint's contribution to one arenian bone's local rotation."""

    bone: str  # arenian CMU bone name (leaf of a token in the skeleton's joint_order)
    axis: tuple[float, float, float]  # unit rotation axis, in the bone's local rest frame
    sign: float  # +1.0 or -1.0, flips rotation direction (e.g. mirrored L/R bones)


# Semantic joints with no sensible CMU counterpart map to None and are skipped by
# ExternalPoseProvider; every joint in GaitGenerator.JOINT_NAMES happens to have one here
# since both rigs model the same 20-ish major human DOFs.
BONE_MAP: dict[str, BoneTarget | None] = {
    "waist": BoneTarget("LowerBack", (0.9984, -0.0327, -0.0452), 1.0),  # torso forward lean (spine flexion)
    "r_head": BoneTarget("Head", (0.0499, 0.0644, 0.9967), 1.0),  # head roll (ear-to-shoulder tilt)
    "y_head": BoneTarget("Neck1", (-0.0100, 0.9624, 0.2714), 1.0),  # head yaw (look left/right), at the neck twist
    "p_head": BoneTarget("Head", (-0.9988, 0.0044, 0.0498), 1.0),  # head pitch (nod up/down)
    "l_y_shoulder": BoneTarget("LeftArm", (0.0724, 0.9964, 0.0436), 1.0),  # L shoulder yaw (arm swing across body)
    "l_p_shoulder": BoneTarget("LeftArm", (0.8777, -0.0844, 0.4717), 1.0),  # L shoulder flexion, primary arm swing
    "l_r_shoulder": BoneTarget("LeftArm", (-0.0724, -0.9964, -0.0436), 1.0),  # L shoulder roll (upper-arm axial)
    "l_elbow": BoneTarget("LeftForeArm", (0.8603, -0.0895, 0.5019), 1.0),  # L elbow flexion
    "r_y_shoulder": BoneTarget("RightArm", (0.0291, -0.9994, -0.0178), 1.0),  # R shoulder yaw
    "r_p_shoulder": BoneTarget("RightArm", (0.9496, 0.0332, -0.3116), 1.0),  # R shoulder flexion, primary arm swing
    "r_r_shoulder": BoneTarget("RightArm", (-0.0291, 0.9994, 0.0178), 1.0),  # R shoulder roll
    "r_elbow": BoneTarget("RightForeArm", (0.9309, 0.1228, -0.3441), 1.0),  # R elbow flexion
    "l_y_hip": BoneTarget("LeftUpLeg", (0.1633, 0.9788, -0.1238), 1.0),  # L hip yaw (internal/external rotation)
    "l_p_hip": BoneTarget("LeftUpLeg", (-0.0703, 0.1367, 0.9881), 1.0),  # L hip abduction (leg out to the side)
    "l_r_hip": BoneTarget("LeftUpLeg", (0.9841, -0.1527, 0.0911), 1.0),  # L hip flexion, primary leg swing
    "l_knee": BoneTarget("LeftLeg", (0.9843, -0.1750, 0.0241), 1.0),  # L knee flexion
    "r_y_hip": BoneTarget("RightUpLeg", (-0.0457, 0.9961, -0.0758), 1.0),  # R hip yaw
    "r_p_hip": BoneTarget("RightUpLeg", (0.2170, -0.0642, -0.9741), 1.0),  # R hip abduction
    "r_r_hip": BoneTarget("RightUpLeg", (0.9751, 0.0610, 0.2133), 1.0),  # R hip flexion, primary leg swing
    "r_knee": BoneTarget("RightLeg", (0.9752, -0.0099, 0.2212), 1.0),  # R knee flexion
}
