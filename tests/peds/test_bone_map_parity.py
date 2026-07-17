"""BONE_MAP parity against the ros4hri contract rig.

Contract side: human-tpl.xacro rendered through xacro, wire values passed through
rviz_utils.hri.rig.semantic_to_rig (the same path the panel FK preview and rviz
use). Isaac side: ExternalPoseProvider over the roster-neutral skeleton snapshot
in fixtures/skeleton_neutral.json (extracted from a converted bundle's meta.json).

For every wire DOF one probe angle is applied and the world-frame delta rotation
of the anatomically corresponding segment is measured on both rigs, expressed in
each rig's own body frame (X forward, Y left, Z up, derived from its geometry:
up from the spine, forward from the toes). Parity means matching axis, matching
magnitude, and no rotation leaking to the mirrored side.

Script mode: `python3 test_bone_map_parity.py report` prints the per-DOF table,
`python3 test_bone_map_parity.py regen` prints a BONE_MAP regenerated from the
measured contract axes (needs xacro + human_description + rviz_utils, so run
in-container).
"""
from __future__ import annotations

import json
import math
import shutil
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest

from peds.providers.bone_map import BONE_MAP
from peds.providers.external import ExternalPoseProvider

_FIXTURE = Path(__file__).parent / "fixtures" / "skeleton_neutral.json"

# (wire DOF, probe angle inside JOINTS.md limits, observed CMU bone)
_PROBES: tuple[tuple[str, float, str], ...] = (
    ("waist", 0.5, "Spine1"),
    ("r_head", 0.5, "Head"),
    ("y_head", 0.5, "Head"),
    ("p_head", 0.5, "Head"),
    ("l_p_shoulder", 0.5, "LeftArm"),
    ("l_elbow", 0.8, "LeftForeArm"),
    ("r_p_shoulder", 0.5, "RightArm"),
    ("r_elbow", 0.8, "RightForeArm"),
    ("l_y_hip", 0.4, "LeftUpLeg"),
    ("l_p_hip", 0.5, "LeftUpLeg"),
    ("l_r_hip", 0.5, "LeftUpLeg"),
    ("l_knee", -0.8, "LeftLeg"),
    ("r_y_hip", 0.4, "RightUpLeg"),
    ("r_p_hip", 0.5, "RightUpLeg"),
    ("r_r_hip", 0.5, "RightUpLeg"),
    ("r_knee", -0.8, "RightLeg"),
)
# semantic_to_rig overrides these with a constant rig posture, the contract renders them as no-ops
_RESERVED: tuple[str, ...] = ("l_y_shoulder", "l_r_shoulder", "r_y_shoulder", "r_r_shoulder")
_ALL_DOFS: tuple[str, ...] = tuple(name for name, _, _ in _PROBES) + _RESERVED


def _quat_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _axis_angle_mat(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-9 or angle == 0.0:
        return np.eye(3)
    x, y, z = a / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    )


def _rpy_mat(rpy: tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    return _axis_angle_mat((0, 0, 1), y) @ _axis_angle_mat((0, 1, 0), p) @ _axis_angle_mat((1, 0, 0), r)


def _delta_axis_angle(r_zero: np.ndarray, r_probe: np.ndarray) -> tuple[np.ndarray | None, float]:
    rd = r_probe @ r_zero.T
    cos = min(1.0, max(-1.0, (np.trace(rd) - 1.0) / 2.0))
    angle = math.acos(cos)
    if angle < 1e-6:
        return None, 0.0
    axis = np.array([rd[2, 1] - rd[1, 2], rd[0, 2] - rd[2, 0], rd[1, 0] - rd[0, 1]]) / (2.0 * math.sin(angle))
    return axis / np.linalg.norm(axis), angle


# ---------------------------------------------------------------- CMU side


@lru_cache(maxsize=1)
def _skeleton() -> tuple[tuple[str, ...], np.ndarray, np.ndarray]:
    data = json.loads(_FIXTURE.read_text())
    joints = tuple(str(j) for j in data["joints"])
    rot = np.asarray(data["neutral"]["rotations_xyzw"], dtype=float)
    trans = np.asarray(data["neutral"]["translations"], dtype=float)
    return joints, rot, trans


def _cmu_frames(wire: dict[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Leaf bone name -> (world R, world t) for the provider-rendered wire pose."""
    joints, neutral_rot, neutral_trans = _skeleton()
    provider = ExternalPoseProvider(joints, neutral_rot, neutral_trans, blend_s=0.0)
    names = list(wire)
    provider.push(0.0, names, [wire[n] for n in names])
    pose = provider.evaluate(0.1, 1.0)

    frames: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    by_path: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for i, path in enumerate(joints):
        local_r = _quat_to_mat(pose.rotations[i])
        local_t = pose.translations[i]
        parent = path.rsplit("/", 1)[0] if "/" in path else None
        if parent is None:
            world_r, world_t = local_r, local_t.copy()
        else:
            pr, pt = by_path[parent]
            world_r = pr @ local_r
            world_t = pt + pr @ local_t
        by_path[path] = (world_r, world_t)
        frames[path.rsplit("/", 1)[-1]] = (world_r, world_t)
    return frames


def _cmu_body_frame(frames: dict[str, tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """Columns X forward, Y left, Z up, from spine direction and toe direction."""
    pos = {name: t for name, (_, t) in frames.items()}
    z = pos["Spine1"] - pos["Hips"]
    z = z / np.linalg.norm(z)
    fwd = (pos["LeftToeBase"] - pos["LeftFoot"]) + (pos["RightToeBase"] - pos["RightFoot"])
    x = fwd - z * float(fwd @ z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


# ------------------------------------------------------------ contract side


@lru_cache(maxsize=1)
def _contract() -> tuple[object, object, str, dict[str, object]]:
    """(fk module, semantic_to_rig, root link, joints by name), or raises ImportError/RuntimeError."""
    import human_steering.fk as fk
    from rviz_utils.hri.rig import semantic_to_rig

    if shutil.which("xacro") is None:
        raise RuntimeError("xacro binary unavailable")
    root, joints = fk.parse_urdf(fk.render_urdf())
    return fk, semantic_to_rig, root, joints


def _urdf_frames(wire: dict[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Link name -> (world R, world t) for the rig-adapted wire pose."""
    fk, semantic_to_rig, root, joints = _contract()
    names = list(wire)
    rig_values = semantic_to_rig(names, [wire[n] for n in names])
    angles = {f"{bare}_{fk.PREVIEW_ID}": value for bare, value in zip(names, rig_values, strict=True)}

    by_parent: dict[str, list] = {}
    for joint in joints.values():
        by_parent.setdefault(joint.parent, []).append(joint)
    frames: dict[str, tuple[np.ndarray, np.ndarray]] = {root: (np.eye(3), np.zeros(3))}
    stack = [root]
    while stack:
        parent = stack.pop()
        pr, pt = frames[parent]
        for joint in by_parent.get(parent, ()):
            world_t = pt + pr @ np.asarray(joint.xyz)
            world_r = pr @ _rpy_mat(joint.rpy)
            if joint.kind in ("revolute", "continuous"):
                world_r = world_r @ _axis_angle_mat(joint.axis, angles.get(joint.name, 0.0))
            frames[joint.child] = (world_r, world_t)
            stack.append(joint.child)
    return frames


def _urdf_link_for(dof: str) -> str:
    fk, _, _, joints = _contract()
    return joints[f"{dof}_{fk.PREVIEW_ID}"].child


# ----------------------------------------------------------- measurements


def _zero_wire() -> dict[str, float]:
    return dict.fromkeys(_ALL_DOFS, 0.0)


def _measure(dof: str, probe: float, bone: str) -> dict[str, object]:
    """Signed delta axes (positive-value convention) in each rig's body frame."""
    wire = _zero_wire()
    wire[dof] = probe
    sign = 1.0 if probe >= 0 else -1.0

    cmu_zero = _cmu_frames(_zero_wire())
    cmu_probe = _cmu_frames(wire)
    body = _cmu_body_frame(cmu_zero)
    cmu_axis, cmu_angle = _delta_axis_angle(cmu_zero[bone][0], cmu_probe[bone][0])
    cmu_axis_body = (body.T @ cmu_axis) * sign if cmu_axis is not None else None

    urdf_zero = _urdf_frames(_zero_wire())
    urdf_probe = _urdf_frames(wire)
    link = _urdf_link_for(dof)
    urdf_axis, urdf_angle = _delta_axis_angle(urdf_zero[link][0], urdf_probe[link][0])
    urdf_axis_body = urdf_axis * sign if urdf_axis is not None else None  # URDF world frame is the body frame

    mirrored = bone.replace("Left", "@").replace("Right", "Left").replace("@", "Right")
    leak = 0.0
    if mirrored != bone:
        _, leak = _delta_axis_angle(cmu_zero[mirrored][0], cmu_probe[mirrored][0])

    return {
        "dof": dof,
        "probe": probe,
        "urdf_axis": urdf_axis_body,
        "urdf_angle": urdf_angle,
        "cmu_axis": cmu_axis_body,
        "cmu_angle": cmu_angle,
        "leak": leak,
    }


# ------------------------------------------------------------------ tests


def test_probe_table_covers_bone_map() -> None:
    assert set(_ALL_DOFS) == set(BONE_MAP)


def test_left_bones_are_on_the_left() -> None:
    frames = _cmu_frames(_zero_wire())
    body = _cmu_body_frame(frames)
    left = frames["LeftUpLeg"][1] - frames["Hips"][1]
    assert float(left @ body[:, 1]) > 0.0, "CMU Left* bones sit on the anatomical right, rig is mirrored"


def _require_contract() -> None:
    try:
        _contract()
    except (ImportError, RuntimeError) as error:
        pytest.skip(f"contract rig unavailable: {error!r}")


@pytest.mark.parametrize(("dof", "probe", "bone"), _PROBES)
def test_dof_parity(dof: str, probe: float, bone: str) -> None:
    _require_contract()
    m = _measure(dof, probe, bone)
    assert m["urdf_axis"] is not None, f"{dof}: contract rig did not move"
    assert m["cmu_axis"] is not None, f"{dof}: isaac rig did not move"
    dot = float(np.dot(m["urdf_axis"], m["cmu_axis"]))
    assert dot > 0.98, f"{dof}: axis mismatch, urdf {m['urdf_axis']} vs cmu {m['cmu_axis']} (dot {dot:.3f})"
    assert abs(m["cmu_angle"] - abs(probe)) < 0.1 * abs(probe) + 0.05, (
        f"{dof}: magnitude {m['cmu_angle']:.3f} vs probe {abs(probe):.3f}"
    )
    assert m["leak"] < 0.02, f"{dof}: mirrored side moved by {m['leak']:.3f}"


@pytest.mark.parametrize("dof", _RESERVED)
def test_reserved_dofs_are_noops(dof: str) -> None:
    wire = _zero_wire()
    wire[dof] = 1.0
    zero, probe = _cmu_frames(_zero_wire()), _cmu_frames(wire)
    moved = max(_delta_axis_angle(zero[b][0], probe[b][0])[1] for b in zero)
    assert moved < 1e-6, f"{dof} is reserved (contract renders it as a no-op) but moved the isaac rig by {moved:.3f}"


# ------------------------------------------------------------ script mode


def _report() -> None:
    def fmt(v: np.ndarray | None) -> str:
        return "-" if v is None else "({:+.2f},{:+.2f},{:+.2f})".format(*v)

    print(f"{'dof':<14}{'probe':>7}  {'urdf axis (body)':<21}{'cmu axis (body)':<21}{'dot':>6}{'cmu ang':>8}{'leak':>7}")
    for dof, probe, bone in _PROBES:
        m = _measure(dof, probe, bone)
        dot = float(np.dot(m["urdf_axis"], m["cmu_axis"])) if m["urdf_axis"] is not None and m["cmu_axis"] is not None else float("nan")
        print(
            f"{dof:<14}{probe:>7.2f}  {fmt(m['urdf_axis']):<21}{fmt(m['cmu_axis']):<21}{dot:>6.2f}{m['cmu_angle']:>8.3f}{m['leak']:>7.3f}"
        )
    for dof in _RESERVED:
        wire = _zero_wire()
        wire[dof] = 1.0
        zero, probe_frames = _cmu_frames(_zero_wire()), _cmu_frames(wire)
        moved = max(_delta_axis_angle(zero[b][0], probe_frames[b][0])[1] for b in zero)
        print(f"{dof:<14}{'resvd':>7}  contract no-op, cmu moved {moved:.3f}")


def _regen() -> None:
    """Print BONE_MAP entries rebuilt from the measured contract axes."""
    cmu_zero = _cmu_frames(_zero_wire())
    body = _cmu_body_frame(cmu_zero)
    print("BONE_MAP = {")
    for dof in _ALL_DOFS:
        targets = BONE_MAP[dof]
        if dof in _RESERVED or targets is None:
            print(f"    {dof!r}: None,  # reserved, contract renders as no-op")
            continue
        probe, bone = next((p, b) for n, p, b in _PROBES if n == dof)
        m = _measure(dof, probe, bone)
        world_axis = body @ m["urdf_axis"]
        entries = []
        for target in targets:
            bone_world_r = cmu_zero[target.bone][0]
            local = bone_world_r.T @ world_axis
            local = local / np.linalg.norm(local)
            scale = "" if target.scale == 1.0 else f", {target.scale}"
            entries.append(f"BoneTarget({target.bone!r}, ({local[0]:.4f}, {local[1]:.4f}, {local[2]:.4f}), 1.0{scale})")
        joined = " ".join(f"{e}," for e in entries)
        print(f"    {dof!r}: ({joined}),")
    print("}")


if __name__ == "__main__":
    {"regen": _regen}.get(sys.argv[1] if len(sys.argv) > 1 else "report", _report)()
