"""map -> odom translations of the robots in a stacked world, by the TF frame prefix of each robot."""

from __future__ import annotations

_offsets: dict[str, tuple[str, tuple[float, float, float]]] = {}


def register(robot_prim: str, base_frame_id: str, odom_frame_id: str) -> None:
    """Remember the TF frame prefix a robot's frames share, its translation zero."""
    common = ''
    for a, b in zip(base_frame_id, odom_frame_id, strict=False):
        if a != b:
            break
        common += a
    _offsets[robot_prim] = (common[: common.rfind('/') + 1], (0.0, 0.0, 0.0))


def set_translation(robot_prim: str, translation: tuple[float, float, float]) -> None:
    prefix, _ = _offsets.get(robot_prim, ('', (0.0, 0.0, 0.0)))
    _offsets[robot_prim] = (prefix, translation)


def forget_under(path: str) -> None:
    """Drop every robot at or under path."""
    for robot_prim in [prim for prim in _offsets if prim == path or prim.startswith(f'{path}/')]:
        del _offsets[robot_prim]


def frame_translation(entity: str) -> tuple[float, float, float]:
    """map -> odom translation of the robot a TF frame or prim path belongs to, zero for anything else."""
    for robot_prim, (prefix, translation) in _offsets.items():
        if entity == robot_prim or entity.startswith(f'{robot_prim}/') or (prefix and entity.lstrip('/').startswith(prefix)):
            return translation
    return (0.0, 0.0, 0.0)
