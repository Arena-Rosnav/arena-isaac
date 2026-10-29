"""isaacsim_msgs/Wall carries the segment flags, all true by default."""

import pytest

pytest.importorskip("isaacsim_msgs.msg")

from isaacsim_msgs.msg import Wall  # noqa: E402


def test_wall_flags_default_to_true() -> None:
    wall = Wall()
    assert (wall.visible, wall.solid, wall.shadows, wall.is_static) == (True, True, True, True)


def test_wall_flags_are_settable() -> None:
    wall = Wall(visible=False, solid=False, shadows=False, is_static=False)
    assert (wall.visible, wall.solid, wall.shadows, wall.is_static) == (False, False, False, False)
