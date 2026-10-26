from __future__ import annotations

from isaac_utils.utils import map_offsets


def test_a_robot_frame_or_prim_takes_its_map_translation_and_others_stay_zero():
    map_offsets.register('/World/env_0/go1', 'env_0/go1/base', 'env_0/go1/odom')
    map_offsets.register('/World/env_0/jackal', 'env_0/jackal/base_link', 'env_0/jackal/odom')
    assert map_offsets.frame_translation('env_0/go1/base') == (0.0, 0.0, 0.0)
    map_offsets.set_translation('/World/env_0/go1', (7.8, 0.0, -3.0))
    assert map_offsets.frame_translation('env_0/go1/lidar') == (7.8, 0.0, -3.0)
    assert map_offsets.frame_translation('/env_0/go1/lidar') == (7.8, 0.0, -3.0)
    assert map_offsets.frame_translation('/World/env_0/go1/base') == (7.8, 0.0, -3.0)
    assert map_offsets.frame_translation('env_0/jackal/base_link') == (0.0, 0.0, 0.0)
    assert map_offsets.frame_translation('env_0/map') == (0.0, 0.0, 0.0)
    map_offsets.forget_under('/World/env_0')
    assert map_offsets.frame_translation('env_0/go1/lidar') == (0.0, 0.0, 0.0)
