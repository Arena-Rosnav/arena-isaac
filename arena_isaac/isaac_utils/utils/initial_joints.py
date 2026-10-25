"""Joint positions a URDF's ros2_control block starts its joints at."""

from __future__ import annotations

import xml.etree.ElementTree as ET


def initial_positions(urdf_path: str) -> dict[str, float]:
    """Radians per joint from each position state interface's initial_value."""
    positions: dict[str, float] = {}
    for rc in ET.parse(urdf_path).getroot().iter('ros2_control'):
        for joint in rc.iter('joint'):
            name = joint.get('name')
            for state in joint.findall('state_interface'):
                param = state.find('param[@name="initial_value"]')
                if name and state.get('name') == 'position' and param is not None and param.text:
                    positions[name] = float(param.text)
    return positions


__all__ = ['initial_positions']
