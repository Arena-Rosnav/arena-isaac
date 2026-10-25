"""Link inertials the URDF importer can turn into rigid bodies."""

from __future__ import annotations

import xml.etree.ElementTree as ET

_ROOT_MASS = 0.01
_ROOT_INERTIA = 1e-4


def _degenerate(inertial: ET.Element) -> bool | None:
    """Whether mass or inertia is not physical (zero, negative or not positive definite), None when unreadable."""
    mass_el = inertial.find('mass')
    inertia_el = inertial.find('inertia')
    if mass_el is None or inertia_el is None:
        return None
    try:
        mass = float(mass_el.attrib.get('value', '0'))
        ixx, ixy, ixz, iyy, iyz, izz = (float(inertia_el.attrib.get(key, '0')) for key in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'))
    except ValueError:
        return None
    det = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz) + ixz * (ixy * iyz - iyy * ixz)
    return mass <= 0.0 or min(ixx, iyy, izz) <= 0.0 or det <= 0.0


def prune_inertials(root: ET.Element) -> None:
    """Drop non-physical link inertials, the root link's replaced by a small valid one so it stays the articulation root."""
    children = {child.attrib.get('link') for joint in root.iter('joint') for child in joint.iter('child')}
    for link in root.iter('link'):
        inertial = link.find('inertial')
        if inertial is None or not _degenerate(inertial):
            continue
        link.remove(inertial)
        if link.attrib.get('name') in children:
            continue
        replacement = ET.SubElement(link, 'inertial')
        ET.SubElement(replacement, 'mass', {'value': f'{_ROOT_MASS}'})
        ET.SubElement(replacement, 'inertia', {'ixx': f'{_ROOT_INERTIA}', 'iyy': f'{_ROOT_INERTIA}', 'izz': f'{_ROOT_INERTIA}', 'ixy': '0', 'ixz': '0', 'iyz': '0'})


__all__ = ['prune_inertials']
