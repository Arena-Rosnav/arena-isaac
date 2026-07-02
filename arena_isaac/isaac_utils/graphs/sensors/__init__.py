import abc
import os
import typing
from collections.abc import Sequence

import omni.usd
from pxr import Usd, UsdPhysics


def join_topic(base_topic: str, *parts: str) -> str:
    """Join relative topic segments onto base_topic, stripping any leading '/'.

    URDF-authored topics carry a Gazebo-side namespace prefix that expands
    empty under Isaac, so parts may arrive absolute (e.g. '/scan'); strip
    them so they nest under base_topic instead of discarding it.
    """
    return os.path.join(base_topic, *(part.lstrip('/') for part in parts))


def resolve_link_prim(robot_root: str, link_name: str, *, require_rigid_body: bool = False) -> str:
    """Resolve a URDF link name to its prim path within the robot subtree.

    The URDF importer nests links under `<robot_root>/Geometry/...`, so a flat
    `<robot_root>/<link_name>` no longer exists. Returns the first prim named
    `link_name` under the robot. When `require_rigid_body` is set (sensors that
    must attach to a physics body), a named link without RigidBodyAPI falls back
    to its nearest rigid-body ancestor, then the first rigid body in the tree.
    Falls back to the flat join when nothing matches.
    """
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(robot_root) if stage is not None else None
    if root is not None and root.IsValid():
        named = next((p for p in Usd.PrimRange(root) if p.GetName() == link_name), None)
        if named is not None:
            if not require_rigid_body or named.HasAPI(UsdPhysics.RigidBodyAPI):
                return str(named.GetPath())
            ancestor = named.GetParent()
            while ancestor is not None and ancestor.IsValid() and str(ancestor.GetPath()) != robot_root:
                if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                    return str(ancestor.GetPath())
                ancestor = ancestor.GetParent()
        if require_rigid_body:
            body = next((p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)), None)
            if body is not None:
                return str(body.GetPath())
    return os.path.join(robot_root, link_name)


class SensorBase(abc.ABC):
    @abc.abstractmethod
    def simulate(self, base_prim: str) -> typing.Any:
        ...

    @abc.abstractmethod
    def publish(self, base_topic: str) -> typing.Any:
        ...

    def paths(self) -> Sequence[str]:
        return ()

    def destroy(self) -> None:
        """Release non-prim resources (writers, render products). Prims handled by caller."""
        return None
