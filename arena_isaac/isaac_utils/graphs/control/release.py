"""Spawn hold of effort-commanded joints: their position drive keeps the spawn pose until the first effort command arrives."""

from __future__ import annotations

import math

import carb
import numpy as np
import rclpy.node
import sensor_msgs.msg

_pending: list[tuple[str, str, list[str]]] = []
_released: set[str] = set()
_subscriptions: dict[str, object] = {}


def hold(topic: str, articulation_path: str, joints: list[str]) -> None:
    """Release ``joints`` of the articulation at ``articulation_path`` on the first message on ``topic``."""
    if articulation_path not in _released and articulation_path not in _subscriptions:
        _pending.append((topic, articulation_path, joints))


def released(articulation_path: str) -> bool:
    return articulation_path in _released


def attach(node: rclpy.node.Node) -> None:
    """Subscribe the pending holds on ``node``."""
    while _pending:
        topic, path, joints = _pending.pop()
        _subscriptions[path] = node.create_subscription(sensor_msgs.msg.JointState, topic, lambda msg, path=path, joints=joints, node=node: _release(node, path, joints, msg), 10)


def _release(node: rclpy.node.Node, path: str, joints: list[str], msg: sensor_msgs.msg.JointState) -> None:
    from isaacsim.core.experimental.prims import Articulation

    if not any(math.isfinite(value) for value in msg.effort):
        return

    view = Articulation(path)
    if not view.is_physics_tensor_entity_valid():
        return
    indices = [view.dof_names.index(joint) for joint in joints if joint in view.dof_names]
    if indices:
        zeros = np.zeros((1, len(indices)), dtype=np.float32)
        view.set_dof_gains(zeros, zeros, dof_indices=indices)
    _released.add(path)
    subscription = _subscriptions.pop(path, None)
    if subscription is not None:
        node.destroy_subscription(subscription)
    carb.log_info(f"topic_bridge: released the spawn hold of {len(indices)} effort joints of {path}")
