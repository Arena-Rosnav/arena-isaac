from isaac_utils.graphs import Graph, _Node

_BASE_POSE_SCRIPT = """
import numpy as np
from pxr import Gf


def _decompose(matrix_values):
    m = Gf.Matrix4d(*np.asarray(matrix_values, dtype=float).reshape(-1).tolist())
    t = m.ExtractTranslation()
    q = m.RemoveScaleShear().ExtractRotationQuat()
    im = q.GetImaginary()
    return [t[0], t[1], t[2]], [im[0], im[1], im[2], q.GetReal()]


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def _rotate(q, v):
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return [
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    ]


def setup(db):
    db.per_instance_state.ready = False


def compute(db):
    body_t, body_q = _decompose(db.inputs.body_matrix)
    if not db.per_instance_state.ready:
        base_t, base_q = _decompose(db.inputs.base_matrix)
        conj = [-body_q[0], -body_q[1], -body_q[2], body_q[3]]
        gap = [base_t[0] - body_t[0], base_t[1] - body_t[1], base_t[2] - body_t[2]]
        db.per_instance_state.offset_q = _mul(conj, base_q)
        db.per_instance_state.offset_t = _rotate(conj, gap)
        db.per_instance_state.ready = True
    world_gap = _rotate(body_q, db.per_instance_state.offset_t)
    db.outputs.translation = [body_t[0] + world_gap[0], body_t[1] + world_gap[1], body_t[2] + world_gap[2]]
    db.outputs.quaternion = _mul(body_q, db.per_instance_state.offset_q)
    return True
"""


def base_pose(graph: Graph, name: str) -> _Node:
    """ScriptNode: base-link world pose = the tracked body's live pose times the
    constant body->base offset, captured on the first tick. xyzw quaternions."""
    node = graph.node(name, 'omni.graph.scriptnode.ScriptNode')
    node.create_attribute('inputs:body_matrix', 'matrixd[4]')
    node.create_attribute('inputs:base_matrix', 'matrixd[4]')
    node.create_attribute('outputs:translation', 'vectord[3]')
    node.create_attribute('outputs:quaternion', 'quatd[4]')
    node.attribute('script', _BASE_POSE_SCRIPT)
    return node
