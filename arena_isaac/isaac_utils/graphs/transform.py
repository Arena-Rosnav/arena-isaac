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


def compute(db):
    if db.inputs.body_is_matrix:
        body_t, body_q = _decompose(db.inputs.body_matrix)
    else:
        t, q = db.inputs.body_translation, db.inputs.body_orientation
        body_t = [float(t[0]), float(t[1]), float(t[2])]
        body_q = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        if body_t == [0.0, 0.0, 0.0] and body_q == [0.0, 0.0, 0.0, 1.0]:
            # exact identity means the fabric pose is not written yet (ticks
            # before physics warmup), fall back to the authored usd pose
            body_t, body_q = _decompose(db.inputs.body_matrix)
    offset_t = [float(v) for v in db.inputs.offset_translation]
    offset_q = [float(v) for v in db.inputs.offset_orientation]
    world_gap = _rotate(body_q, offset_t)
    db.outputs.translation = [body_t[0] + world_gap[0], body_t[1] + world_gap[1], body_t[2] + world_gap[2]]
    db.outputs.quaternion = _mul(body_q, offset_q)
    return True
"""


def base_pose(graph: Graph, name: str, offset_translation: list[float], offset_orientation: list[float], body_is_matrix: bool = True) -> _Node:
    """ScriptNode: base-link world pose = the tracked body's live pose times the
    constant body->base offset. xyzw quaternions.
    The live body pose is body_matrix, or body_translation plus
    body_orientation when body_is_matrix is false."""
    node = graph.node(name, 'omni.graph.scriptnode.ScriptNode')
    node.create_attribute('inputs:body_matrix', 'matrixd[4]')
    node.create_attribute('inputs:body_translation', 'vectord[3]')
    node.create_attribute('inputs:body_orientation', 'quatd[4]')
    node.create_attribute('inputs:body_is_matrix', 'bool')
    node.create_attribute('inputs:offset_translation', 'vectord[3]')
    node.create_attribute('inputs:offset_orientation', 'quatd[4]')
    node.create_attribute('outputs:translation', 'vectord[3]')
    node.create_attribute('outputs:quaternion', 'quatd[4]')
    node.attribute('body_is_matrix', body_is_matrix)
    node.attribute('offset_translation', offset_translation)
    node.attribute('offset_orientation', offset_orientation)
    node.attribute('script', _BASE_POSE_SCRIPT)
    return node
