"""Offline COLLADA actor to USD (UsdSkel) converter. numpy + pxr, no omni/carb.

Parses the self-contained Fuel DAEs an actor SDF references and authors a
standalone ``character.usda`` (SkelRoot + Skeleton + skinned Mesh) plus one
``clips/<name>.usda`` per animation (a single SkelAnimation each). Emitted clips
load through peds.providers.clip.Clip.load. pxr is imported lazily inside the
authoring helpers so this module stays importable without pxr (e.g. for SDF
parsing and cache digests), matching peds.providers.clip.
"""

from __future__ import annotations

import json
import math
import pathlib
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from pxr import Gf

# COLLADA matrices are row-major, column-vector convention (v' = M v). USD/Gf is
# row-major, row-vector convention (v' = v M), so the USD form is the transpose.
# The rotation quaternion is the standard column-vector matrix-to-quat of the
# COLLADA rotation block, which reconstructs the same USD row-vector rotation.

_ROOT_STRIP_CLIPS = frozenset({"walk"})
_TIME_CODES_PER_SECOND = 24.0
_DISPLAY_COLOR = (0.72, 0.66, 0.60)
_SHADING_TAGS = ("phong", "lambert", "blinn")
_ROUGHNESS_BOUNDS = (0.05, 1.0)

# Bump whenever convert_actor's output changes shape, cache.py salts its digest
# with this so stale disk caches rebuild instead of serving old geometry.
CONVERTER_VERSION = 3


@dataclass(eq=False)
class ActorSpec:
    """Actor identity and its mesh/clip source URIs parsed from the SDF."""

    name: str
    skin_uri: str
    clips: dict[str, str]  # clip name -> mesh URI

    @property
    def mesh_uris(self) -> list[str]:
        """Unique source URIs (skin first, then clips) in stable order."""
        ordered: list[str] = []
        for uri in [self.skin_uri, *self.clips.values()]:
            if uri not in ordered:
                ordered.append(uri)
        return ordered


@dataclass(eq=False)
class JointNode:
    """One skeleton joint in canonical topological order."""

    name: str  # leaf sid, e.g. "LeftUpLeg"
    path: str  # full topological path, e.g. "Hips/LHipJoint/LeftUpLeg"
    parent: int  # canonical index of parent, -1 for the root
    rest: np.ndarray  # (4, 4) COLLADA local rest transform


@dataclass(eq=False)
class SkinData:
    """Skin controller: joint list, inverse bind poses, per-vertex influences."""

    joint_names: list[str]  # skin-local joint order (not topological)
    inv_bind: np.ndarray  # (J, 4, 4) COLLADA inverse bind matrices, skin order
    bind_shape: np.ndarray  # (4, 4) COLLADA bind shape matrix
    influences: list[list[tuple[int, float]]]  # per vertex: (skin joint idx, weight)


@dataclass(eq=False)
class MeshData:
    """Triangulated mesh geometry ready for USD authoring."""

    points: np.ndarray  # (N, 3) positions
    face_counts: list[int]  # all 3 after triangulation
    face_indices: list[int]  # position indices into points
    normals: np.ndarray | None  # (F, 3) faceVarying normals, or None
    uvs: np.ndarray | None  # (F, 2) faceVarying texcoords, or None
    subsets: dict[str, list[int]]  # polylist material symbol -> triangle indices


@dataclass(eq=False)
class MaterialData:
    """A resolved COLLADA material: constant diffuse plus an optional diffuse map."""

    name: str  # polylist material symbol
    diffuse: tuple[float, float, float]
    roughness: float
    diffuse_texture: str | None  # bundle-relative diffuse image path, or None


# --------------------------------------------------------------------------- #
# SDF parsing (pure xml, no pxr)
# --------------------------------------------------------------------------- #
def _resolve_uri(uri: str, sdf_dir: pathlib.Path) -> str:
    """Resolve an SDF-relative filename against the SDF's directory, Gazebo-style."""
    if uri.startswith(("http://", "https://", "file://")) or pathlib.PurePath(uri).is_absolute():
        return uri
    return str(sdf_dir / uri)


def parse_actor(sdf_path: str) -> ActorSpec:
    """Parse the actor name, skin URI and clip-name to URI map from an SDF."""
    sdf_dir = pathlib.Path(sdf_path).resolve().parent
    root = ET.parse(sdf_path).getroot()
    actor = root.find(".//actor")
    if actor is None:
        raise ValueError(f"no <actor> element in {sdf_path}")
    name = actor.get("name")
    if not name:
        raise ValueError(f"actor in {sdf_path} has no name")
    skin_el = actor.find("skin/filename")
    if skin_el is None or not skin_el.text:
        raise ValueError(f"actor {name} has no <skin><filename>")
    clips: dict[str, str] = {}
    for anim in actor.findall("animation"):
        clip_name = anim.get("name")
        filename = anim.find("filename")
        if not clip_name or filename is None or not filename.text:
            raise ValueError(f"actor {name} has an <animation> without name/filename")
        clips[clip_name] = _resolve_uri(filename.text.strip(), sdf_dir)
    return ActorSpec(name=name, skin_uri=_resolve_uri(skin_el.text.strip(), sdf_dir), clips=clips)


def parse_actor_sdf(sdf_path: str) -> dict[str, str]:
    """Map each clip name to its mesh URI (the required public entry point)."""
    return parse_actor(sdf_path).clips


# --------------------------------------------------------------------------- #
# COLLADA parsing (pure xml + numpy, no pxr)
# --------------------------------------------------------------------------- #
class _Collada:
    """Thin namespace-aware reader over one COLLADA document."""

    def __init__(self, path: str) -> None:
        self.root = ET.parse(path).getroot()
        self.ns = self.root.tag[: self.root.tag.index("}") + 1]
        self.id_map = {el.get("id"): el for el in self.root.iter() if el.get("id")}

    def q(self, *tags: str) -> str:
        return "/".join(self.ns + tag for tag in tags)

    def localname(self, el: ET.Element) -> str:
        return el.tag[len(self.ns) :]

    def by_id(self, ref: str) -> ET.Element:
        el = self.id_map.get(ref.lstrip("#"))
        if el is None:
            raise ValueError(f"COLLADA id {ref} not found")
        return el

    def read_floats(self, ref: str) -> np.ndarray:
        """Read a source float_array reshaped to (rows, accessor stride)."""
        src = self.by_id(ref)
        arr = src.find(self.ns + "float_array")
        if arr is None or not arr.text:
            raise ValueError(f"source {ref} has no float_array")
        data = np.array(arr.text.split(), dtype=float)
        accessor = src.find(self.q("technique_common", "accessor"))
        stride = int(accessor.get("stride", "1")) if accessor is not None else 1
        return data.reshape(-1, stride)

    def read_names(self, ref: str) -> list[str]:
        src = self.by_id(ref)
        arr = src.find(self.ns + "Name_array")
        if arr is None or not arr.text:
            raise ValueError(f"source {ref} has no Name_array")
        return arr.text.split()


def _parse_skeleton(doc: _Collada) -> list[JointNode]:
    """Depth-first joint tree from the visual scene, parents before children."""
    scene = doc.root.find(doc.q("library_visual_scenes", "visual_scene"))
    if scene is None:
        raise ValueError("COLLADA has no visual_scene")
    joints: list[JointNode] = []

    def recurse(node: ET.Element, parent_idx: int, parent_path: str | None) -> None:
        for child in node.findall(doc.ns + "node"):
            if child.get("type") != "JOINT":
                recurse(child, parent_idx, parent_path)
                continue
            sid = child.get("sid") or child.get("id")
            if not sid:
                raise ValueError("JOINT node without sid/id")
            matrix = child.find(doc.ns + "matrix")
            if matrix is None or not matrix.text:
                raise ValueError(f"joint {sid} has no <matrix>")
            rest = np.array(matrix.text.split(), dtype=float).reshape(4, 4)
            path = sid if parent_path is None else f"{parent_path}/{sid}"
            index = len(joints)
            joints.append(JointNode(name=sid, path=path, parent=parent_idx, rest=rest))
            recurse(child, index, path)

    for top in scene.findall(doc.ns + "node"):
        recurse(top, -1, None)
    if not joints:
        raise ValueError("no JOINT nodes in visual scene")
    return joints


def _parse_skin(doc: _Collada) -> SkinData:
    controller = doc.root.find(doc.q("library_controllers", "controller"))
    if controller is None:
        raise ValueError("COLLADA has no controller")
    skin = controller.find(doc.ns + "skin")
    if skin is None:
        raise ValueError("controller has no skin")

    bind_el = skin.find(doc.ns + "bind_shape_matrix")
    bind_shape = np.array(bind_el.text.split(), dtype=float).reshape(4, 4) if bind_el is not None and bind_el.text else np.eye(4)

    joints_el = skin.find(doc.ns + "joints")
    joint_src = inv_bind_src = None
    for inp in joints_el.findall(doc.ns + "input"):
        if inp.get("semantic") == "JOINT":
            joint_src = inp.get("source")
        elif inp.get("semantic") == "INV_BIND_MATRIX":
            inv_bind_src = inp.get("source")
    if joint_src is None or inv_bind_src is None:
        raise ValueError("skin joints missing JOINT or INV_BIND_MATRIX input")
    joint_names = doc.read_names(joint_src)
    inv_bind = doc.read_floats(inv_bind_src).reshape(-1, 4, 4)

    vertex_weights = skin.find(doc.ns + "vertex_weights")
    joint_off = weight_off = None
    weight_src = None
    for inp in vertex_weights.findall(doc.ns + "input"):
        semantic, offset = inp.get("semantic"), int(inp.get("offset"))
        if semantic == "JOINT":
            joint_off = offset
        elif semantic == "WEIGHT":
            weight_off, weight_src = offset, inp.get("source")
    if joint_off is None or weight_off is None or weight_src is None:
        raise ValueError("vertex_weights missing JOINT or WEIGHT input")
    weights = doc.read_floats(weight_src).reshape(-1)
    stride = max(joint_off, weight_off) + 1
    vcount = np.array(vertex_weights.find(doc.ns + "vcount").text.split(), dtype=int)
    v = np.array(vertex_weights.find(doc.ns + "v").text.split(), dtype=int)

    influences: list[list[tuple[int, float]]] = []
    cursor = 0
    for count in vcount:
        entry: list[tuple[int, float]] = []
        for _ in range(count):
            joint_idx = int(v[cursor + joint_off])
            weight = float(weights[int(v[cursor + weight_off])])
            entry.append((joint_idx, weight))
            cursor += stride
        influences.append(entry)
    return SkinData(joint_names=joint_names, inv_bind=inv_bind, bind_shape=bind_shape, influences=influences)


def _parse_mesh(doc: _Collada) -> MeshData:
    mesh = doc.root.find(doc.q("library_geometries", "geometry", "mesh"))
    if mesh is None:
        raise ValueError("COLLADA has no geometry mesh")

    points: np.ndarray | None = None
    face_counts: list[int] = []
    face_indices: list[int] = []
    normals_out: list[np.ndarray] = []
    uvs_out: list[np.ndarray] = []
    have_normals = False
    have_uvs = False
    subsets: dict[str, list[int]] = {}
    triangle_index = 0

    primitives = mesh.findall(doc.ns + "polylist") + mesh.findall(doc.ns + "triangles")
    for prim in primitives:
        material = prim.get("material")
        vertex_off = normal_off = texcoord_off = None
        vertex_src = normal_src = texcoord_src = None
        max_off = 0
        for inp in prim.findall(doc.ns + "input"):
            semantic, offset = inp.get("semantic"), int(inp.get("offset"))
            max_off = max(max_off, offset)
            if semantic == "VERTEX":
                vertex_off, vertex_src = offset, inp.get("source")
            elif semantic == "NORMAL":
                normal_off, normal_src = offset, inp.get("source")
            elif semantic == "TEXCOORD":
                texcoord_off, texcoord_src = offset, inp.get("source")
        if vertex_off is None or vertex_src is None:
            raise ValueError("mesh primitive without VERTEX input")
        prim_stride = max_off + 1

        if points is None:
            vertices_el = doc.by_id(vertex_src)
            position_src = next((inp.get("source") for inp in vertices_el.findall(doc.ns + "input") if inp.get("semantic") == "POSITION"), None)
            if position_src is None:
                raise ValueError("vertices element without POSITION input")
            points = doc.read_floats(position_src)
        normals = doc.read_floats(normal_src) if normal_src is not None else None
        if normals is not None:
            have_normals = True
        texcoords = doc.read_floats(texcoord_src) if texcoord_src is not None else None
        if texcoords is not None:
            have_uvs = True

        indices = np.array(prim.find(doc.ns + "p").text.split(), dtype=int).reshape(-1, prim_stride)
        if doc.localname(prim) == "polylist":
            vcount = np.array(prim.find(doc.ns + "vcount").text.split(), dtype=int)
        else:
            vcount = np.full(int(prim.get("count")), 3, dtype=int)

        row = 0
        for count in vcount:
            face = indices[row : row + count]
            row += count
            for corner in range(1, count - 1):
                for tuple_row in (face[0], face[corner], face[corner + 1]):
                    face_indices.append(int(tuple_row[vertex_off]))
                    if normals is not None and normal_off is not None:
                        normals_out.append(normals[int(tuple_row[normal_off])])
                    if texcoords is not None and texcoord_off is not None:
                        uvs_out.append(texcoords[int(tuple_row[texcoord_off])][:2])
                face_counts.append(3)
                if material is not None:
                    subsets.setdefault(material, []).append(triangle_index)
                triangle_index += 1

    if points is None:
        raise ValueError("mesh has no primitives")
    return MeshData(
        points=points,
        face_counts=face_counts,
        face_indices=face_indices,
        normals=np.array(normals_out) if have_normals else None,
        uvs=np.array(uvs_out) if have_uvs else None,
        subsets=subsets,
    )


def _parse_materials(doc: _Collada) -> dict[str, MaterialData]:
    """Resolve each polylist material symbol to a constant diffuse/roughness.

    Follows instance_material (symbol -> material id) -> instance_effect
    (-> effect id) -> profile_COMMON/technique's phong/lambert/blinn block. A
    symbol whose chain is missing any piece is omitted, not raised on.
    """
    materials: dict[str, MaterialData] = {}
    for instance in doc.root.iter(doc.ns + "instance_material"):
        symbol = instance.get("symbol")
        target = instance.get("target")
        if not symbol or not target:
            continue
        material_el = doc.id_map.get(target.lstrip("#"))
        if material_el is None:
            continue
        instance_effect = material_el.find(doc.ns + "instance_effect")
        if instance_effect is None:
            continue
        effect_url = instance_effect.get("url")
        if not effect_url:
            continue
        effect_el = doc.id_map.get(effect_url.lstrip("#"))
        if effect_el is None:
            continue
        technique = effect_el.find(doc.q("profile_COMMON", "technique"))
        if technique is None:
            continue
        shading = next((el for el in technique.iter() if doc.localname(el) in _SHADING_TAGS), None)
        if shading is None:
            continue

        diffuse_el = shading.find(doc.ns + "diffuse")
        if diffuse_el is None:
            continue
        color_el = diffuse_el.find(doc.ns + "color")
        if color_el is not None and color_el.text:
            values = [float(x) for x in color_el.text.split()]
            diffuse = (values[0], values[1], values[2])
        else:
            diffuse = _DISPLAY_COLOR

        texture_el = diffuse_el.find(doc.ns + "texture")
        diffuse_texture = _resolve_texture(doc, effect_el, texture_el) if texture_el is not None else None

        shininess_el = shading.find(doc.q("shininess", "float"))
        if shininess_el is not None and shininess_el.text:
            shininess = float(shininess_el.text)
            low, high = _ROUGHNESS_BOUNDS
            roughness = min(high, max(low, math.sqrt(2.0 / (shininess + 2.0))))
        else:
            roughness = 1.0

        materials[symbol] = MaterialData(name=symbol, diffuse=diffuse, roughness=roughness, diffuse_texture=diffuse_texture)
    return materials


def _resolve_texture(doc: _Collada, effect_el: ET.Element, texture_el: ET.Element) -> str | None:
    """Follow a <texture> through the effect's sampler/surface newparams to the
    library_images file path, returned bundle-relative with any leading './' stripped.

    Chain: texture@texture -> newparam(sampler2D)/source -> newparam(surface)/init_from
    (an image id) -> library_images image/init_from (the on-disk path). A break
    anywhere yields None so the caller falls back to the constant diffuse color.
    """
    newparams = {el.get("sid"): el for el in effect_el.iter(doc.ns + "newparam") if el.get("sid")}
    sampler_el = newparams.get(texture_el.get("texture"))
    if sampler_el is None:
        return None
    source_el = sampler_el.find(doc.q("sampler2D", "source"))
    if source_el is None or not source_el.text:
        return None
    surface_el = newparams.get(source_el.text.strip())
    if surface_el is None:
        return None
    image_ref = surface_el.find(doc.q("surface", "init_from"))
    if image_ref is None or not image_ref.text:
        return None
    image_el = doc.id_map.get(image_ref.text.strip())
    if image_el is None:
        return None
    init_from = image_el.find(doc.ns + "init_from")
    ref = init_from.find(doc.ns + "ref") if init_from is not None else None
    path = ref.text if ref is not None and ref.text else (init_from.text if init_from is not None else None)
    if not path:
        return None
    path = path.strip()
    if path.startswith(("http://", "https://", "file://")) or pathlib.PurePath(path).is_absolute():
        return None
    return path[2:] if path.startswith("./") else path


def _parse_animations(doc: _Collada) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Map joint leaf name to (times, (K, 4, 4) COLLADA local matrices)."""
    library = doc.root.find(doc.ns + "library_animations")
    if library is None:
        raise ValueError("COLLADA has no library_animations")
    channels: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for channel in library.iter(doc.ns + "channel"):
        target = channel.get("target", "")
        joint = target.split("/")[0]
        sampler = doc.by_id(channel.get("source"))
        input_src = output_src = None
        for inp in sampler.findall(doc.ns + "input"):
            if inp.get("semantic") == "INPUT":
                input_src = inp.get("source")
            elif inp.get("semantic") == "OUTPUT":
                output_src = inp.get("source")
        if input_src is None or output_src is None:
            raise ValueError(f"sampler for {target} missing INPUT or OUTPUT")
        times = doc.read_floats(input_src).reshape(-1)
        matrices = doc.read_floats(output_src).reshape(-1, 4, 4)
        channels[joint] = (times, matrices)
    return channels


# --------------------------------------------------------------------------- #
# Numpy math
# --------------------------------------------------------------------------- #
def _orthonormalize(matrix: np.ndarray) -> np.ndarray:
    """Nearest proper rotation to a 3x3 block, guarding reflection and scale."""
    u, _, vt = np.linalg.svd(matrix)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u = u.copy()
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def _matrix_to_quat(rotation: np.ndarray) -> np.ndarray:
    """Column-vector rotation matrix to a wxyz quaternion (Shepperd's method)."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] >= m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


def enforce_hemisphere_continuity(quats: np.ndarray) -> np.ndarray:
    """Negate each key so dot(q[k], q[k-1]) >= 0 per joint, for stable slerp.

    quats is (K, J, 4) in wxyz order (sign is irrelevant to the rotation).
    """
    out = quats.copy()
    for k in range(1, out.shape[0]):
        flip = np.sum(out[k] * out[k - 1], axis=-1) < 0.0
        out[k][flip] *= -1.0
    return out


def _build_clip(joints: list[JointNode], channels: dict[str, tuple[np.ndarray, np.ndarray]], clip_name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    """Decompose per-joint matrix keys into (times, wxyz quats, translations, stride, duration)."""
    times: np.ndarray | None = None
    for joint in joints:
        if joint.name in channels:
            times = channels[joint.name][0]
            break
    if times is None:
        raise ValueError(f"clip {clip_name} has no animation channels for the skeleton joints")
    key_count = len(times)
    joint_count = len(joints)

    translations = np.zeros((key_count, joint_count, 3), dtype=float)
    quats = np.zeros((key_count, joint_count, 4), dtype=float)
    for jindex, joint in enumerate(joints):
        if joint.name in channels:
            channel_times, matrices = channels[joint.name]
            if len(channel_times) != key_count:
                raise ValueError(f"joint {joint.name} in clip {clip_name} has a mismatched keyframe count")
        else:
            matrices = np.repeat(joint.rest[None], key_count, axis=0)
        for k in range(key_count):
            local = matrices[k]
            translations[k, jindex] = local[:3, 3]
            quats[k, jindex] = _matrix_to_quat(_orthonormalize(local[:3, :3]))

    times = times - times[0]
    duration = float(times[-1]) if key_count > 1 else 0.0

    stride = 0.0
    if clip_name in _ROOT_STRIP_CLIPS and key_count > 1 and duration > 0.0:
        root_index = next(i for i, joint in enumerate(joints) if joint.parent < 0)
        drift = translations[-1, root_index, :2] - translations[0, root_index, :2]
        fraction = times / duration
        translations[:, root_index, :2] -= fraction[:, None] * drift
        stride = float(np.linalg.norm(drift))

    return times, enforce_hemisphere_continuity(quats), translations, stride, duration


# --------------------------------------------------------------------------- #
# USD authoring (pxr, imported lazily)
# --------------------------------------------------------------------------- #
def _gf_matrix(collada: np.ndarray) -> Gf.Matrix4d:
    from pxr import Gf

    usd = np.ascontiguousarray(collada.T, dtype=float)
    return Gf.Matrix4d(*(float(x) for x in usd.flatten()))


def _skin_arrays(joints: list[JointNode], skin: SkinData) -> tuple[list[int], list[float], int]:
    """Remap influences to canonical joints, pad to max count, renormalize weights."""
    name_to_canon = {joint.name: i for i, joint in enumerate(joints)}
    canon_of_skin = [name_to_canon.get(name, -1) for name in skin.joint_names]
    element_size = max((len(entry) for entry in skin.influences), default=1)
    element_size = max(element_size, 1)

    indices: list[int] = []
    weights: list[float] = []
    for entry in skin.influences:
        row_indices: list[int] = []
        row_weights: list[float] = []
        for skin_idx, weight in entry:
            canon = canon_of_skin[skin_idx]
            if canon < 0:
                continue
            row_indices.append(canon)
            row_weights.append(weight)
        while len(row_indices) < element_size:
            row_indices.append(0)
            row_weights.append(0.0)
        row = np.array(row_weights[:element_size], dtype=float)
        total = row.sum()
        if total > 0.0:
            row = row / total
        indices.extend(int(x) for x in row_indices[:element_size])
        weights.extend(float(x) for x in row)
    return indices, weights, element_size


def _author_character(out_dir: pathlib.Path, joints: list[JointNode], skin: SkinData, mesh: MeshData, materials: dict[str, MaterialData]) -> None:
    from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdShade, UsdSkel, Vt

    stage = Usd.Stage.CreateNew(str(out_dir / "character.usda"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    skel_root = UsdSkel.Root.Define(stage, "/Character")
    stage.SetDefaultPrim(skel_root.GetPrim())

    skeleton = UsdSkel.Skeleton.Define(stage, "/Character/Skeleton")
    skeleton.CreateJointsAttr(Vt.TokenArray([joint.path for joint in joints]))

    name_to_skin = {name: i for i, name in enumerate(skin.joint_names)}
    bind: list[Gf.Matrix4d] = []
    rest: list[Gf.Matrix4d] = []
    for joint in joints:
        rest.append(_gf_matrix(joint.rest))
        if joint.name in name_to_skin:
            world = np.linalg.inv(skin.inv_bind[name_to_skin[joint.name]])
        else:
            world = _world_rest(joints, joint)
        bind.append(_gf_matrix(world))
    skeleton.CreateBindTransformsAttr(Vt.Matrix4dArray(bind))
    skeleton.CreateRestTransformsAttr(Vt.Matrix4dArray(rest))

    mesh_prim = UsdGeom.Mesh.Define(stage, "/Character/Mesh")
    mesh_prim.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in mesh.points.tolist()]))
    mesh_prim.CreateFaceVertexCountsAttr(Vt.IntArray(mesh.face_counts))
    mesh_prim.CreateFaceVertexIndicesAttr(Vt.IntArray(mesh.face_indices))
    low = mesh.points.min(axis=0).tolist()
    high = mesh.points.max(axis=0).tolist()
    mesh_prim.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*low), Gf.Vec3f(*high)]))
    mesh_prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*_DISPLAY_COLOR)]))
    if mesh.normals is not None:
        mesh_prim.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*normal) for normal in mesh.normals.tolist()]))
        mesh_prim.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    if mesh.uvs is not None:
        st = UsdGeom.PrimvarsAPI(mesh_prim).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set(Vt.Vec2fArray([Gf.Vec2f(float(uv[0]), float(uv[1])) for uv in mesh.uvs.tolist()]))

    binding = UsdSkel.BindingAPI.Apply(mesh_prim.GetPrim())
    binding.CreateSkeletonRel().SetTargets([Sdf.Path("/Character/Skeleton")])
    indices, weights, element_size = _skin_arrays(joints, skin)
    binding.CreateJointIndicesPrimvar(False, element_size).Set(Vt.IntArray(indices))
    binding.CreateJointWeightsPrimvar(False, element_size).Set(Vt.FloatArray(weights))
    binding.CreateGeomBindTransformAttr(_gf_matrix(skin.bind_shape))

    total_faces = len(mesh.face_counts)
    covered_faces = 0
    subset_count = 0
    for symbol, face_indices in mesh.subsets.items():
        material_data = materials.get(symbol)
        if material_data is None:
            continue
        name = Tf.MakeValidIdentifier(symbol)
        material = UsdShade.Material.Define(stage, f"/Character/Materials/{name}")
        shader = UsdShade.Shader.Define(stage, f"/Character/Materials/{name}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(material_data.roughness)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        diffuse_input = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        if material_data.diffuse_texture is not None and mesh.uvs is not None and (out_dir / material_data.diffuse_texture).is_file():
            reader = UsdShade.Shader.Define(stage, f"/Character/Materials/{name}/stReader")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            texture = UsdShade.Shader.Define(stage, f"/Character/Materials/{name}/DiffuseTexture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f"./{material_data.diffuse_texture}"))
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
            texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            texture.CreateInput("fallback", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*material_data.diffuse, 1.0))
            texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            diffuse_input.ConnectToSource(texture.ConnectableAPI(), "rgb")
        else:
            diffuse_input.Set(Gf.Vec3f(*material_data.diffuse))

        subset = UsdGeom.Subset.CreateGeomSubset(mesh_prim, name, UsdGeom.Tokens.face, Vt.IntArray(face_indices), "materialBind")
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
        covered_faces += len(face_indices)
        subset_count += 1

    if subset_count:
        family_type = UsdGeom.Tokens.partition if covered_faces == total_faces else UsdGeom.Tokens.nonOverlapping
        UsdGeom.Subset.SetFamilyType(mesh_prim, "materialBind", family_type)

    stage.GetRootLayer().Save()


def _world_rest(joints: list[JointNode], joint: JointNode) -> np.ndarray:
    """Accumulate a joint's COLLADA world rest transform up the parent chain."""
    world = joint.rest
    parent = joint.parent
    while parent >= 0:
        world = joints[parent].rest @ world
        parent = joints[parent].parent
    return world


def _author_clip(path: pathlib.Path, joint_paths: list[str], times: np.ndarray, quats: np.ndarray, translations: np.ndarray, stride: float, duration: float) -> None:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt

    stage = Usd.Stage.CreateNew(str(path))
    stage.SetTimeCodesPerSecond(_TIME_CODES_PER_SECOND)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    anim = UsdSkel.Animation.Define(stage, "/Anim")
    stage.SetDefaultPrim(anim.GetPrim())
    anim.CreateJointsAttr(Vt.TokenArray(joint_paths))
    rotations_attr = anim.CreateRotationsAttr()
    translations_attr = anim.CreateTranslationsAttr()

    key_count = len(times)
    joint_count = len(joint_paths)
    timecodes = times * _TIME_CODES_PER_SECOND
    for k in range(key_count):
        code = float(timecodes[k])
        rotations_attr.Set(Vt.QuatfArray([Gf.Quatf(float(quats[k, j, 0]), Gf.Vec3f(float(quats[k, j, 1]), float(quats[k, j, 2]), float(quats[k, j, 3]))) for j in range(joint_count)]), code)
        translations_attr.Set(Vt.Vec3fArray([Gf.Vec3f(float(translations[k, j, 0]), float(translations[k, j, 1]), float(translations[k, j, 2])) for j in range(joint_count)]), code)
    anim.CreateScalesAttr(Vt.Vec3hArray([Gf.Vec3h(1.0, 1.0, 1.0) for _ in range(joint_count)]))
    if key_count:
        stage.SetStartTimeCode(float(timecodes[0]))
        stage.SetEndTimeCode(float(timecodes[-1]))

    prim = anim.GetPrim()
    prim.CreateAttribute("arena:strideLength", Sdf.ValueTypeNames.Float).Set(float(stride))
    prim.CreateAttribute("arena:duration", Sdf.ValueTypeNames.Float).Set(float(duration))

    stage.GetRootLayer().Save()


# --------------------------------------------------------------------------- #
# Top-level conversion
# --------------------------------------------------------------------------- #
def convert_actor(sdf_path: str, dae_paths: dict[str, str], out_dir: str | pathlib.Path) -> ActorSpec:
    """Author character.usda, clips/<name>.usda and meta.json into out_dir.

    dae_paths maps each SDF mesh URI to a local .dae file. The skeleton, skin and
    mesh come from the skin DAE, each clip's channels from its own DAE.
    """
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    spec = parse_actor(sdf_path)

    skin_doc = _Collada(dae_paths[spec.skin_uri])
    joints = _parse_skeleton(skin_doc)
    skin = _parse_skin(skin_doc)
    mesh = _parse_mesh(skin_doc)
    materials = _parse_materials(skin_doc)
    _author_character(out, joints, skin, mesh, materials)

    joint_paths = [joint.path for joint in joints]
    clips_dir = out / "clips"
    clips_dir.mkdir(exist_ok=True)

    meta: dict[str, object] = {
        "actor": spec.name,
        "up_axis": "Z",
        "meters_per_unit": 1.0,
        "fps": _TIME_CODES_PER_SECOND,
        "joints": joint_paths,
        "clips": {},
    }
    clip_meta: dict[str, dict[str, float]] = {}
    for clip_name, uri in spec.clips.items():
        channels = _parse_animations(_Collada(dae_paths[uri]))
        times, quats, translations, stride, duration = _build_clip(joints, channels, clip_name)
        _author_clip(clips_dir / f"{clip_name}.usda", joint_paths, times, quats, translations, stride, duration)
        clip_meta[clip_name] = {"stride_length_m": stride, "duration_s": duration}
    meta["clips"] = clip_meta
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    return spec
