"""Full parse -> author pipeline tests over a synthetic in-test COLLADA doc.

No network: the synthetic DAE mirrors the structural elements of the real Fuel
files (library_geometries polylist with VERTEX/NORMAL offsets, a skin controller
with a scrambled joint order, a joint node tree, and matrix animation channels).
"""

from __future__ import annotations

import json
import math
import os
import pathlib
from collections.abc import Iterator

import numpy as np
import pytest

pytest.importorskip("pxr")

from peds.cache import actor_cache_dir, convert_cached
from peds.convert import _Collada, _parse_materials, convert_actor, parse_actor_sdf
from pxr import Gf, Usd, UsdGeom, UsdShade, UsdSkel

# Canonical joint tree J0 -> J1 -> J2. The skin lists joints scrambled to force
# the name-based remap. Constants drive both the emitted DAE and the expectations.
CANONICAL_PATHS = ("J0", "J0/J1", "J0/J1/J2")
SKIN_ORDER = ("J2", "J0", "J1")
INV_BIND_SKIN = np.stack(
    [
        np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -2], [0, 0, 0, 1]], dtype=float),  # J2
        np.eye(4),  # J0
        np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -1], [0, 0, 0, 1]], dtype=float),  # J1
    ]
)
BIND_SHAPE = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0.2], [0, 0, 1, 0.3], [0, 0, 0, 1]], dtype=float)
REST_LOCAL = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]], dtype=float)
ROOT_DRIFT = np.array([0.4, 0.3])
ROOT_BOB_Z = 0.1
MATERIAL_DIFFUSE = (0.8, 0.2, 0.1)
MATERIAL_SHININESS = 50.0
MATERIAL_ROUGHNESS_EXPECTED = math.sqrt(2.0 / (MATERIAL_SHININESS + 2.0))
TEXTURE_REL = "textures/skin.png"


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _compose(translation: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


def _fmt(matrix: np.ndarray) -> str:
    return " ".join(repr(float(x)) for x in matrix.flatten())


def _gf_to_np(matrix: Gf.Matrix4d) -> np.ndarray:
    return np.array([list(matrix.GetRow(i)) for i in range(4)], dtype=float)


def _dae_text(*, textured: bool = False, idref_joints: bool = False, root_offset: np.ndarray | None = None) -> str:
    rest = _fmt(REST_LOCAL)
    inv_bind = _fmt(INV_BIND_SKIN.reshape(-1))
    root_start = np.zeros(3) if root_offset is None else root_offset
    a0 = f"{_fmt(_compose(root_start, _rot_z(np.pi / 4)))} {_fmt(_compose(np.array([ROOT_DRIFT[0], ROOT_DRIFT[1], ROOT_BOB_Z]), np.eye(3)))}"
    a1 = f"{_fmt(_compose(np.zeros(3), np.eye(3)))} {_fmt(_compose(np.zeros(3), _rot_z(np.pi / 2)))}"
    a2 = f"{_fmt(_compose(np.zeros(3), np.eye(3)))} {_fmt(_compose(np.zeros(3), _rot_x(np.pi / 2)))}"

    def anim(name: str, target: str, output: str) -> str:
        return (
            f'<animation id="{name}">'
            f'<source id="{name}in"><float_array id="{name}ina" count="2">0 0.5</float_array>'
            f'<technique_common><accessor source="#{name}ina" count="2" stride="1"><param name="T" type="float"/></accessor></technique_common></source>'
            f'<source id="{name}out"><float_array id="{name}outa" count="32">{output}</float_array>'
            f'<technique_common><accessor source="#{name}outa" count="2" stride="16"><param name="M" type="float4x4"/></accessor></technique_common></source>'
            f'<sampler id="{name}s"><input semantic="INPUT" source="#{name}in"/><input semantic="OUTPUT" source="#{name}out"/></sampler>'
            f'<channel source="#{name}s" target="{target}/transform"/>'
            f"</animation>"
        )

    joint_array_tag = "IDREF_array" if idref_joints else "Name_array"
    joint_param_type = "IDREF" if idref_joints else "name"
    uv_source = '<source id="g-uv"><float_array id="gua" count="6">0 0 1 0 0 1</float_array><technique_common><accessor source="#gua" count="3" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>' if textured else ""
    uv_input = '<input semantic="TEXCOORD" source="#g-uv" offset="2"/>' if textured else ""
    p_text = "0 0 0 1 1 1 2 2 2" if textured else "0 0 1 1 2 2"
    if textured:
        effect = (
            '<effect id="mat-effect"><profile_COMMON>'
            '<newparam sid="skin-surface"><surface type="2D"><init_from>skin-image</init_from></surface></newparam>'
            '<newparam sid="skin-sampler"><sampler2D><source>skin-surface</source></sampler2D></newparam>'
            '<technique sid="common"><phong>'
            '<diffuse><texture texture="skin-sampler" texcoord="UVTex"/></diffuse>'
            f"<shininess><float>{MATERIAL_SHININESS}</float></shininess>"
            "</phong></technique></profile_COMMON></effect>"
        )
        images = f'<library_images><image id="skin-image"><init_from>./{TEXTURE_REL}</init_from></image></library_images>'
    else:
        effect = f'<effect id="mat-effect"><profile_COMMON><technique sid="common"><phong><diffuse><color>{MATERIAL_DIFFUSE[0]} {MATERIAL_DIFFUSE[1]} {MATERIAL_DIFFUSE[2]} 1</color></diffuse><shininess><float>{MATERIAL_SHININESS}</float></shininess></phong></technique></profile_COMMON></effect>'
        images = ""

    return f"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 {images}
 <library_geometries>
  <geometry id="g-mesh" name="g"><mesh>
   <source id="g-positions"><float_array id="gpa" count="9">0 0 0 1 0 0 0 1 0</float_array>
    <technique_common><accessor source="#gpa" count="3" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
   <source id="g-normals"><float_array id="gna" count="9">0 0 1 0 0 1 0 0 1</float_array>
    <technique_common><accessor source="#gna" count="3" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
   {uv_source}
   <vertices id="g-vertices"><input semantic="POSITION" source="#g-positions"/></vertices>
   <polylist material="m" count="1">
    <input semantic="VERTEX" source="#g-vertices" offset="0"/>
    <input semantic="NORMAL" source="#g-normals" offset="1"/>
    {uv_input}
    <vcount>3</vcount><p>{p_text}</p>
   </polylist>
  </mesh></geometry>
 </library_geometries>
 <library_effects>
  {effect}
 </library_effects>
 <library_materials>
  <material id="mat-material" name="m"><instance_effect url="#mat-effect"/></material>
 </library_materials>
 <library_controllers>
  <controller id="c" name="c"><skin source="#g-mesh">
   <bind_shape_matrix>{_fmt(BIND_SHAPE)}</bind_shape_matrix>
   <source id="c-joints"><{joint_array_tag} id="cja" count="3">{" ".join(SKIN_ORDER)}</{joint_array_tag}>
    <technique_common><accessor source="#cja" count="3" stride="1"><param name="J" type="{joint_param_type}"/></accessor></technique_common></source>
   <source id="c-binds"><float_array id="cba" count="48">{inv_bind}</float_array>
    <technique_common><accessor source="#cba" count="3" stride="16"><param name="M" type="float4x4"/></accessor></technique_common></source>
   <source id="c-weights"><float_array id="cwa" count="2">1.0 0.5</float_array>
    <technique_common><accessor source="#cwa" count="2" stride="1"><param name="W" type="float"/></accessor></technique_common></source>
   <joints><input semantic="JOINT" source="#c-joints"/><input semantic="INV_BIND_MATRIX" source="#c-binds"/></joints>
   <vertex_weights count="3">
    <input semantic="JOINT" source="#c-joints" offset="0"/>
    <input semantic="WEIGHT" source="#c-weights" offset="1"/>
    <vcount>1 2 1</vcount><v>1 0 2 1 0 1 0 0</v>
   </vertex_weights>
  </skin></controller>
 </library_controllers>
 <library_animations>
  {anim("anim0", "J0", a0)}
  {anim("anim1", "J1", a1)}
  {anim("anim2", "J2", a2)}
 </library_animations>
 <library_visual_scenes>
  <visual_scene id="Scene" name="Scene"><node id="wrap" type="NODE">
   <node id="J0" sid="J0" type="JOINT"><matrix sid="transform">{rest}</matrix>
    <node id="J1" sid="J1" type="JOINT"><matrix sid="transform">{rest}</matrix>
     <node id="J2" sid="J2" type="JOINT"><matrix sid="transform">{rest}</matrix></node>
    </node>
   </node>
   <node id="mesh-node" name="g">
    <instance_controller url="#c"><bind_material><technique_common>
      <instance_material symbol="m" target="#mat-material"/>
     </technique_common></bind_material></instance_controller>
   </node>
  </node></visual_scene>
 </library_visual_scenes>
 <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>"""


def _sdf_text(dae_uri: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
 <actor name="synth">
  <skin><filename>{dae_uri}</filename></skin>
  <animation name="walk"><filename>{dae_uri}</filename></animation>
  <animation name="idle"><filename>{dae_uri}</filename></animation>
 </actor>
</sdf>"""


@pytest.fixture()
def built(tmp_path: pathlib.Path) -> pathlib.Path:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text())
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)))
    out = tmp_path / "out"
    spec = parse_actor_sdf(str(sdf))  # exercises the required entry point
    assert set(spec) == {"walk", "idle"}
    convert_actor(str(sdf), {str(dae): str(dae)}, out)
    return out


@pytest.fixture()
def built_travelling(tmp_path: pathlib.Path) -> pathlib.Path:
    dae = tmp_path / "synth_travel.dae"
    dae.write_text(_dae_text(root_offset=np.array([-0.1, -1.9, 0.9])))
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)))
    out = tmp_path / "out"
    convert_actor(str(sdf), {str(dae): str(dae)}, out)
    return out


@pytest.fixture()
def built_textured(tmp_path: pathlib.Path, arena_data_dir: pathlib.Path) -> pathlib.Path:
    dae = tmp_path / "synth_tex.dae"
    dae.write_text(_dae_text(textured=True))
    (tmp_path / "textures").mkdir()
    (tmp_path / "textures" / "skin.png").write_bytes(b"\x89PNG\r\n\x1a\n")
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)))
    return convert_cached(str(sdf))


@pytest.fixture()
def arena_data_dir(tmp_path: pathlib.Path) -> Iterator[pathlib.Path]:
    previous = os.environ.get("ARENA_DATA_DIR")
    os.environ["ARENA_DATA_DIR"] = str(tmp_path)
    try:
        yield tmp_path
    finally:
        if previous is None:
            os.environ.pop("ARENA_DATA_DIR", None)
        else:
            os.environ["ARENA_DATA_DIR"] = previous


def test_parse_actor_sdf_maps_clip_to_uri(tmp_path: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)))
    clips = parse_actor_sdf(str(sdf))
    assert clips == {"walk": str(dae), "idle": str(dae)}


def test_joint_order_is_topological(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath("/Character/Skeleton"))
    joints = tuple(str(j) for j in skeleton.GetJointsAttr().Get())
    assert joints == CANONICAL_PATHS


def test_bind_transforms_invert_inverse_bind(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath("/Character/Skeleton"))
    bind = skeleton.GetBindTransformsAttr().Get()
    skin_of_canon = [SKIN_ORDER.index(name) for name in ("J0", "J1", "J2")]
    for canon_idx, skin_idx in enumerate(skin_of_canon):
        expected = np.linalg.inv(INV_BIND_SKIN[skin_idx]).T
        np.testing.assert_allclose(_gf_to_np(bind[canon_idx]), expected, atol=1e-9)


def test_rest_transforms_from_hierarchy(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath("/Character/Skeleton"))
    rest = skeleton.GetRestTransformsAttr().Get()
    for canon_idx in range(3):
        np.testing.assert_allclose(_gf_to_np(rest[canon_idx]), REST_LOCAL.T, atol=1e-9)


def test_skin_weights_remapped_and_normalized(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    binding = UsdSkel.BindingAPI(stage.GetPrimAtPath("/Character/Mesh"))
    indices_primvar = binding.GetJointIndicesPrimvar()
    assert indices_primvar.GetElementSize() == 2
    indices = list(indices_primvar.Get())
    weights = list(binding.GetJointWeightsPrimvar().Get())
    assert indices == [0, 0, 1, 2, 2, 0]
    np.testing.assert_allclose(weights, [1.0, 0.0, 0.5, 0.5, 1.0, 0.0], atol=1e-9)


def test_geom_bind_transform_from_bind_shape(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    binding = UsdSkel.BindingAPI(stage.GetPrimAtPath("/Character/Mesh"))
    geom = _gf_to_np(binding.GetGeomBindTransformAttr().Get())
    np.testing.assert_allclose(geom, BIND_SHAPE.T, atol=1e-9)


def test_character_structure_and_binding(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    assert stage is not None
    assert stage.GetPrimAtPath("/Character").GetTypeName() == "SkelRoot"
    assert stage.GetPrimAtPath("/Character/Skeleton").GetTypeName() == "Skeleton"
    mesh_prim = stage.GetPrimAtPath("/Character/Mesh")
    assert mesh_prim.GetTypeName() == "Mesh"
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == pytest.approx(1.0)
    binding = UsdSkel.BindingAPI(mesh_prim)
    assert [str(p) for p in binding.GetSkeletonRel().GetTargets()] == ["/Character/Skeleton"]
    assert binding.GetJointIndicesPrimvar().Get() is not None
    assert binding.GetJointWeightsPrimvar().Get() is not None


def test_parse_materials_resolves_diffuse_and_roughness(tmp_path: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text())
    materials = _parse_materials(_Collada(str(dae)))
    assert set(materials) == {"m"}
    material = materials["m"]
    assert material.name == "m"
    np.testing.assert_allclose(material.diffuse, MATERIAL_DIFFUSE, atol=1e-6)
    assert material.roughness == pytest.approx(MATERIAL_ROUGHNESS_EXPECTED, abs=1e-9)


def test_character_mesh_has_material_subset_bound_to_shader(built: pathlib.Path) -> None:
    stage = Usd.Stage.Open(str(built / "character.usda"))
    mesh_prim = stage.GetPrimAtPath("/Character/Mesh")
    subset = UsdGeom.Subset(stage.GetPrimAtPath("/Character/Mesh/m"))
    assert subset.GetPrim().IsValid()
    assert subset.GetElementTypeAttr().Get() == UsdGeom.Tokens.face
    assert subset.GetFamilyNameAttr().Get() == "materialBind"
    assert list(subset.GetIndicesAttr().Get()) == [0]
    assert UsdGeom.Subset.GetFamilyType(UsdGeom.Imageable(mesh_prim), "materialBind") == UsdGeom.Tokens.partition

    bound_material, _ = UsdShade.MaterialBindingAPI(subset.GetPrim()).ComputeBoundMaterial()
    assert str(bound_material.GetPath()) == "/Character/Materials/m"

    shader = UsdShade.Shader(stage.GetPrimAtPath("/Character/Materials/m/Shader"))
    assert shader.GetIdAttr().Get() == "UsdPreviewSurface"
    diffuse = shader.GetInput("diffuseColor").Get()
    np.testing.assert_allclose([diffuse[0], diffuse[1], diffuse[2]], MATERIAL_DIFFUSE, atol=1e-6)
    assert shader.GetInput("roughness").Get() == pytest.approx(MATERIAL_ROUGHNESS_EXPECTED, abs=1e-6)
    assert bound_material.GetSurfaceOutput().HasConnectedSource()


def test_parse_materials_resolves_diffuse_texture(tmp_path: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text(textured=True))
    materials = _parse_materials(_Collada(str(dae)))
    assert materials["m"].diffuse_texture == TEXTURE_REL


def test_textured_mesh_authors_st_and_diffuse_texture(built_textured: pathlib.Path) -> None:
    assert (built_textured / TEXTURE_REL).is_file()  # cache mirrored the skin's image
    stage = Usd.Stage.Open(str(built_textured / "character.usda"))

    mesh_prim = stage.GetPrimAtPath("/Character/Mesh")
    st = UsdGeom.PrimvarsAPI(mesh_prim).GetPrimvar("st")
    assert st.HasValue()
    assert st.GetInterpolation() == UsdGeom.Tokens.faceVarying

    shader = UsdShade.Shader(stage.GetPrimAtPath("/Character/Materials/m/Shader"))
    texture_source, texture_output, _ = shader.GetInput("diffuseColor").GetConnectedSource()
    assert texture_output == "rgb"
    texture = UsdShade.Shader(texture_source.GetPrim())
    assert texture.GetIdAttr().Get() == "UsdUVTexture"
    assert texture.GetInput("file").Get().path.endswith("skin.png")

    reader_source, reader_output, _ = texture.GetInput("st").GetConnectedSource()
    assert reader_output == "result"
    reader = UsdShade.Shader(reader_source.GetPrim())
    assert reader.GetIdAttr().Get() == "UsdPrimvarReader_float2"
    assert reader.GetInput("varname").Get() == "st"


def test_no_clips_authored(built: pathlib.Path) -> None:
    assert not (built / "clips").exists()


def test_meta_neutral_root_is_rest_others_from_idle_first_frame(built: pathlib.Path) -> None:
    meta = json.loads((built / "meta.json").read_text())
    assert meta["actor"] == "synth"
    assert meta["up_axis"] == "Z"
    assert meta["joints"] == list(CANONICAL_PATHS)
    # root pinned to rest, other joints from the identity first idle frame
    np.testing.assert_allclose(meta["neutral"]["rotations_xyzw"], [[0.0, 0.0, 0.0, 1.0]] * 3, atol=1e-9)
    np.testing.assert_allclose(meta["neutral"]["translations"], [list(REST_LOCAL[:3, 3])] + [[0.0, 0.0, 0.0]] * 2, atol=1e-9)


def test_meta_neutral_root_ignores_idle_root_travel(built_travelling: pathlib.Path) -> None:
    meta = json.loads((built_travelling / "meta.json").read_text())
    np.testing.assert_allclose(meta["neutral"]["translations"][0], REST_LOCAL[:3, 3], atol=1e-9)
    np.testing.assert_allclose(meta["neutral"]["rotations_xyzw"][0], [0.0, 0.0, 0.0, 1.0], atol=1e-9)


def test_convert_actor_requires_idle(tmp_path: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text())
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)).replace(f'<animation name="idle"><filename>{dae}</filename></animation>', ""))
    with pytest.raises(ValueError, match="no idle animation"):
        convert_actor(str(sdf), {str(dae): str(dae)}, tmp_path / "out")


def test_convert_actor_rejects_non_collada_idle(tmp_path: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text())
    bvh = tmp_path / "idle.bvh"
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)).replace(f"<filename>{dae}</filename></animation>\n </actor>", f"<filename>{bvh}</filename></animation>\n </actor>"))
    clips = parse_actor_sdf(str(sdf))
    assert clips["idle"] == str(bvh)  # the replace really rewired the idle clip
    with pytest.raises(ValueError, match="not COLLADA"):
        convert_actor(str(sdf), {str(dae): str(dae), str(bvh): str(bvh)}, tmp_path / "out")


def test_idref_joint_arrays_rejected(tmp_path: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text(idref_joints=True))
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)))
    with pytest.raises(ValueError, match="no Name_array"):
        convert_actor(str(sdf), {str(dae): str(dae)}, tmp_path / "out")


def test_cache_digest_stable_and_sensitive(tmp_path: pathlib.Path, arena_data_dir: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text())
    sdf_a = tmp_path / "a.sdf"
    sdf_a.write_text(_sdf_text(str(dae)))
    assert actor_cache_dir(str(sdf_a)) == actor_cache_dir(str(sdf_a))

    sdf_b = tmp_path / "b.sdf"
    sdf_b.write_text(_sdf_text(str(dae) + "?v=2"))
    assert actor_cache_dir(str(sdf_b)) != actor_cache_dir(str(sdf_a))


def test_cache_hit_returns_without_rebuild(tmp_path: pathlib.Path, arena_data_dir: pathlib.Path) -> None:
    dae = tmp_path / "synth.dae"
    dae.write_text(_dae_text())
    sdf = tmp_path / "actor.sdf"
    sdf.write_text(_sdf_text(str(dae)))

    first = convert_cached(str(sdf))
    assert (first / "character.usda").is_file()
    assert (first / "meta.json").is_file()
    assert (first / "ATTRIBUTION.md").is_file()
    assert "neutral" in json.loads((first / "meta.json").read_text())

    marker = first / "MARKER"
    marker.write_text("kept")
    second = convert_cached(str(sdf))
    assert second == first
    assert marker.is_file()  # a rebuild would replace the dir and drop the marker
