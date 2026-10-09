from __future__ import annotations

import math

import attrs
import carb
import omni.kit.commands as commands
from isaacsim_msgs.msg import Light, LightState
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdLux, UsdShade

from isaac_utils.utils import ceilings
from isaac_utils.utils.path import world_path
from isaac_utils.utils.prim import ensure_path, stage

DEFAULT_LIGHT = '/World/Light_1'
VIEWPORT_RIG = '/OmniKit_Viewport_LightRig'
AMBIENT_SHAPES = ('dome', 'distant')


@attrs.define
class _Registry:
    fixtures: dict[str, list[str]] = attrs.field(factory=dict)
    spawned: dict[str, Light] = attrs.field(factory=dict)
    default_released: bool = False
    demoted: set[str] = attrs.field(factory=set)
    levels: dict[str, float] = attrs.field(factory=dict)
    lit: bool = False


_registry = _Registry()


def _declared() -> bool:
    return any(not light.intrinsic for light in _registry.spawned.values())


def _sync_default() -> None:
    declared = _declared()
    target = stage.GetPrimAtPath(DEFAULT_LIGHT)
    if target:
        target.SetActive(not _registry.default_released and not declared)
    rig = stage.GetPrimAtPath(VIEWPORT_RIG)
    if rig:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            rig.SetActive(not declared)
    ceilings.lit(declared)
    lit = declared
    if lit and not _registry.lit:
        demote_emission(stage.GetPseudoRoot())
    elif _registry.lit and not lit:
        _restore_emission()
    _registry.lit = lit


def demote_emission(root: Usd.Prim) -> None:
    """Shade texture-emitting materials under root by that texture while declared lights exist."""
    if not _declared():
        return
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdShade.Shader):
                continue
            shader = UsdShade.Shader(prim)
            emissive = shader.GetInput('emissiveColor')
            if not emissive or not emissive.HasConnectedSource():
                continue
            diffuse = shader.GetInput('diffuseColor')
            if diffuse and (diffuse.HasConnectedSource() or (diffuse.Get() is not None and max(diffuse.Get()) > 0.0)):
                continue
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(emissive.GetConnectedSources()[0][0])
            emissive.GetAttr().SetConnections([])
            emissive.Set(Gf.Vec3f(0.0))
            _registry.demoted.add(str(prim.GetPath()))


def _restore_emission() -> None:
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path in _registry.demoted:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue
            for name in ('inputs:emissiveColor', 'inputs:diffuseColor'):
                attribute = prim.GetAttribute(name)
                attribute.ClearConnections()
                attribute.Clear()
    _registry.demoted.clear()


def release_default() -> None:
    """Drop the fallback dome for good."""
    _registry.default_released = True
    _sync_default()


def _define(path: str, light: Light, position: Gf.Vec3d | None) -> Usd.Prim:
    if light.shape == 'rect':
        schema = UsdLux.RectLight.Define(stage, path)
        schema.CreateWidthAttr(light.x_length)
        schema.CreateHeightAttr(light.y_length)
    elif light.shape == 'disk':
        schema = UsdLux.DiskLight.Define(stage, path)
        schema.CreateRadiusAttr(light.radius)
    elif light.shape == 'sphere':
        schema = UsdLux.SphereLight.Define(stage, path)
        schema.CreateRadiusAttr(light.radius)
    elif light.shape == 'dome':
        schema = UsdLux.DomeLight.Define(stage, path)
    elif light.shape == 'distant':
        schema = UsdLux.DistantLight.Define(stage, path)
        schema.CreateNormalizeAttr(True)
    else:
        raise ValueError(f'unknown light shape {light.shape!r}')
    schema.CreateIntensityAttr(light.intensity)
    schema.CreateEnableColorTemperatureAttr(True)
    schema.CreateColorTemperatureAttr(light.cct_k)
    prim = schema.GetPrim()
    UsdLux.ShadowAPI.Apply(prim).CreateShadowEnableAttr(light.cast_shadows)
    if light.cone_deg < 180.0:
        UsdLux.ShapingAPI.Apply(prim).CreateShapingConeAngleAttr(light.cone_deg / 2.0)
    xform = UsdGeom.Xformable(prim)
    if position is not None:
        xform.AddTranslateOp().Set(position)
    direction = Gf.Vec3d(light.direction.x, light.direction.y, light.direction.z)
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Rotation(Gf.Vec3d(0.0, 0.0, -1.0), direction).GetQuat())
    return prim


def _same_ambient(a: Light, b: Light) -> bool:
    return (a.shape, a.intensity, a.cct_k, a.direction) == (b.shape, b.intensity, b.cct_k, b.direction)


def _glow(name: str, level: float) -> None:
    light = _registry.spawned[name]
    root = stage.GetPrimAtPath(world_path(light.glow_prim)) if light.glow else None
    if not root:
        return
    material = Tf.MakeValidIdentifier(light.glow)
    color = Gf.Vec3f(*(float(channel) * level for channel in light.glow_color))
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdShade.Material) or prim.GetName() != material:
                continue
            shader = UsdShade.Material(prim).ComputeSurfaceSource()[0]
            if shader:
                shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(color)


def bind_glow() -> None:
    """Render the glow of every spawned light again, for owners that appeared after their light."""
    for name, level in _registry.levels.items():
        _glow(name, level)


def _render(name: str, level: float, alive: list[bool]) -> None:
    intensity = _registry.spawned[name].intensity * level
    for index, path in enumerate(_registry.fixtures[name]):
        live = index >= len(alive) or alive[index]
        UsdLux.LightAPI(stage.GetPrimAtPath(path)).GetIntensityAttr().Set(intensity if live else 0.0)
    _registry.levels[name] = level
    _glow(name, level)


def spawn(light: Light) -> tuple[bool, bool]:
    """Create the fixture prims of a light, the second flag marks an ambient light kept from an earlier caller."""
    ambient = light.shape in AMBIENT_SHAPES
    if light.name in _registry.fixtures:
        if ambient:
            if not _same_ambient(_registry.spawned[light.name], light):
                carb.log_warn(f'ambient light {light.name!r} already exists with other settings, keeping it')
            return True, True
        delete(light.name)
    root = world_path(light.name)
    ensure_path(root)
    paths: list[str] = []
    if ambient:
        paths.append(f'{root}/light')
        _define(paths[0], light, None)
    else:
        for index, position in enumerate(light.positions):
            paths.append(f'{root}/fixture_{index}')
            _define(paths[-1], light, Gf.Vec3d(position.x, position.y, position.z))
    _registry.fixtures[light.name] = paths
    _registry.spawned[light.name] = light
    _render(light.name, light.level, list(light.alive))
    _sync_default()
    return True, False


def _place(name: str, position: object, yaw: float) -> None:
    light = _registry.spawned[name]
    direction = Gf.Vec3d(light.direction.x, light.direction.y, light.direction.z)
    aim = Gf.Rotation(Gf.Vec3d(0.0, 0.0, -1.0), direction) * Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), math.degrees(yaw))
    translate, orient = UsdGeom.Xformable(stage.GetPrimAtPath(_registry.fixtures[name][0])).GetOrderedXformOps()[:2]
    translate.Set(Gf.Vec3d(position.x, position.y, position.z))
    orient.Set(aim.GetQuat())


def set_state(state: LightState) -> bool:
    """Move a spawned light, or render it at the given level and per-fixture alive mask."""
    if state.name not in _registry.fixtures:
        return False
    if state.move:
        _place(state.name, state.position, state.yaw)
    else:
        _render(state.name, state.level, list(state.alive))
    return True


def delete(name: str) -> bool:
    """Remove a light and its fixture prims."""
    if name in _registry.spawned:
        _glow(name, 0.0)
    _registry.levels.pop(name, None)
    _registry.fixtures.pop(name, None)
    _registry.spawned.pop(name, None)
    root = world_path(name)
    if stage.GetPrimAtPath(root):
        commands.execute('DeletePrims', paths=[root])
        stage.RemovePrim(root)
    _sync_default()
    return True
