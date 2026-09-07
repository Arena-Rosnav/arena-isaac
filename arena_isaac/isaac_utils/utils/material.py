from __future__ import annotations

import functools
import os
import re
import tempfile
from pathlib import Path

import attrs
import carb
import omni
from isaacsim_msgs.msg import Material as MaterialMsg
from isaacsim_msgs.msg import PhysicsParams as PhysicsParamsMsg
from pxr import PhysxSchema, UsdPhysics, UsdShade

from isaac_utils.utils.path import sanitize_path_component, world_path
from isaac_utils.utils.prim import ensure_path

_COMBINE_FROM_UINT: dict[int, str | None] = {
    PhysicsParamsMsg.COMBINE_DEFAULT: None,
    PhysicsParamsMsg.COMBINE_MIN: 'min',
    PhysicsParamsMsg.COMBINE_MULTIPLY: 'multiply',
    PhysicsParamsMsg.COMBINE_MAX: 'max',
}


@attrs.frozen
class PhysicsParams:
    static_friction: float
    dynamic_friction: float
    restitution: float = 0.0
    combine_mode: str | None = None


def _params_from_msg(msg: PhysicsParamsMsg) -> PhysicsParams:
    return PhysicsParams(
        static_friction=msg.static_friction,
        dynamic_friction=msg.dynamic_friction,
        restitution=msg.restitution,
        combine_mode=_COMBINE_FROM_UINT.get(msg.combine_mode),
    )


def _params_intern_key(params: PhysicsParams) -> str:
    sf = round(params.static_friction * 1000)
    df = round(params.dynamic_friction * 1000)
    r = round(params.restitution * 1000)
    cm = params.combine_mode or 'def'
    return f'msg_{sf}_{df}_{r}_{cm}'


def _apply_physics_apis(material_prim_path: str, params: PhysicsParams) -> None:
    """Apply UsdPhysics.MaterialAPI and PhysxSchema.PhysxMaterialAPI to the prim.

    Args:
        material_prim_path: Path of an existing UsdShade.Material prim.
        params: Friction, restitution, combine_mode values to write.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(material_prim_path)

    # tripwire, physics material must never carry collider/body apis
    assert not prim.HasAPI(UsdPhysics.CollisionAPI), f'physics material {material_prim_path} unexpectedly has CollisionAPI'
    assert not prim.HasAPI(UsdPhysics.RigidBodyAPI), f'physics material {material_prim_path} unexpectedly has RigidBodyAPI'

    usd_phys = UsdPhysics.MaterialAPI.Apply(prim)
    usd_phys.CreateStaticFrictionAttr().Set(params.static_friction)
    usd_phys.CreateDynamicFrictionAttr().Set(params.dynamic_friction)
    usd_phys.CreateRestitutionAttr().Set(params.restitution)

    if params.combine_mode is not None:
        physx = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        physx.CreateFrictionCombineModeAttr().Set(params.combine_mode)


class MdlPreprocessor:
    @classmethod
    def relative_to_absolute(cls, base_path: Path, relative_path: str) -> str:
        """Convert a relative MDL path to an absolute path.

        Args:
            base_path (Path): Base path to resolve relative paths against.
            relative_path (str): Relative path.

        Returns:
            str: Absolute path from the base path.
        """
        abs_path = (base_path / relative_path).resolve()
        return str(abs_path)

    @classmethod
    def preprocess_mdl(cls, mdl_path: Path) -> Path | None:
        """Preprocess an MDL file to resolve relative paths.

        Args:
            mdl_path (Path): Path to the MDL file.

        Returns:
            Path | None: Path to the preprocessed MDL file, or None if no substitutions performed or processing failed.
        """
        if not mdl_path.exists():
            return None

        base_path = mdl_path.parent

        with open(mdl_path) as f:
            mdl_content = f.read()

        replacer_fn = functools.partial(cls.relative_to_absolute, base_path)
        mdl_content, subs = re.subn(r'"(\./[^"]+)"', lambda m: f'"{replacer_fn(m.group(1))}"', mdl_content)

        if not subs:
            return None

        with tempfile.NamedTemporaryFile(delete=False, suffix=f'_{mdl_path.name}', mode='w') as tmp_file:
            tmp_file.write(mdl_content)
            return Path(tmp_file.name)


class Material:
    _path: str
    _purposes: frozenset[str]

    @classmethod
    def from_msg(cls, msg: MaterialMsg) -> Material | None:
        """Create a Material from a MaterialMsg.

        Visual fields (path, name) and the optional physics array are
        independent. Both set yields a single prim at the visual MDL location
        with physics APIs applied; only one set yields the corresponding
        single-purpose Material. Returns None if neither block is populated.

        Args:
            msg: Material msg, possibly carrying 0 or 1 PhysicsParams entries.

        Returns:
            Material with the appropriate _purposes set, or None.
        """
        has_visual = bool(msg.path and msg.name)
        physics_entries = list(msg.physics)

        if len(physics_entries) > 1:
            carb.log_warn(f'MaterialMsg carries {len(physics_entries)} PhysicsParams entries, expected at most 1, using the first.')
        has_physics = len(physics_entries) >= 1

        if not has_visual and not has_physics:
            return None

        material: Material | None = None
        if has_visual:
            material = cls.load(path=Path(msg.path), name=msg.name)
            if material is None:
                return None

        if has_physics:
            params = _params_from_msg(physics_entries[0])
            if material is not None:
                _apply_physics_apis(material._path, params)
                material._purposes = frozenset({'', 'physics'})
            else:
                material = cls.physics(
                    parent_prim_path=world_path(),
                    key=_params_intern_key(params),
                    params=params,
                )

        return material

    @classmethod
    def load(cls, path: Path, name: str) -> Material | None:
        """Load a visual MDL material at the world-scoped Looks/Material path.

        Args:
            path: MDL file path, optionally with a `::ref` suffix.
            name: MDL material name within the file.

        Returns:
            Material with _purposes = {""}, or None on creation failure.
        """
        material_path = world_path('Looks', 'Material', f'{name}_{str(hash(path))[8:16]}')

        basename, *ref = path.name.split('::', 1)

        if (processed := MdlPreprocessor.preprocess_mdl(path.parent / basename)) is not None:
            path = processed.parent / (processed.name + (f'::{ref[0]}' if ref else ''))

        stage = omni.usd.get_context().get_stage()
        mtl = stage.GetPrimAtPath(material_path)

        if not (mtl and mtl.IsValid()):
            if not omni.kit.commands.execute('CreateMdlMaterialPrimCommand', mtl_url=str(path), mtl_name=name, mtl_path=material_path):
                return None

        obj = cls()
        obj._path = material_path
        obj._purposes = frozenset({''})
        return obj

    @classmethod
    def physics(cls, parent_prim_path: str, key: str, params: PhysicsParams) -> Material:
        """Create or fetch an interned physics-only Material under a parent prim.

        The material prim lives at `<parent_prim_path>/Looks/Physics/<sanitized key>`,
        making its lifecycle follow the parent (CLAUDE.md WORLD vs INUSE layer).
        Idempotent: a second call with the same parent and key returns the
        existing prim without overwriting its attributes.

        Args:
            parent_prim_path: USD prim path whose lifecycle the material follows.
            key: Intern key, sanitized to a valid USD identifier.
            params: Friction, restitution, combine_mode to write on creation.

        Returns:
            Material with _purposes = {"physics"}.
        """
        material_path = os.path.join(
            parent_prim_path,
            'Looks',
            'Physics',
            sanitize_path_component(key),
        )

        stage = omni.usd.get_context().get_stage()
        existing = stage.GetPrimAtPath(material_path)

        if not (existing and existing.IsValid()):
            ensure_path(os.path.dirname(material_path))
            UsdShade.Material.Define(stage, material_path)
            _apply_physics_apis(material_path, params)

        obj = cls()
        obj._path = material_path
        obj._purposes = frozenset({'physics'})
        return obj

    @property
    def path(self) -> str:
        return self._path

    def bind_to(self, prim_path: str) -> bool:
        """Bind this material to a prim across every purpose it carries.

        Args:
            prim_path: USD prim path to bind to.

        Returns:
            True if every purpose bound successfully, False otherwise.
        """
        ok = True
        for purpose in sorted(self._purposes):
            kwargs: dict[str, str] = {
                'prim_path': prim_path,
                'material_path': self._path,
            }
            if purpose:
                kwargs['material_purpose'] = purpose
            result = omni.kit.commands.execute('BindMaterialCommand', **kwargs)
            ok = ok and bool(result)
        return ok
