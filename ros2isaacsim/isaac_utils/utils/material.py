from __future__ import annotations

import omni
from isaac_utils.utils.path import world_path

from isaacsim_msgs.msg import Material as MaterialMsg


class Material:
    _path: str

    @classmethod
    def from_msg(cls, msg: MaterialMsg) -> Material | None:
        """
        Create a material from a MaterialMsg.
        :param msg: The msg to create the material from.
        :return: The created material, or None if the material could not be created.
        """
        return cls.load(path=msg.url, name=msg.name) if msg.url and msg.name else None

    @classmethod
    def load(cls, path: str, name: str) -> Material | None:
        """
        Load a material from an MDL path.
        :param path: The MDL path of the material to load.
        :param name: The name of the material to load.
        :return: The loaded material, or None if the material could not be loaded.
        """
        material_path = world_path('Looks', 'Material', name)

        stage = omni.usd.get_context().get_stage()
        mtl = stage.GetPrimAtPath(material_path)

        if not (mtl and mtl.IsValid()):
            if not omni.kit.commands.execute(
                'CreateMdlMaterialPrimCommand',
                mtl_url=path,
                mtl_name=name,
                mtl_path=material_path
            ):
                return None

        obj = cls()
        obj._path = material_path
        return obj

    @property
    def path(self) -> str:
        return self._path

    def bind_to(self, prim_path: str) -> bool:
        """
        Bind this material to a prim.
        :param prim_path: The path of the prim to bind the material to.
        :return: True if the material was successfully bound, False otherwise.
        """
        return omni.kit.commands.execute(
            'BindMaterialCommand',
            prim_path=prim_path,
            material_path=self._path
        )
