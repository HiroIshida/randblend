import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar, Generic, Optional, Set, Tuple, Type, TypeVar

import bpy
from scipy.spatial.transform import Rotation

import randblend.utils as utils
from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    ObjectDescription,
    ObjectDescriptionT,
)
from randblend.path import get_ambientcg_dataset_path

BlenderObjectT = TypeVar("BlenderObjectT", bound="BlenderObject")

OptionalPath = Optional[Path]


_registered_materials: Set[str] = set()


@dataclass
class FileBasedMaterial:
    name: str
    color: OptionalPath = None
    metalness: OptionalPath = None
    roughness: OptionalPath = None
    normalgl: OptionalPath = None
    displacement: OptionalPath = None
    ambientocclusion: OptionalPath = None
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    displacement_scale: float = 1.0

    def __post_init__(self):
        mat = utils.add_material(self.name, use_nodes=True, make_node_tree_empty=True)

        def convert(path: OptionalPath) -> str:
            if isinstance(path, Path):
                return str(path)
            else:
                return ""

        utils.build_pbr_textured_nodes(
            mat.node_tree,
            color_texture_path=convert(self.color),
            roughness_texture_path=convert(self.roughness),
            normal_texture_path=convert(self.normalgl),
            metallic_texture_path=convert(self.metalness),
            displacement_texture_path=convert(self.displacement),
            ambient_occlusion_texture_path=convert(self.ambientocclusion),
            scale=self.scale,
            displacement_scale=self.displacement_scale,
        )

    @classmethod
    def from_ambientcg_id(cls, texture_id: str, *args, **kwargs) -> "FileBasedMaterial":
        dataset_path = get_ambientcg_dataset_path()
        resolution = "_2K"  # TODO(load from yaml??"
        path = dataset_path / (texture_id + resolution)
        assert path.is_dir()
        return cls.from_ambientcg_path(path, *args, **kwargs)

    @classmethod
    def from_ambientcg_path(
        cls, base_dir_path: Path, *args, **kwargs
    ) -> "FileBasedMaterial":
        base_dir_path = base_dir_path.expanduser()
        data_id = base_dir_path.name
        extension = "jpg"  # TODO(HiroIshida) png

        postfix_list = [
            "Color",
            "Metalness",
            "Roughness",
            "NormalGL",
            "Displacement",
            "AmbientOcclusion",
        ]

        for postfix in postfix_list:
            path = base_dir_path / "{}_{}.{}".format(data_id, postfix, extension)
            if not path.exists():
                path = None
            kwargs[postfix.lower()] = path

        return cls(name=data_id, **kwargs)


@dataclass
class BlenderObject(Generic[ObjectDescriptionT]):
    description_type: ClassVar[Type]
    description: ObjectDescriptionT
    texture: Optional[FileBasedMaterial] = None
    obj: Optional[Any] = None

    @classmethod
    def from_descriptoin(
        cls: Type[BlenderObjectT],
        description: ObjectDescriptionT,
        texture: Optional[FileBasedMaterial] = None,
    ) -> BlenderObjectT:
        return cls(description=description, texture=texture)

    def spawn_blender_object(self) -> Any:
        description = self.description
        obj = self._spawn_blender_object()
        obj.location = description.pose.translation
        rot = Rotation.from_quat(description.pose.orientation)
        obj.rotation_euler = tuple(rot.as_euler("zyx").tolist())
        if self.texture is not None:
            obj.data.materials.append(bpy.data.materials[self.texture.name])
        return obj

    @abstractmethod
    def _spawn_blender_object(self) -> Any:
        pass

    @property
    def name(self) -> str:
        return self.description.name


class BlenderFileBasedObject(BlenderObject[FileBasedObjectDescription]):
    description_tpe = FileBasedObjectDescription

    def _spawn_blender_object(self):
        description = self.description
        mesh_path = os.path.join(description.path, "visual_geometry.obj")
        bpy.ops.import_scene.obj(filepath=mesh_path)
        objs = bpy.context.selected_objects
        obj = objs[-1]
        obj.scale = description.scale
        return obj


class BlenderCubeObject(BlenderObject[CubeObjectDescription]):
    description_type = CubeObjectDescription

    def _spawn_blender_object(self) -> Any:
        description = self.description
        bpy.ops.mesh.primitive_cube_add()
        obj = bpy.context.object
        obj.scale = description.shape
        return obj
