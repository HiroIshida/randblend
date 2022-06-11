import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, ClassVar, Generic, Optional, Type, TypeVar

import bpy
from scipy.spatial.transform import Rotation

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    ObjectDescription,
    ObjectDescriptionT,
)

BlenderObjectT = TypeVar("BlenderObjectT", bound="BlenderObject")


@dataclass
class BlenderObject(Generic[ObjectDescriptionT]):
    description_type: ClassVar[Type[ObjectDescriptionT]]  # type: ignore https://github.com/python/mypy/issues/5144
    description: ObjectDescriptionT
    texture_id: Optional[str] = None
    obj: Optional[Any] = None

    @classmethod
    def from_descriptoin(
        cls: Type[BlenderObjectT],
        description: ObjectDescriptionT,
        texture_id: Optional[str] = None,
    ) -> BlenderObjectT:
        return cls(description=description, texture_id=texture_id)

    def spawn_blender_object(self) -> Any:
        description = self.description
        obj = self._spawn_blender_object()
        obj.location = description.pose.translation
        rot = Rotation.from_quat(description.pose.orientation)
        obj.rotation_euler = tuple(rot.as_euler("zyx").tolist())
        return obj

    @abstractmethod
    def _spawn_blender_object(self) -> Any:
        pass

    @property
    def name(self) -> str:
        return self.description.name


class BlenderFileBasedObject(BlenderObject[FileBasedObjectDescription]):
    def _spawn_blender_object(self):
        description = self.description
        mesh_path = os.path.join(description.path, "visual_geometry.obj")
        bpy.ops.import_scene.obj(filepath=mesh_path)
        objs = bpy.context.selected_objects
        obj = objs[-1]
        obj.scale = description.scale
        return obj


class BlenderCubeObject(BlenderObject[CubeObjectDescription]):
    def _spawn_blender_object(self) -> Any:
        description = self.description
        bpy.ops.mesh.primitive_cube_add()
        obj = bpy.context.object
        obj.scale = description.shape
        return obj
