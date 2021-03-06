import os
import pickle
import queue
from abc import abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import (
    Any,
    ClassVar,
    Dict,
    Generic,
    List,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
)

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
_registered_objects: Dict[str, "BlenderObject"] = {}


def get_inst_id_map() -> Dict[str, int]:
    d = {}
    for key, val in _registered_objects.items():
        d[key] = val.inst_id
    return d


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
        obj.rotation_euler = tuple(rot.as_euler("xyz").tolist())
        if self.texture is not None:
            obj.data.materials.append(bpy.data.materials[self.texture.name])
        self.obj = obj

        _registered_objects[self.name] = self
        inst_id = len(_registered_objects) - 1
        assert inst_id > -1 and inst_id < 256  # currently to output picture as np.uint8
        self.obj["inst_id"] = inst_id

    @property
    def inst_id(self) -> Optional[int]:
        assert "inst_id" in self.obj
        return self.obj["inst_id"]

    @abstractmethod
    def _spawn_blender_object(self) -> Any:
        pass

    @property
    def name(self) -> str:
        return self.description.name

    @property
    def is_spawned(self) -> bool:
        return self.obj is not None

    def set_material(self, material: FileBasedMaterial) -> None:
        assert self.texture is None
        assert not self.is_spawned
        self.texture = material

    @staticmethod
    def get_all_leaf_types() -> List[Type["BlenderObject"]]:
        concrete_types: List[Type] = []
        q = queue.Queue()  # type: ignore
        q.put(BlenderObject)
        while not q.empty():
            t: Type = q.get()
            if len(t.__subclasses__()) == 0:
                concrete_types.append(t)

            for st in t.__subclasses__():
                q.put(st)
        return list(set(concrete_types))


class BlenderFileBasedObject(BlenderObject[FileBasedObjectDescription]):
    description_type = FileBasedObjectDescription

    def _spawn_blender_object(self):
        description = self.description
        mesh_path = os.path.join(description.path, "visual_geometry.obj")
        bpy.ops.import_scene.obj(filepath=mesh_path)
        objs = bpy.context.selected_objects
        obj = objs[-1]
        scale = description.scale
        obj.scale = (scale, scale, scale)
        return obj


class BlenderCubeObject(BlenderObject[CubeObjectDescription]):
    description_type = CubeObjectDescription

    def _spawn_blender_object(self) -> Any:
        description = self.description
        bpy.ops.mesh.primitive_cube_add()
        obj = bpy.context.object
        half_extents = tuple([0.5 * e for e in description.shape])
        obj.scale = half_extents
        return obj


class BlenderWorld(Dict[str, BlenderObject]):
    @classmethod
    def from_descriptions(cls, descriptions: List[ObjectDescription]) -> "BlenderWorld":
        leaf_types = BlenderObject.get_all_leaf_types()
        table = {t.description_type: t for t in leaf_types}

        dd = {}
        for description in descriptions:
            target_type: Type[BlenderObject] = table[type(description)]
            blender_object = target_type.from_descriptoin(description)
            dd[blender_object.name] = blender_object
        return cls(dd)

    @classmethod
    def from_pickle_file(cls, filepath: Union[Path, str]):
        if isinstance(filepath, str):
            filepath = Path(filepath)
        filepath = filepath.expanduser().absolute()
        with filepath.open(mode="rb") as f:
            descriptions = pickle.load(f)
        return cls.from_descriptions(descriptions)

    def spawn_all(self):
        for obj in self.values():
            obj.spawn_blender_object()
