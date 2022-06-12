import copy
import json
import math
import queue
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Type, TypeVar

from pydantic.dataclasses import dataclass

from randblend.path import get_gso_dataset_path

if "bpy" in sys.modules:
    import bpy
else:
    bpy = None

DictableT = TypeVar("DictableT", bound="DictableMixIn")

Float3d = Tuple[float, float, float]
Float4d = Tuple[float, float, float, float]


@dataclass
class DictableMixIn:
    @staticmethod
    def get_all_leaf_types() -> List[Type["DictableMixIn"]]:
        concrete_types: List[Type] = []
        q = queue.Queue()  # type: ignore
        q.put(DictableMixIn)
        while not q.empty():
            t: Type = q.get()
            if len(t.__subclasses__()) == 0:
                concrete_types.append(t)

            for st in t.__subclasses__():
                q.put(st)
        return list(set(concrete_types))

    def to_dict(self) -> Dict:
        # NOTE: instead of asdict, the following doe sshallow-conversion to dict
        d = {key: self.__dict__[key] for key in self.__dataclass_fields__}
        d["type"] = self.__class__.__name__

        for key, val in d.items():
            if isinstance(val, DictableMixIn):
                d[key] = val.to_dict()
            if isinstance(val, tuple) or isinstance(val, list):
                if isinstance(val[0], DictableMixIn):
                    d[key] = tuple([e.to_dict() for e in val])
        return d

    @classmethod
    def from_dict(cls: Type[DictableT], d: Dict) -> DictableT:
        leaf_types = cls.get_all_leaf_types()
        type_table = {t.__name__: t for t in leaf_types}

        for key, val in d.items():

            if isinstance(val, dict):
                t = type_table[val["type"]]
                d[key] = t.from_dict(val)

            if isinstance(val, list) or isinstance(val, tuple):

                def convert(e):
                    if isinstance(e, dict):
                        t = type_table[e["type"]]
                        return t.from_dict(e)
                    else:
                        return e

                d[key] = tuple([convert(e) for e in val])

        d.pop("type", None)
        return cls(**d)  # type: ignore

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def from_json(cls: Type[DictableT], json_str: str) -> DictableT:
        return cls.from_dict(json.loads(json_str))


@dataclass
class Pose(DictableMixIn):
    translation: Float3d
    orientation: Float4d

    @classmethod
    def identity(cls) -> "Pose":
        return cls((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))

    @classmethod
    def create(
        cls,
        translation: Optional[Float3d] = None,
        orientation: Optional[Float4d] = None,
    ) -> "Pose":

        if translation is None:
            translation = (0.0, 0.0, 0.0)

        if orientation is None:
            orientation = (1.0, 0.0, 0.0, 0.0)

        assert isinstance(translation, tuple)
        assert isinstance(orientation, tuple)

        return cls(translation, orientation)


@dataclass
class Inertia(DictableMixIn):
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    def get_diagonal(self) -> Float3d:
        return (self.ixx, self.iyy, self.izz)

    @classmethod
    def from_mass(cls, mass: float) -> "Inertia":
        # sphere tensor
        wood_density_g_cm = 0.4  # e.g. hinoki
        wood_density_g_m = wood_density_g_cm * 100 ** 3
        wood_density_kg_m = wood_density_g_m * 0.001
        r = (3 * mass / (4 * math.pi * wood_density_kg_m)) ** (1 / 3.0)

        iner = 2.0 / 5.0 * mass * r ** 2
        return cls(iner, 0.0, 0.0, iner, 0.0, iner)

    @classmethod
    def zeros(cls) -> "Inertia":
        return cls(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class RawDict(DictableMixIn):
    data: Dict

    def to_dict(self) -> Dict:
        d = copy.deepcopy(self.data)
        d["type"] = self.__class__.__name__
        return d

    @classmethod
    def from_dict(cls: Type[DictableT], d: Dict) -> "RawDict":
        d.pop("type", None)
        return cls(d)


@dataclass
class ObjectDescription(DictableMixIn):
    name: str
    pose: Pose


ObjectDescriptionT = TypeVar("ObjectDescriptionT", bound="ObjectDescription")


@dataclass
class CubeObjectDescription(ObjectDescription):
    shape: Float3d
    mass: float = 0.0
    inertia: Inertia = Inertia.zeros()

    @classmethod
    def create_floor(cls) -> "CubeObjectDescription":
        name = "floor"
        pose = Pose.create(translation=(0.0, 0.0, -0.05))
        mass = 0.0
        inertia = Inertia.zeros()
        return cls(name, pose, (100.0, 100.0, 0.1), mass, inertia)


@dataclass
class FileBasedObjectDescription(ObjectDescription):
    scale: float  # blender can specify 3dim scaling, but pybullet can only scalar scaling
    path: str
    bbox_min: Float3d
    bbox_max: Float3d
    metadata: RawDict

    def __post_init__(self):
        path = Path(self.path)
        assert path.is_absolute()

    @classmethod
    def from_gso_name(cls, name: str, pose: Optional[Pose] = None, scale: float = 1.0):
        gso_path = get_gso_dataset_path() / name
        return cls.from_gso_path(gso_path, pose=pose, scale=scale)

    @classmethod
    def from_gso_path(
        cls, path: Path, pose: Optional[Pose] = None, scale: float = 1.0
    ) -> "FileBasedObjectDescription":

        path = path.expanduser()
        assert path.is_dir(), path

        json_path = path / "data.json"
        with json_path.open(mode="r") as f:
            info = json.load(f)

        name = info["id"]
        if pose is None:
            pose = Pose.identity()

        bbox_min, bbox_max = info["kwargs"]["bounds"]

        return cls(name, pose, scale, str(path), bbox_min, bbox_max, RawDict(info))


@dataclass
class WorldDescription(DictableMixIn):
    descriptions: Tuple[ObjectDescription, ...]
