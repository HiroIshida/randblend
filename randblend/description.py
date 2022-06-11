import json
import os
import queue
import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar

from scipy.spatial.transform import Rotation

if "bpy" in sys.modules:
    import bpy
else:
    bpy = None

DictableT = TypeVar("DictableT", bound="Dictable")

Float3d = Tuple[float, float, float]
Float4d = Tuple[float, float, float, float]


@dataclass
class Dictable:
    @staticmethod
    def get_all_leaf_types() -> List[Type["Dictable"]]:
        concrete_types: List[Type] = []
        q = queue.Queue()  # type: ignore
        q.put(Dictable)
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
            if isinstance(val, Dictable):
                d[key] = val.to_dict()
            if isinstance(val, tuple) or isinstance(val, list):
                if isinstance(val[0], Dictable):
                    d[key] = tuple([e.to_dict() for e in val])
        return d

    @classmethod
    def from_dict(cls: Type[DictableT], d: Dict) -> DictableT:
        leaf_types = cls.get_all_leaf_types()
        type_table = {t.__name__: t for t in leaf_types}

        for key, val in d.items():

            if isinstance(val, dict):
                is_rawdict = "type" not in val
                if is_rawdict:
                    d[key] = val
                else:
                    t = type_table[val["type"]]
                    d[key] = t.from_dict(val)

            if isinstance(val, list) or isinstance(val, tuple):

                if isinstance(val[0], dict):
                    t = type_table[val[0]["type"]]
                    d[key] = tuple([t.from_dict(e) for e in val])
                else:
                    d[key] = tuple(val)

        d.pop("type", None)
        return cls(**d)  # type: ignore

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls: Type[DictableT], json_str: str) -> DictableT:
        return cls.from_dict(json.loads(json_str))


@dataclass
class Pose(Dictable):
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
class RawDict(Dictable):
    data: Dict

    def to_dict(self) -> Dict:
        return self.data

    @classmethod
    def from_dict(cls: Type[DictableT], d: Dict) -> "RawDict":
        return cls(d)


@dataclass
class ObjectDescription(Dictable):
    name: str
    pose: Pose


ObjectDescriptionT = TypeVar("ObjectDescriptionT", bound="ObjectDescription")


@dataclass
class CubeObjectDescription(ObjectDescription):
    shape: Float3d


@dataclass
class FileBasedObjectDescription(ObjectDescription):
    scale: Float3d
    path: str
    metadata: RawDict

    def __post_init__(self):
        path = Path(self.path)
        assert path.is_absolute()

    @classmethod
    def from_gso_path(
        cls,
        path: Path,
        pose: Optional[Pose] = None,
        scale: Optional[Float3d] = None,
        is_visual: bool = True,
    ) -> "FileBasedObjectDescription":

        path = path.expanduser()
        assert path.is_dir(), path

        json_path = path / "data.json"
        with json_path.open(mode="r") as f:
            info = json.load(f)

        name = info["id"]
        if pose is None:
            pose = Pose.identity()

        if scale is None:
            scale = (1.0, 1.0, 1.0)

        return cls(name, pose, scale, str(path), info)


@dataclass
class WorldDescription(Dictable):
    dynamic_objects: Tuple[ObjectDescription]
    static_objects: Tuple[ObjectDescription]
