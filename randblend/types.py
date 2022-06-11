import json
import os
import queue
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Type, TypeVar

import bpy
import numpy as np
from scipy.spatial.transform import Rotation

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
        translation: Optional[np.ndarray] = None,
        orientation: Optional[Rotation] = None,
    ) -> "Pose":

        if translation is not None:
            trans = tuple(translation.tolist())
        else:
            trans = (0.0, 0.0, 0.0)

        if orientation is not None:
            ori = tuple(orientation.as_quat().tolist())
        else:
            ori = (1.0, 0.0, 0.0, 0.0)
        return cls(trans, ori)


class PhysicalObject(Dictable):
    pass


class RawDict(Dictable):
    data: Dict

    def to_dict(self) -> Dict:
        return self.data

    @classmethod
    def from_dict(cls: Type[DictableT], d: Dict) -> "RawDict":
        return cls(d)


@dataclass
class FileBasedObject(PhysicalObject):
    name: str
    path: str
    pose: Pose
    scale: Float3d
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
    ) -> "FileBasedObject":

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

        return cls(name, str(path), pose, scale, info)

    def spawn_blender_object(self):
        mesh_path = os.path.join(self.path, "visual_geometry.obj")
        bpy.ops.import_scene.obj(filepath=mesh_path)
        objs = bpy.context.selected_objects
        obj = objs[-1]
        obj.location = self.pose.translation
        obj.scale = self.scale

        rot = Rotation.from_quat(self.pose.orientation)
        obj.rotation_euler = tuple(rot.as_euler("zyx").tolist())


@dataclass
class WorldDescription(Dictable):
    dynamic_objects: Tuple[PhysicalObject]
    static_objects: Tuple[PhysicalObject]


if __name__ == "__main__":
    obj_list = []
    for i in range(10):
        pose = Pose(translation=(0, 1, 2), orientation=(3, 2, 1, 0))
        file_uuid = str(uuid.uuid4())[-6:]
        obj = FileBasedObject(file_uuid, "/tmp/{}".format(file_uuid), pose)
        obj_list.append(obj)
    wd = WorldDescription(tuple(obj_list), tuple(obj_list))
    wd_again = wd.from_json(wd.to_json())
    from IPython import embed

    embed()
    assert wd == wd_again
