import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, TypeVar

import numpy as np

from randblend.path import get_gso_dataset_path

if "bpy" in sys.modules:
    import bpy
else:
    bpy = None


@dataclass
class Pose:
    translation: np.ndarray
    orientation: np.ndarray

    def __post_init__(self):
        assert self.translation.shape == (3,)
        assert self.orientation.shape == (4,)

    @classmethod
    def identity(cls) -> "Pose":
        return cls(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))

    @classmethod
    def create(
        cls,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> "Pose":

        if translation is None:
            translation = np.array([0.0, 0.0, 0.0])

        if orientation is None:
            orientation = np.array([0.0, 0.0, 0.0, 1.0])

        return cls(translation, orientation)


@dataclass
class Inertia:
    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    def get_diagonal(self) -> np.ndarray:
        return np.array([self.ixx, self.iyy, self.izz])

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
class ObjectDescription:
    name: str
    pose: Pose


ObjectDescriptionT = TypeVar("ObjectDescriptionT", bound="ObjectDescription")


@dataclass
class CubeObjectDescription(ObjectDescription):
    shape: np.ndarray
    mass: float = 0.0
    inertia: Inertia = Inertia.zeros()

    def __post_init__(self):
        assert self.shape.shape == (3,)

    @classmethod
    def create_floor(cls) -> "CubeObjectDescription":
        name = "floor"
        pose = Pose.create(translation=np.array([0.0, 0.0, -0.05]))
        mass = 0.0
        inertia = Inertia.zeros()
        return cls(name, pose, np.array([100.0, 100.0, 0.1]), mass, inertia)

    def sample_position_on_top(self) -> np.ndarray:
        np.testing.assert_almost_equal(
            self.pose.orientation, np.array([0, 0, 0.0, 1.0])
        )

        center = np.array(self.pose.translation)
        extent = np.array(self.shape)

        xyz_min = center - 0.5 * extent
        xy_min = xyz_min[:2]
        xy_random = xy_min + np.random.rand(2) * extent[:2]

        z_top = center[2] + extent[2]
        ret = np.array(xy_random.tolist() + [z_top])
        return ret


@dataclass
class FileBasedObjectDescription(ObjectDescription):
    scale: float  # blender can specify 3dim scaling, but pybullet can only scalar scaling
    path: str
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    metadata: Dict

    def __post_init__(self):
        path = Path(self.path)
        assert path.is_absolute()

        assert self.bbox_min.shape == (3,)
        assert self.bbox_max.shape == (3,)

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

        return cls(
            name, pose, scale, str(path), np.array(bbox_min), np.array(bbox_max), info
        )
