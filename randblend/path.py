import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np


def get_root_path() -> Path:
    path = Path("~/.randblend").expanduser()
    path.mkdir(exist_ok=True)
    return path


def get_ambientcg_dataset_path():
    path = get_root_path() / "texture_dataset"
    path.mkdir(exist_ok=True)
    return path


def get_texture_metainfo_path():
    path = get_ambientcg_dataset_path() / "metainfo.yaml"
    return path


def get_gso_dataset_path():
    path = get_root_path() / "gso_dataset"
    return path


@dataclass
class TemporaryDataPaths:
    base_path: Path
    descriptions_path: Path
    predicate_path: Path
    segmentation_path: Path

    @staticmethod
    def get_project_path(project_name: str) -> Path:
        path = get_root_path() / "projects" / project_name
        path.mkdir(exist_ok=True, parents=True)
        return path

    @classmethod
    def get_project_temp_path(cls, project_name: str) -> Path:
        return cls.get_project_path(project_name) / "temp"

    @classmethod
    def get_temps(cls, project_name: str) -> List["TemporaryDataPaths"]:
        temps = []
        for subpath in cls.get_project_temp_path(project_name).iterdir():
            temps.append(TemporaryDataPaths.from_base_path(subpath))
        return temps

    @classmethod
    def get_latest_temp(cls, project_name: str) -> "TemporaryDataPaths":
        temps = cls.get_temps(project_name)
        creation_times = [temp.base_path.stat().st_mtime for temp in temps]
        idx_max = np.argmax(creation_times)
        return temps[idx_max]

    @classmethod
    def from_base_path(cls, path: Path) -> "TemporaryDataPaths":
        assert path.parent.name == "temp"
        scene_path = path / "descriptions.pkl"
        predicate_ptah = path / "predicates.pkl"
        segmentation_path = path / "segmentation.pkl"
        return cls(path, scene_path, predicate_ptah, segmentation_path)

    @classmethod
    def from_uuid(cls, project_name: str, uuid_str: str) -> "TemporaryDataPaths":
        base_path = cls.get_project_temp_path(project_name) / uuid_str
        base_path.mkdir(exist_ok=True, parents=True)
        return cls.from_base_path(base_path)

    @classmethod
    def random(cls, project_name: str) -> "TemporaryDataPaths":
        uuid_str = str(uuid.uuid4())
        return cls.from_uuid(project_name, uuid_str)
