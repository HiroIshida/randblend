from pathlib import Path


def get_root_path() -> Path:
    path = Path("~/.randblend").expanduser()
    path.mkdir(exist_ok=True)
    return path


def get_texture_dataset_path():
    path = get_root_path() / "texture_dataset"
    path.mkdir(exist_ok=True)
    return path


def get_texture_metainfo_path():
    path = get_texture_dataset_path() / "metainfo.yaml"
    return path


def get_gso_dataset_path():
    path = get_root_path() / "gso_dataset"
    return path
