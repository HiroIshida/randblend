import json
from typing import Dict, List

from randblend.path import get_gso_dataset_path


def create_gso_category_table() -> Dict[str, List[str]]:
    table = {}
    for dirpath in get_gso_dataset_path().iterdir():
        if not dirpath.is_dir():
            continue
        metadata_path = dirpath / "data.json"

        with metadata_path.open(mode="r") as f:
            dic = json.load(f)
        category = dic["metadata"]["category"].replace(" ", "")
        object_name = dic["id"]

        if category not in table:
            table[category] = [object_name]
        else:
            table[category].append(object_name)
    return table
