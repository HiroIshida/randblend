from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Set, Tuple

import yaml


@dataclass
class Dataset:
    data: Dict

    @classmethod
    def construct(cls, dataste_path: Path) -> "Dataset":
        with dataste_path.open(mode="r") as f:
            dic = yaml.safe_load(f)
        return cls(dic)

    def filter_by_categories(self, categories_q: Tuple[str, ...]) -> List[str]:
        assert isinstance(categories_q, tuple)
        keys = []
        for category_q in categories_q:
            keys.extend(self.filter_by_category(category_q))
        return keys

    def filter_by_category(self, category_q: str) -> List[str]:
        filtered_key_list = []
        for key, val in self.data.items():
            category = val["category"]
            if isinstance(category, List):
                raise NotADirectoryError()
            if category == category_q:
                filtered_key_list.append(key)
        return filtered_key_list

    def categories(self) -> Set[str]:
        cats = set()
        for key, val in self.data.items():
            cats.add(val["category"])
        return cats


if __name__ == "__main__":
    dataset = Dataset.construct(Path("./texture_dataset/metainfo.yaml"))
    keys = dataset.filter_by_categories(("Wood", "Plastic"))
    cats = dataset.categories()
    print(cats)
    print(keys)
