import json

from randblend.gso_dataset import get_all_gso_names
from randblend.path import get_gso_dataset_path

flat_object_key_wards = [
    "Asus",
    "Snacks",
    "DishTowl",
    "Crayons",
    "Launchers",
    "DVD",
    "Cartridge",
    "Canon",
    "Board_Game",
    "Crayola",
    "Final_Fantasy",
    "Gigabyte",
    "Cardboard",
    "Acting_Game",
    "WashCloth",
    "Tablet",
    "Just_For_Men",
    "DVD",
    "Game",
    "LEGO",
    "Lenovo",
    "Lunch_Box",
    "Pepsi",
    "Perricone",
    "Caplets",
    "Nintendo",
    "Puzzle",
    "Wipe_Warmer",
    "Nescafe",
    "NESCAFE",
    "Nestl",
    "Netgear",
    "Pokemon",
    "Super_Mario",
]

container_object_key_wards = [
    "Frypan",
    "Fry_Pan",
    "Cake_Pan",
    "Bowl",
    "Pot",
    "Mug",
    "Saucer",
    "Dish",
    "Container",
    "Plate",
    "Piccher",
    "Ramekin",
    "Dog_Bow",
]


class SpecialGeometry:
    LargeFlat = 0
    MediumFlat = 1
    SmallFlat = 2
    Container = 3


def include_one_of_keywards(gso_name, keywards):
    for kw in keywards:
        if kw in gso_name:
            return True
    return False


if __name__ == "__main__":
    shape_name_table = {
        "LargeFlatShape": [],
        "MediumFlatShape": [],
        "SmallFlatShape": [],
        "ContainerShape": [],
    }

    gso_dataset_path = get_gso_dataset_path()

    flat_objects = []
    for gso_name in get_all_gso_names():
        if not include_one_of_keywards(gso_name, flat_object_key_wards):
            continue

        jsonfile_path = gso_dataset_path / gso_name / "data.json"

        with jsonfile_path.open(mode="r") as f:
            obj = json.load(f)
            bbox_min, bbox_max = obj["kwargs"]["bounds"]
            width_list = [u - l for u, l in zip(bbox_max, bbox_min)]
            max_width = max(width_list)
            if max_width > 0.3:
                shape_name_table["LargeFlatShape"].append(gso_name)
            elif max_width > 0.15:
                shape_name_table["MediumFlatShape"].append(gso_name)
            else:
                shape_name_table["SmallFlatShape"].append(gso_name)

    for gso_name in get_all_gso_names():
        if include_one_of_keywards(gso_name, container_object_key_wards):
            shape_name_table["ContainerShape"].append(gso_name)

    shape_name_table_path = gso_dataset_path / "shape_name_table.json"
    with shape_name_table_path.open(mode="w") as f:
        json.dump(shape_name_table, f, indent=2)
