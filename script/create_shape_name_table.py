import json

from randblend.gso_dataset import get_all_gso_names
from randblend.path import get_gso_dataset_path

flat_object_include_keywards = [
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

flat_object_exclude_keywards = [
    "LEGO_5887_Dino_Defense_HQ",
    "Cootie_Game",
    "Melissa_Doug_Chunky_Puzzle_Vehicles",
    "Hasbro_Dont_Wake_Daddy_Board_Game_NJnjGna4u1a",
]

container_object_include_keywards = [
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

container_object_exclude_keywards = []


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


def exclude_all_keywards(gso_name, keywards):
    for kw in keywards:
        if kw in gso_name:
            return False
    return True


def match_keyward_conditions(gso_name, include_keywards, exclude_keywards):
    if include_keywards is not None:
        if not include_one_of_keywards(gso_name, include_keywards):
            return False
    if exclude_keywards is not None:
        if not exclude_all_keywards(gso_name, exclude_keywards):
            return False
    return True


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
        if not match_keyward_conditions(
            gso_name, flat_object_include_keywards, flat_object_exclude_keywards
        ):
            continue

        jsonfile_path = gso_dataset_path / gso_name / "data.json"

        with jsonfile_path.open(mode="r") as f:
            obj = json.load(f)
            bbox_min, bbox_max = obj["kwargs"]["bounds"]
            width_list = [u - l for u, l in zip(bbox_max, bbox_min)]

            min_width = min(width_list)
            max_width = max(width_list)

            if min_width * 4 > max_width:
                continue
            if max_width > 0.3:
                shape_name_table["LargeFlatShape"].append(gso_name)
            elif max_width > 0.15:
                shape_name_table["MediumFlatShape"].append(gso_name)
            else:
                shape_name_table["SmallFlatShape"].append(gso_name)

    for gso_name in get_all_gso_names():
        if match_keyward_conditions(
            gso_name,
            container_object_include_keywards,
            container_object_exclude_keywards,
        ):
            shape_name_table["ContainerShape"].append(gso_name)

    shape_name_table_path = gso_dataset_path / "shape_name_table.json"
    with shape_name_table_path.open(mode="w") as f:
        json.dump(shape_name_table, f, indent=2)
