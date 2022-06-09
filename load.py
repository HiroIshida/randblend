import os
import sys

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)
sys.path.extend(["/home/h-ishida/.pyenv/versions/3.10.2/lib/python3.10/site-packages"])

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import bpy

import randblend.utils as utils

OptionalPath = Optional[Path]


@dataclass
class FileBasedMaterial:
    name: str
    color: OptionalPath = None
    metalness: OptionalPath = None
    roughness: OptionalPath = None
    normalgl: OptionalPath = None
    displacement: OptionalPath = None
    ambientocclusion: OptionalPath = None

    @classmethod
    def from_ambientcg_path(cls, base_dir_path: Path):
        base_dir_path = base_dir_path.expanduser()
        data_id = base_dir_path.name
        extension = "jpg"  # TODO(HiroIshida) png

        postfix_list = [
            "Color",
            "Metalness",
            "Roughness",
            "NormalGL",
            "Displacement",
            "AmbientOcclusion",
        ]

        kwargs = {}
        for postfix in postfix_list:
            path = base_dir_path / "{}_{}.{}".format(data_id, postfix, extension)
            if not path.exists():
                path = None
            kwargs[postfix.lower()] = path

        return cls(name=data_id, **kwargs)


def add_named_material(
    fbmat: FileBasedMaterial, scale=(1.0, 1.0, 1.0), displacement_scale: float = 1.0
) -> bpy.types.Material:
    mat = utils.add_material(fbmat.name, use_nodes=True, make_node_tree_empty=True)

    def convert(path: OptionalPath) -> str:
        if isinstance(path, Path):
            return str(path)
        else:
            return ""

    utils.build_pbr_textured_nodes(
        mat.node_tree,
        color_texture_path=convert(fbmat.color),
        roughness_texture_path=convert(fbmat.roughness),
        normal_texture_path=convert(fbmat.normalgl),
        metallic_texture_path=convert(fbmat.displacement),
        displacement_texture_path=convert(fbmat.displacement),
        ambient_occlusion_texture_path=convert(fbmat.ambientocclusion),
        scale=scale,
        displacement_scale=displacement_scale,
    )
    return mat


if __name__ == "__main__":

    # Args
    output_file_path = bpy.path.relpath(str(sys.argv[sys.argv.index("--") + 1]))
    resolution_percentage = int(sys.argv[sys.argv.index("--") + 2])
    num_samples = int(sys.argv[sys.argv.index("--") + 3])

    # prepare material
    p = Path("~/python/random-texture/texture_dataset/Wood027_2K")
    fbmat = FileBasedMaterial.from_ambientcg_path(p)
    add_named_material(fbmat)

    # Render Setting
    scene = bpy.data.scenes["Scene"]
    utils.clean_objects()

    # Scene Building
    obj = utils.create_smooth_monkey(location=(0, 0, 0.0), name="Suzanne")
    obj.data.materials.append(bpy.data.materials[fbmat.name])
    add_named_material(fbmat)

    camera_object = utils.create_camera(location=(1.0, 1.0, 5.0))

    utils.add_track_to_constraint(camera_object, obj)
    utils.set_camera_params(camera_object.data, obj, lens=50.0)

    utils.create_sun_light(rotation=(0.0, math.pi * 0.5, -math.pi * 0.1))

    resolution_percentage = 30
    utils.set_output_properties(scene, resolution_percentage, output_file_path)
    utils.set_cycles_renderer(scene, camera_object, num_samples)
