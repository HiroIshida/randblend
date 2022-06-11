import os
import sys

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)
sys.path.extend(["/home/h-ishida/.pyenv/versions/3.10.2/lib/python3.10/site-packages"])

import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import bpy
import mathutils
import numpy as np
from scipy.spatial.transform import Rotation

import randblend.utils as utils
from randblend.dataset import Dataset
from randblend.path import (
    get_gso_dataset_path,
    get_texture_dataset_path,
    get_texture_metainfo_path,
)
from randblend.types import FileBasedObject, Pose

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
    def from_ambientcg_id(cls, texture_id: str) -> "FileBasedMaterial":
        dataset_path = get_texture_dataset_path()
        resolution = "_2K"  # TODO(load from yaml??"
        path = dataset_path / (texture_id + resolution)
        assert path.is_dir()
        return cls.from_ambientcg_path(path)

    @classmethod
    def from_ambientcg_path(cls, base_dir_path: Path) -> "FileBasedMaterial":
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


def look_at(obj_camera, point: mathutils.Vector):
    # https://blender.stackexchange.com/questions/5210
    if isinstance(point, Tuple):
        point = mathutils.Vector(point)
    loc_camera = obj_camera.matrix_world.to_translation()
    direction = point - loc_camera
    rot_quat = direction.to_track_quat("-Z", "Y")
    obj_camera.rotation_euler = rot_quat.to_euler()


if __name__ == "__main__":
    yaml_path = get_texture_metainfo_path()
    dataset = Dataset.construct(yaml_path)
    materials_table_cand = dataset.filter_by_categories(("Wood",))
    materials_floor_cand = dataset.filter_by_categories(
        ("Carpet", "WoodFloor", "Asphalt", "Rocks", "Ground", "OfficeCeiling")
    )

    material_table = random.choice(materials_table_cand)
    material_floor = random.choice(materials_floor_cand)

    # Args
    output_file_path = bpy.path.relpath("./out-")
    resolution_percentage = 30
    num_samples = 16

    # prepare material
    fbmat_wood = FileBasedMaterial.from_ambientcg_id(material_table)
    add_named_material(fbmat_wood, scale=(1, 1, 1))

    fbmat_carpet = FileBasedMaterial.from_ambientcg_id(material_floor)
    add_named_material(fbmat_carpet, scale=(0.1, 0.1, 0.1))

    # Render Setting
    scene = bpy.data.scenes["Scene"]
    utils.clean_objects()

    # create table
    bpy.ops.mesh.primitive_cube_add(location=(0.0, 0.0, 0.8))
    obj = bpy.context.object
    obj.scale = (0.8, 0.5, 0.03)
    obj.data.materials.append(bpy.data.materials[fbmat_wood.name])

    # create mesh
    pose = Pose.create(
        np.array([0.0, 0.0, 0.9]), Rotation.from_euler("y", 90, degrees=True)
    )
    scale = (5.0, 5.0, 5.0)
    path = (get_gso_dataset_path() / "CITY_TAXI_POLICE_CAR").expanduser()
    fbobject = FileBasedObject.from_gso_path(path, pose=pose, scale=scale)
    obj = fbobject.spawn_blender_object()

    # create wall
    wall = utils.create_plane(
        size=12.0,
        location=(0.0, -2.0, 0.0),
        rotation=(math.pi * 90.0 / 180.0, 0.0, 0.0),
        name="Wall",
    )

    # create fllor
    floor = utils.create_plane(size=12.0, name="Floor")
    floor.data.materials.append(bpy.data.materials[fbmat_carpet.name])

    camera_object = utils.create_camera(location=(0.0, -0.6, 3.0))
    look_at(camera_object, (0, 0.3, 0))

    # utils.add_track_to_constraint(camera_object, obj)

    # utils.set_camera_params(camera_object.data, obj, lens=50.0)
    utils.create_area_light(rotation=(0.0, math.pi * 0.1, -math.pi * 0.1), strength=100)

    utils.set_output_properties(scene, resolution_percentage, output_file_path)
    utils.set_cycles_renderer(scene, camera_object, num_samples)
