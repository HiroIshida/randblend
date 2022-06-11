import os
import sys

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)
sys.path.extend(["/home/h-ishida/.pyenv/versions/3.10.2/lib/python3.10/site-packages"])

import math
import random
from typing import Tuple

import bpy
import mathutils
from scipy.spatial.transform import Rotation

import randblend.utils as utils
from randblend.blender_object import (
    BlenderCubeObject,
    BlenderFileBasedObject,
    FileBasedMaterial,
)
from randblend.dataset import Dataset
from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
)
from randblend.path import get_gso_dataset_path, get_texture_metainfo_path


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
    fbmat_carpet = FileBasedMaterial.from_ambientcg_id(
        material_floor, scale=(0.1, 0.1, 0.1)
    )

    # Render Setting
    scene = bpy.data.scenes["Scene"]
    utils.clean_objects()

    # create table
    pose = Pose.create(translation=(0.0, 0.0, 0.8))
    table_description = CubeObjectDescription("table", pose, (0.8, 0.5, 0.03))
    table = BlenderCubeObject.from_descriptoin(table_description, texture=fbmat_wood)
    obj = table.spawn_blender_object()

    # create mesh
    rot = Rotation.from_euler("y", 90, degrees=True)
    pose = Pose.create((0.0, 0.0, 0.9), tuple(rot.as_quat().tolist()))
    scale = (5.0, 5.0, 5.0)
    path = (get_gso_dataset_path() / "CITY_TAXI_POLICE_CAR").expanduser()
    obj_description = FileBasedObjectDescription.from_gso_path(
        path, pose=pose, scale=scale
    )
    meshobj = BlenderFileBasedObject.from_descriptoin(obj_description)
    meshobj.spawn_blender_object()

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
