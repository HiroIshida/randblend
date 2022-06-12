import os
import sys

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)
sys.path.extend(["/home/h-ishida/.pyenv/versions/3.10.2/lib/python3.10/site-packages"])

import random
from typing import Tuple

import bpy
import bpycv
import cv2
import mathutils
import numpy as np
from scipy.spatial.transform import Rotation

import randblend.utils as utils
from randblend.blender_object import BlenderWorld, FileBasedMaterial
from randblend.dataset import Dataset
from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
    WorldDescription,
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

    bpycv.clear_all()
    bpy.context.scene.frame_set(1)
    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.samples = 8
    # bpy.context.scene.render.resolution_y = 512
    # bpy.context.scene.render.resolution_x = 512

    # prepare material
    fbmat_wood = FileBasedMaterial.from_ambientcg_id(material_table)
    fbmat_carpet = FileBasedMaterial.from_ambientcg_id(
        material_floor, scale=(0.1, 0.1, 0.1)
    )

    descriptions = []

    # create table
    pose = Pose.create(translation=(0.0, 0.0, 0.8))
    table_description = CubeObjectDescription("table", pose, (0.8, 0.5, 0.03))
    descriptions.append(table_description)
    # table = BlenderCubeObject.from_descriptoin(table_description, texture=fbmat_wood)
    # obj = table.spawn_blender_object()

    # create mesh
    rot = Rotation.from_euler("y", 90, degrees=True)
    pose = Pose.create((0.0, 0.0, 0.9), tuple(rot.as_quat().tolist()))
    path = (get_gso_dataset_path() / "CITY_TAXI_POLICE_CAR").expanduser()
    obj_description = FileBasedObjectDescription.from_gso_path(
        path, pose=pose, scale=5.0
    )
    descriptions.append(obj_description)
    world_description = WorldDescription(descriptions)

    world = BlenderWorld.from_descriptions(world_description)

    world["table"].set_material(fbmat_wood)
    world.spawn_all()

    # create wall
    # wall = utils.create_plane(
    #    size=12.0,
    #    location=(0.0, -2.0, 0.0),
    #    rotation=(math.pi * 90.0 / 180.0, 0.0, 0.0),
    #    name="Wall",
    # )

    # create fllor
    floor = utils.create_plane(size=12.0, name="Floor")
    floor.data.materials.append(bpy.data.materials[fbmat_carpet.name])

    camera = bpy.context.scene.camera
    camera.location = (-0.0, -0.2, 3.0)
    camera.rotation_euler = (0.0, 0.0, 0.0)
    camera.data.lens = 60
    camera.data.sensor_width = 70.0
    camera.data.sensor_height = 40.0
    result = bpycv.render_data()

    # save result
    cv2.imwrite(
        "demo-rgb.jpg", result["image"][..., ::-1]
    )  # transfer RGB image to opencv's BGR

    # save instance map as 16 bit png
    # the value of each pixel represents the inst_id of the object to which the pixel belongs
    cv2.imwrite("demo-inst.png", np.uint16(result["inst"]))

    # convert depth units from meters to millimeters
    depth_in_mm = result["depth"] * 1000
    cv2.imwrite("demo-depth.png", np.uint16(depth_in_mm))  # save as 16bit png

    # visualization inst_rgb_depth for human
    cv2.imwrite("demo-vis-inst_rgb_depth-.png", result.vis()[..., ::-1])
