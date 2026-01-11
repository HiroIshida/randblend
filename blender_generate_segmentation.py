import os
import pickle
import sys

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)
sys.path.extend(["/home/h-ishida/.pyenv/versions/3.10.2/lib/python3.10/site-packages"])


import math
import random

import bpy
import bpycv
import cv2

import randblend.utils as utils
from randblend.blender_object import BlenderWorld, FileBasedMaterial, get_inst_id_map
from randblend.blender_raycast import raycast
from randblend.dataset import Dataset
from randblend.path import TemporaryDataPaths, get_texture_metainfo_path
from randblend.types import SegmentedImage

if __name__ == "__main__":
    bpycv.clear_all()
    bpy.context.scene.frame_set(1)
    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.samples = 4
    bpy.context.scene.render.resolution_x = 640
    bpy.context.scene.render.resolution_y = 480

    # randomely pick two materials
    yaml_path = get_texture_metainfo_path()
    dataset = Dataset.construct(yaml_path)
    materials_table_cand = dataset.filter_by_categories(("Wood",))
    materials_floor_cand = dataset.filter_by_categories(
        ("Carpet", "WoodFloor", "Asphalt", "Rocks", "Ground", "OfficeCeiling")
    )

    material_table = random.choice(materials_table_cand)
    material_floor = random.choice(materials_floor_cand)

    fbmat_wood = FileBasedMaterial.from_ambientcg_id(material_table)
    fbmat_carpet = FileBasedMaterial.from_ambientcg_id(
        material_floor, scale=(0.01, 0.01, 0.01)
    )

    temp = TemporaryDataPaths.get_latest_temp("touching")

    bw = BlenderWorld.from_pickle_file(str(temp.descriptions_path))
    bw["floor"].set_material(fbmat_carpet)
    bw.spawn_all()

    utils.create_area_light(rotation=(0.0, math.pi * 0.1, -math.pi * 0.1), strength=100)

    bpy.ops.object.empty_add(location=(0.0, 0.0, 0.0))
    track_obj = bpy.context.object
    track_obj.name = "track_target_point"

    camera = bpy.context.scene.camera
    camera.location = (0.0, -0.4, 0.9)
    utils.add_track_to_constraint(camera, track_obj)

    # camera.location = (-0.0, -0.0, 0.9)
    ## camera.rotation_euler = (0.2, 0.0, 0.0)
    # camera.rotation_euler = (0.0, 0.0, 0.0)
    # camera.data.lens = 50
    # camera.data.sensor_width = 70.0
    # camera.data.sensor_height = 60.0

    dists, segmentation = raycast(camera)

    with open("segmentation.pkl", "wb") as f:
        pickle.dump(segmentation, f)

    with open("dists.pkl", "wb") as f:
        pickle.dump(dists, f)

    result = bpycv.render_data()

    image = result["image"]
    segmenation = result["inst"]
    inst_id_map = get_inst_id_map()
    segmentation = SegmentedImage(image, segmenation, inst_id_map)
    with temp.segmentation_path.open(mode="wb") as f:
        pickle.dump(segmenation, f)

    cv2.imwrite("debug.png", result.vis()[..., ::-1])
