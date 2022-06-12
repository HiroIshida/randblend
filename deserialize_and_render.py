import os
import sys

working_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(working_dir_path)
sys.path.extend(["/home/h-ishida/.pyenv/versions/3.10.2/lib/python3.10/site-packages"])


import bpy
import bpycv
import cv2
import numpy as np

from randblend.blender_object import BlenderWorld

if __name__ == "__main__":
    bpycv.clear_all()
    bpy.context.scene.frame_set(1)
    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.samples = 8
    bw = BlenderWorld.from_json_file("/tmp/randblend.json")
    bw.spawn_all()

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
