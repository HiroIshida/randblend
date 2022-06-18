from typing import List, Optional, Tuple

import bpy
import numpy as np
from mathutils import Vector


def raycast(
    camera, mesh_names: Optional[List[str]] = None
) -> Tuple[np.ndarray, np.ndarray]:

    frame = camera.data.view_frame(scene=bpy.context.scene)
    top_right = frame[0]
    bottom_right = frame[1]
    bottom_left = frame[2]
    top_left = frame[3]

    resol_ratio = bpy.context.scene.render.resolution_percentage / 100.0
    resol_x = int(bpy.context.scene.render.resolution_x * resol_ratio)
    resol_y = int(bpy.context.scene.render.resolution_y * resol_ratio)

    x_range = np.linspace(top_left[0], top_right[0], resol_x)
    y_range = np.linspace(top_left[1], bottom_left[1], resol_y)

    hit_points = np.ones((resol_y, resol_x, 3)) * np.inf
    segmentation = np.ones((resol_y, resol_x)) * -1

    if mesh_names is None:
        mesh_names = []
        for obj_name in bpy.data.objects.keys():
            if bpy.data.objects[obj_name].type == "MESH":
                mesh_names.append(obj_name)

    for mesh_id, target_name in enumerate(mesh_names):
        print(target_name)
        target = bpy.data.objects[target_name]

        mat_world = target.matrix_world
        mat_world_inv = mat_world.inverted()
        origin = mat_world_inv @ camera.matrix_world.translation

        for i, x in enumerate(x_range):
            for j, y in enumerate(y_range):
                v_pixel = Vector((x, y, top_left[2]))
                v_pixel.rotate(camera.matrix_world.to_quaternion())

                destination = mat_world_inv @ (
                    v_pixel + camera.matrix_world.translation
                )
                direction = (destination - origin).normalized()

                # perform the actual ray casting
                hit, location, norm, face = target.ray_cast(origin, direction)

                if hit:
                    pt = mat_world @ location
                    hit_points[j, i] = np.array([pt.x, pt.y, pt.z])
                    segmentation[j, i] = mesh_id

    return hit_points, segmentation
