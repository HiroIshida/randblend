import pathlib

import bpy
from scipy.spatial.transform import Rotation

from randblend.world_description import FileBasedObject

# def create_mesh_from_file(
#    scene: bpy.types.Scene,
#    path: pathlib.Path,
#    mesh_name: str,
#    object_name: str,
#    use_smooth: bool = True,
# ) -> bpy.types.Object:
#
#    mesh = trimesh.load_mesh(str(path))
#    V = mesh.vertices
#    F = mesh.faces
#    return create_mesh_from_pydata(
#        scene, V, F, mesh_name, object_name, use_smooth=use_smooth
#    )


def create_obj(scene: bpy.types.Scene, fbobj: FileBasedObject) -> bpy.types.Object:
    mesh_path = pathlib.Path(fbobj.path).expanduser()

    bpy.ops.import_scene.obj(filepath=str(mesh_path))
    objs = bpy.context.selected_objects
    obj = objs[-1]
    obj.location = fbobj.pose.translation

    rot = Rotation.from_quat(fbobj.pose.orientation)
    obj.rotation_euler = tuple(rot.as_euler("zyx").tolist())
    return obj
