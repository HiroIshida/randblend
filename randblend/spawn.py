import pathlib

import bpy

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
    # https://blender.stackexchange.com/questions/24133/modify-obj-after-import-using-python
    mesh_path = pathlib.Path(fbobj.path).expanduser()
    # bpy.ops.import_scene.obj(filepath=str(mesh_path), use_split_objects=False)

    bpy.ops.import_scene.obj(filepath=str(mesh_path))
    objs = bpy.context.selected_objects
    obj = objs[-1]
    obj.location = fbobj.pose.translation
