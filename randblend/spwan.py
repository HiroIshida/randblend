import pathlib
from abc import ABC, abstractmethod

import bpy
import trimesh

from randblend.utils import create_mesh_from_pydata
from randblend.world_description import FileBasedObject


def create_mesh_from_file(
    scene: bpy.types.Scene,
    path: pathlib.Path,
    mesh_name: str,
    object_name: str,
    use_smooth: bool = True,
) -> bpy.types.Object:

    mesh = trimesh.load_mesh(str(path))
    V = mesh.vertices
    F = mesh.faces
    return create_mesh_from_pydata(
        scene, V, F, mesh_name, object_name, use_smooth=use_smooth
    )


def create_obj(scene: bpy.types.Scene, fbobj: FileBasedObject) -> bpy.types.Object:
    path = pathlib.Path(fbobj.path)
    obj = create_mesh_from_file(scene, path, fbobj.name + "-mesh", fbobj.name)
    return obj
