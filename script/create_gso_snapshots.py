import os
import pathlib
from multiprocessing import Pool

from randblend.path import get_gso_dataset_path, get_root_path

snapshots_path = get_root_path() / "gso_snapshots"
snapshots_path.mkdir(exist_ok=True)
data_path_list = list(get_gso_dataset_path().iterdir())


def task(filepath: pathlib.Path):
    import numpy as np
    import trimesh

    meshfile_path = filepath / "visual_geometry.obj"
    name = meshfile_path.parent.name

    mesh = trimesh.load_mesh(str(meshfile_path))
    scene = mesh.scene()

    rotate = trimesh.transformations.rotation_matrix(
        angle=np.radians(75.0), direction=[1, 0.2, 0.2], point=scene.centroid
    )

    camera_old, _geometry = scene.graph[scene.camera.name]
    camera_new = np.dot(rotate, camera_old)
    scene.graph[scene.camera.name] = camera_new

    png = scene.save_image(resolution=[400, 200], visible=True)
    outputfile_path = snapshots_path / (name + ".png")
    with outputfile_path.open(mode="wb") as f:
        f.write(png)
        f.close()
    print(len(list(snapshots_path.iterdir())))


n = os.cpu_count()
p = Pool(n - 1)
p.map(task, data_path_list)
