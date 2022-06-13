import pickle
import random

import numpy as np
import pybullet
import pybullet_data

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
)
from randblend.gso_dataset import get_all_gso_names, get_gso_names_by_shape
from randblend.path import TemporaryDataPaths
from randblend.predicate import (
    IsTouchingPredicate,
    compute_predicates_for_spawned_objects,
)
from randblend.pybullet_object import (
    CubeObjectBulletObject,
    FileBasedBulletObject,
    serialize_spawned_object_to_pickle,
    spawn_registered_objects,
    update_spawned_object_descriptions,
)


def save_current_snapshot():
    temp = TemporaryDataPaths.random("touching")

    update_spawned_object_descriptions()

    pickle_str = serialize_spawned_object_to_pickle()

    with temp.descriptions_path.open(mode="wb") as f:
        f.write(pickle_str)

    preds = compute_predicates_for_spawned_objects(IsTouchingPredicate)

    with temp.predicate_path.open(mode="wb") as f:
        pickle.dump(preds, f)


if __name__ == "__main__":
    # meshes
    containre_gso_names = get_gso_names_by_shape("LargeContainerShape")
    tiny_gso_names = get_gso_names_by_shape("TinyShape")

    def myrand(a):
        return np.array([np.random.rand() * a, np.random.rand() * a, 0.0]) - np.array(
            [0.5 * a, 0.5 * a, 0.0]
        )

    for _ in range(3):
        gso_name = random.choice(containre_gso_names)
        trans = myrand(0.8) + np.array([0, 0, 0.05])
        pose = Pose.create(translation=trans)
        desc_container = FileBasedObjectDescription.from_gso_name(gso_name, pose=pose)
        FileBasedBulletObject.from_descriptoin(desc_container)

        for _ in range(4):
            gso_name = random.choice(tiny_gso_names)
            desc = FileBasedObjectDescription.from_gso_name(
                gso_name, pose=Pose.create()
            )
            FileBasedBulletObject.from_descriptoin(desc)
            desc.place_on_top_of(desc_container)

    gso_names = get_all_gso_names()
    for _ in range(3):
        gso_name = random.choice(gso_names)
        trans = myrand(0.6) + np.array([0, 0, 0.2 + np.random.rand() * 0.2])
        pose = Pose.create(translation=trans)
        desc = FileBasedObjectDescription.from_gso_name(gso_name, pose=pose)
        FileBasedBulletObject.from_descriptoin(desc)

    # floor object
    floor_desc = CubeObjectDescription.create_floor()
    CubeObjectBulletObject.from_descriptoin(floor_desc)

    pybullet.connect(pybullet.DIRECT)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
    pybullet.setGravity(0, 0, -10.0)

    spawn_registered_objects()

    for i in range(50):
        pybullet.stepSimulation()
    save_current_snapshot()

    for i in range(300):
        pybullet.stepSimulation()
    save_current_snapshot()
