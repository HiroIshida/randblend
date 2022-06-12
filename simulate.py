import random

import numpy as np
import pybullet
import pybullet_data

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
)
from randblend.gso_dataset import get_gso_names_by_shape
from randblend.pybullet_object import (
    CubeObjectBulletObject,
    FileBasedBulletObject,
    serialize_spawned_object_to_pickle,
    spawn_registered_objects,
    update_spawned_object_descriptions,
)

# table object
pose = Pose.create(translation=np.array([0.0, 0.0, 0.8]))
table_desc = CubeObjectDescription("table", pose, np.array([0.8, 0.5, 0.03]))
CubeObjectBulletObject.from_descriptoin(table_desc)

# meshes
large_flat_gso_names = get_gso_names_by_shape("LargeFlatShape")
gso_name = random.choice(large_flat_gso_names)
pose = Pose.create(translation=np.array([0.0, 0.0, 0.8]))
desc = FileBasedObjectDescription.from_gso_name(gso_name, scale=1, pose=pose)
desc.place_on_top_of(table_desc)
FileBasedBulletObject.from_descriptoin(desc)


# mesh object
# for _ in range(6):
#    gso_name = randomly_pick_gso_name()
#    pose = Pose.create(
#        translation=(np.random.randn() * 0.1, np.random.randn() * 0.2, 1.3)
#    )
#    desc = FileBasedObjectDescription.from_gso_name(gso_name, scale=1, pose=pose)
#    FileBasedBulletObject.from_descriptoin(desc)


# floor object
floor_desc = CubeObjectDescription.create_floor()
CubeObjectBulletObject.from_descriptoin(floor_desc)


pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
pybullet.setGravity(0, 0, -10)

spawn_registered_objects()

import time

for i in range(300):
    pybullet.stepSimulation()
    update_spawned_object_descriptions()

pickle_str = serialize_spawned_object_to_pickle()
with open("/tmp/randblend.json", "wb") as f:
    f.write(pickle_str)

time.sleep(100)
