import random

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
    serialize_to_json,
    spawn_registered_objects,
    update_spawned_object_descriptions,
)

large_flat_gso_names = get_gso_names_by_shape("LargeFlatShape")
gso_name = random.choice(large_flat_gso_names)
pose = Pose.create(translation=(0.0, 0.0, 0.8))
desc = FileBasedObjectDescription.from_gso_name(gso_name, scale=1, pose=pose)
FileBasedBulletObject.from_descriptoin(desc)


# mesh object
# for _ in range(6):
#    gso_name = randomly_pick_gso_name()
#    pose = Pose.create(
#        translation=(np.random.randn() * 0.1, np.random.randn() * 0.2, 1.3)
#    )
#    desc = FileBasedObjectDescription.from_gso_name(gso_name, scale=1, pose=pose)
#    FileBasedBulletObject.from_descriptoin(desc)

# table object
pose = Pose.create(translation=(0.0, 0.0, 0.8))
table_desc = CubeObjectDescription("table", pose, (0.8, 0.5, 0.03))
CubeObjectBulletObject.from_descriptoin(table_desc)

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

json_str = serialize_to_json()
with open("/tmp/randblend.json", "w") as f:
    f.write(json_str)

time.sleep(100)
