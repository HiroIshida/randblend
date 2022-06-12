import pybullet
import pybullet_data

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
)
from randblend.gso_dataset import randomly_pick_gso_name
from randblend.pybullet_object import (
    CubeObjectBulletObject,
    FileBasedBulletObject,
    serialize_to_json,
    update_spawned_object_descriptions,
)

# mesh object
pose = Pose.create(translation=(0.0, 0.0, 1.9))
gso_name = randomly_pick_gso_name()
desc = FileBasedObjectDescription.from_gso_name(gso_name, scale=1, pose=pose)
obj = FileBasedBulletObject.from_descriptoin(desc)

# table object
pose = Pose.create(translation=(0.0, 0.0, 0.8))
table_desc = CubeObjectDescription("table", pose, (0.8, 0.5, 0.03))
table = CubeObjectBulletObject.from_descriptoin(table_desc)

# floor object
floor_desc = CubeObjectDescription.create_floor()
floor = CubeObjectBulletObject.from_descriptoin(floor_desc)


pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
pybullet.setGravity(0, 0, -10)

obj.spawn_bullet_object()
table.spawn_bullet_object()
floor.spawn_bullet_object()

import time

for i in range(300):
    pybullet.stepSimulation()
    update_spawned_object_descriptions()

json_str = serialize_to_json()
with open("/tmp/randblend.json", "w") as f:
    f.write(json_str)

time.sleep(100)
