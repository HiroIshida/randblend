import time

import pybullet
import pybullet_data

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
)
from randblend.path import get_gso_dataset_path
from randblend.pybullet_object import CubeObjectBulletObject, FileBasedBulletObject

path = (get_gso_dataset_path() / "CITY_TAXI_POLICE_CAR").expanduser()

pose = Pose.create(translation=(0.0, 0.0, 0.9))
desc = FileBasedObjectDescription.from_gso_path(path, scale = 5, pose=pose)
obj = FileBasedBulletObject.from_descriptoin(desc)

pose = Pose.create(translation=(0.0, 0.0, 0.8))
table_desc = CubeObjectDescription("table", pose, (0.8, 0.5, 0.03))
table = CubeObjectBulletObject.from_descriptoin(table_desc)

pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
pybullet.setGravity(0, 0, -10)
planeId = pybullet.loadURDF("plane.urdf")

obj.spawn_bullet_object()
table.spawn_bullet_object()

time.sleep(10)
