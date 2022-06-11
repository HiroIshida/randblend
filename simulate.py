import time

import pybullet

from randblend.description import FileBasedObjectDescription
from randblend.path import get_gso_dataset_path
from randblend.pybullet_object import FileBasedBulletObject

path = (get_gso_dataset_path() / "CITY_TAXI_POLICE_CAR").expanduser()

pose = Pose.create(translation=(0.0, 0.0, 0.9))
desc = FileBasedObjectDescription.from_gso_path(path, scale = 5, pose=pose)
obj = FileBasedBulletObject.from_descriptoin(desc)

pybullet.connect(pybullet.GUI)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, 0)
pybullet.setGravity(0, 0, -10)

obj.spawn_bullet_object()

time.sleep(10)
