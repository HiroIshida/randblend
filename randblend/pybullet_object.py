import os
import pickle
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar, Dict, Generic, Optional, Tuple, Type, TypeVar

import numpy as np
import pybullet

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    ObjectDescriptionT,
)

BulletObjectT = TypeVar("BulletObjectT", bound="BulletObject")


_registered_objects: Dict[str, "BulletObject"] = {}


def spawn_registered_objects():
    for obj in _registered_objects.values():
        obj.spawn_bullet_object()


_spawned_objects: Dict[str, "BulletObject"] = {}  # set when bullet object is spawned


def serialize_spawned_object_to_pickle() -> bytes:
    descriptions = [val.description for val in _spawned_objects.values()]
    return pickle.dumps(descriptions)


def update_spawned_object_descriptions() -> None:
    for val in _spawned_objects.values():
        val.update_description()


@dataclass
class BulletObject(ABC, Generic[ObjectDescriptionT]):
    description_type: ClassVar[Type]
    description: ObjectDescriptionT
    client: int
    object_handle: Optional[int]

    def __post_init__(self):
        # register
        _registered_objects[self.name] = self

    @classmethod
    def from_descriptoin(
        cls: Type[BulletObjectT],
        description: ObjectDescriptionT,
        client: int = 0,
    ) -> BulletObjectT:
        return cls(description=description, client=client, object_handle=None)

    def spawn_bullet_object(self) -> None:
        pose = self.description.pose
        self.object_handle = self._spawn_bullet_object()
        self.set_pose(pose.translation, pose.orientation)
        _spawned_objects[self.name] = self

    def set_pose(self, translation: np.ndarray, orientation: np.ndarray):
        assert translation.shape == (3,)
        assert orientation.shape == (4,)
        pybullet.resetBasePositionAndOrientation(
            self.object_handle, translation, orientation, physicsClientId=self.client
        )

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        trans, quat = pybullet.getBasePositionAndOrientation(
            self.object_handle, physicsClientId=self.client
        )
        return trans, quat

    def update_description(self) -> None:
        trans, quat = self.get_pose()
        self.description.pose.translation = trans
        self.description.pose.orientation = quat

    @property
    def name(self):
        return self.description.name

    @abstractmethod
    def _spawn_bullet_object(self) -> int:
        pass


class FileBasedBulletObject(BulletObject[FileBasedObjectDescription]):
    def _spawn_bullet_object(self) -> int:
        description = self.description
        urdf_path = os.path.join(description.path, "object.urdf")
        object_handle = pybullet.loadURDF(urdf_path, globalScaling=description.scale)
        return object_handle


class CubeObjectBulletObject(BulletObject[CubeObjectDescription]):
    def _spawn_bullet_object(self) -> int:
        description = self.description
        half_shape = [0.5 * e for e in description.shape]

        vis_id = pybullet.createCollisionShape(
            pybullet.GEOM_BOX, halfExtents=half_shape
        )
        body_id = pybullet.createMultiBody(
            baseMass=100000.0, baseCollisionShapeIndex=vis_id
        )

        pose = description.pose
        pybullet.resetBasePositionAndOrientation(
            body_id, pose.translation, pose.orientation
        )
        pybullet.changeDynamics(body_id, -1, mass=description.mass)
        pybullet.changeDynamics(body_id, -1, restitution=0.1, linearDamping=20)
        pybullet.changeDynamics(
            body_id, -1, localInertiaDiagonal=description.inertia.get_diagonal()
        )
        return body_id
