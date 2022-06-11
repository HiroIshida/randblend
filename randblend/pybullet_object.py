import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar, Generic, Optional, Type, TypeVar

import pybullet

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Float3d,
    Float4d,
    ObjectDescriptionT,
)

BulletObjectT = TypeVar("BulletObjectT", bound="BulletObject")


@dataclass
class BulletObject(ABC, Generic[ObjectDescriptionT]):
    description_type: ClassVar[Type]
    description: ObjectDescriptionT
    client: int
    object_handle: Optional[int]

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

    def set_pose(self, translation: Float3d, orientation: Float4d):
        pybullet.resetBasePositionAndOrientation(
            self.object_handle, translation, orientation, physicsClientId=self.client
        )

    @abstractmethod
    def _spawn_bullet_object(self) -> int:
        pass


class FileBasedBulletObject(BulletObject[FileBasedObjectDescription]):
    def _spawn_bullet_object(self) -> int:
        description = self.description
        urdf_path = os.path.join(description.path, "object.urdf")
        object_handle = pybullet.loadURDF(urdf_path, globalScaling = description.scale)
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
        return body_id