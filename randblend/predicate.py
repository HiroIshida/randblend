import itertools
from abc import abstractmethod
from dataclasses import dataclass
from typing import List, Type, TypeVar

import numpy as np
import pybullet

from randblend.pybullet_object import BulletObject, _spawned_objects


@dataclass
class AbstractPredicate:
    id1: str
    id2: str
    value: float

    @classmethod
    @abstractmethod
    def from_objects(cls, obj1: BulletObject, obj2: BulletObject, **kwargs):
        pass


PredicateT = TypeVar("PredicateT", bound=AbstractPredicate)


@dataclass
class IsTouchingPredicate:
    id1: str
    id2: str
    value: float

    @classmethod
    def from_objects(cls, obj1: BulletObject, obj2: BulletObject, **kwargs):
        res = pybullet.getContactPoints(obj1.object_handle, obj2.object_handle)
        value = float(len(res) > 0)
        return cls(obj1.name, obj2.name, value)


def is_relevant_objects(obj1: BulletObject, obj2: BulletObject, relevant_dist_th=0.1):
    closest_points = pybullet.getClosestPoints(
        obj1.object_handle, obj2.object_handle, relevant_dist_th
    )
    return len(closest_points) > 0


def compute_predicates_for_spawned_objects(
    predicate_type: Type[PredicateT],
) -> List[PredicateT]:

    predicates = []
    for pair in itertools.combinations(_spawned_objects.values(), 2):
        if np.random.rand() < 0.5:
            pair = list(reversed(pair))
        obj1, obj2 = pair

        if not is_relevant_objects(obj1, obj2):
            if np.random.rand() < 0.8:
                continue

        if obj1.name == "plane" or obj2.name == "plane":
            if np.random.rand() < 0.9:
                # because plane is too trivial
                continue

        p = predicate_type.from_objects(obj1, obj2)
        predicates.append(p)
    return predicates
