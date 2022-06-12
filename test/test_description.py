import uuid

from randblend.description import (
    CubeObjectDescription,
    FileBasedObjectDescription,
    Pose,
    RawDict,
    WorldDescription,
)


def test_pose_coding():
    pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
    pose_again = pose.from_json(pose.to_json())
    assert pose_again == pose


def test_filebased_object():
    pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
    meta = RawDict({"hoge": "hogehoge"})
    bbox_min = (0.0, 0.0, 0.0)
    bbox_max = (1.0, 1.0, 1.0)
    obj = FileBasedObjectDescription(
        "obj", pose, 2.0, "/tmp/hogehoge", bbox_min, bbox_max, meta
    )
    obj_again = obj.from_json(obj.to_json())
    assert obj == obj_again


def test_cube_object():
    pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
    obj = CubeObjectDescription("cube", pose, (2, 3, 4))
    obj_again = obj.from_json(obj.to_json())
    assert obj == obj_again


def test_world_description():
    static_obj_list = []
    for i in range(2):
        pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
        obj = CubeObjectDescription("cube", pose, (2, 3, 4))
        static_obj_list.append(obj)

        pose = Pose(translation=(0, 1, 2), orientation=(3, 2, 1, 0))
        bbox_min = (0.0, 0.0, 0.0)
        bbox_max = (1.0, 1.0, 1.0)
        file_uuid = str(uuid.uuid4())[-6:]
        meta = RawDict({"hoge": "hogehoge"})
        obj = FileBasedObjectDescription(
            file_uuid, pose, 2.0, "/tmp/{}".format(file_uuid), bbox_min, bbox_max, meta
        )
        static_obj_list.append(obj)
    wd = WorldDescription(tuple(static_obj_list))
    wd_again = wd.from_json(wd.to_json())
    assert wd == wd_again
