import uuid

from randblend.types import CubeObject, FileBasedObject, Pose, WorldDescription


def test_pose_coding():
    pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
    pose_again = pose.from_json(pose.to_json())
    assert pose_again == pose


def test_filebased_object():
    pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
    meta = {"hoge": "hogehoge"}
    obj = FileBasedObject("obj", pose, (1, 2, 3), "/tmp/hogehoge", meta)
    obj_again = obj.from_json(obj.to_json())
    assert obj == obj_again


def test_cube_object():
    pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
    obj = CubeObject("cube", pose, (2, 3, 4))
    obj_again = obj.from_json(obj.to_json())
    assert obj == obj_again


def test_world_description():
    static_obj_list = []
    for i in range(10):
        pose = Pose(translation=(0, 1, 2), orientation=(1.0, 0.0, 0.0, 0.0))
        obj = CubeObject("cube", pose, (2, 3, 4))
        static_obj_list.append(obj)

    dynamic_obj_list = []
    for i in range(10):
        pose = Pose(translation=(0, 1, 2), orientation=(3, 2, 1, 0))
        file_uuid = str(uuid.uuid4())[-6:]
        meta = {"hoge": "hogehoge"}
        obj = FileBasedObject(
            file_uuid, pose, (1, 2, 3), "/tmp/{}".format(file_uuid), meta
        )
        dynamic_obj_list.append(obj)
    wd = WorldDescription(tuple(dynamic_obj_list), tuple(static_obj_list))
    wd_again = wd.from_json(wd.to_json())
    assert wd == wd_again
