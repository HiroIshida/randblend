import csv
import json
import pathlib
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Optional, Union
from urllib import request

import certifi
import yaml


class CreationMethod(Enum):
    PBRApproximated = auto()
    PBRPhotogrammetry = auto()
    PBRProcedural = auto()
    PBRMultiAngle = auto()
    PlainPhoto = auto()
    ThreeDPhotogrammetry = auto()
    HDRIStitched = auto()
    HDRIStitchedEdited = auto()
    UnknownOrOther = auto()


class DataType(Enum):
    ThreeDModelk = auto()
    Atlas = auto()
    Brush = auto()
    Decal = auto()
    HDRI = auto()
    Material = auto()
    PlainTexture = auto()
    Substance = auto()
    Terrain = auto()


class SortMethod(Enum):
    Latest = auto()
    Popular = auto()
    Alphabet = auto()
    Downloads = auto()


@dataclass
class Query:
    # https://help.ambientcg.com/04-API/API_v2.html
    method: Optional[CreationMethod] = None
    type: Optional[DataType] = None
    sort: Optional[SortMethod] = None
    id: Optional[str] = None
    category: Optional[str] = None

    def __post_init__(self):
        if self.category is not None:
            start_with_upper = self.category[0].isupper()
            assert start_with_upper

    def create_request(self) -> request.Request:
        url = "https://ambientcg.com/api/v2/downloads_csv?"
        for key in self.__dataclass_fields__:
            val = self.__dict__[key]
            if val is None:
                continue
            if isinstance(val, Enum):
                val_name = val.name
                if val_name.startswith("Three"):
                    val_name = val_name.replace("Three", "3")
                url += "{}={}&".format(key, val_name)
            else:
                url += "{}={}&".format(key, val)
        url = url[:-1]
        headers = {
            "User-Agent": "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:47.0) Gecko/20100101 Firefox/47.0"
        }
        req = request.Request(url, headers=headers)
        return req


@dataclass
class DownloadInfo:
    assetid: str
    attribute: str
    file_type: str
    size: int
    download_link: str
    raw_link: str

    @property
    def quarity(self) -> str:
        matches: List[str] = re.findall(r"(\w+)-(\w+)", self.attribute)
        return matches[0][0]

    def download(self, output_dir: Union[str, pathlib.Path] = ".") -> pathlib.Path:
        name = "{}_{}.{}".format(self.assetid, self.quarity, "zip")
        if isinstance(output_dir, str):
            output_dir = pathlib.Path(output_dir)
        fullpath = (output_dir / name).expanduser()
        subprocess.run(
            [
                "wget",
                "-c",
                "--no-check-certificate",
                self.download_link,
                "-O",
                str(fullpath),
            ]
        )
        return fullpath


def get_metainfo(asset_id: str) -> Dict:
    url = "https://ambientcg.com/api/v2/full_json?id={}".format(asset_id)
    headers = {
        "User-Agent": "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:47.0) Gecko/20100101 Firefox/47.0"
    }
    req = request.Request(url, headers=headers)
    resp = request.urlopen(req, cafile=certifi.where())
    data_json = json.loads(resp.read())
    return data_json["foundAssets"][0]


def create_download_info_list(csv_filename: str, attribute: str) -> List[DownloadInfo]:
    info_list: List[DownloadInfo] = []

    with open(csv_filename, newline="") as f:
        reader = csv.reader(f)
        next(reader)
        while True:
            try:
                elem = next(reader)
                info = DownloadInfo(*elem)  # type: ignore
                if info.attribute == attribute:
                    info_list.append(info)
            except StopIteration:
                del reader
                break
    return info_list


def get_download_info_list(
    q: Query, resolution: str = "2K", extension: str = "JPG"
) -> List[DownloadInfo]:

    assert resolution.isupper()
    assert extension.isupper()
    attribute = resolution + "-" + extension

    with tempfile.NamedTemporaryFile() as f:
        filename = f.name
        req = q.create_request()
        resp = request.urlopen(req, cafile=certifi.where())
        with open(filename, "w") as f:
            f.write(resp.read().decode("utf-8"))

        info_list = create_download_info_list(filename, attribute)
    return info_list


if __name__ == "__main__":
    data_dir = pathlib.Path("./texture_dataset")
    data_dir.mkdir(exist_ok=True)

    queries = [Query(category=cat) for cat in ["Wood", "Fabric", "Floor", "Carpet"]]

    for q in queries:
        metainfo_dict: Dict[str, DownloadInfo] = {}
        info_list = get_download_info_list(q)
        for download_info in info_list:
            if download_info.assetid in metainfo_dict:
                continue
            metainfo = get_metainfo(download_info.assetid)
            metainfo["link"] = download_info.raw_link
            metainfo_dict[download_info.assetid] = metainfo  # type: ignore

            zip_path = download_info.download(output_dir=data_dir)
            extraction_dir_path = zip_path.with_suffix("")
            if extraction_dir_path.exists():
                shutil.rmtree(str(extraction_dir_path))
            extraction_dir_path.mkdir(exist_ok=True)
            subprocess.run(
                ["unzip", "-qq", str(zip_path), "-d", str(extraction_dir_path)]
            )
            subprocess.run(["rm", str(zip_path)])

            yaml_path = data_dir / "metainfo.yaml"
            with yaml_path.open(mode="w") as f:
                yaml.dump(metainfo_dict, f)
