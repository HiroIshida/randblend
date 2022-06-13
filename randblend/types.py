from dataclasses import dataclass
from typing import Dict

import numpy as np


@dataclass
class SegmentedImage:
    image: np.ndarray
    segmenation: np.ndarray
    inst_id_map: Dict[str, int]
