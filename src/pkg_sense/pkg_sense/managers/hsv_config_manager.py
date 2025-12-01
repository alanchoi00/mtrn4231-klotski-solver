from __future__ import annotations

import os
from typing import TYPE_CHECKING

from ament_index_python import get_package_share_directory

if TYPE_CHECKING:
    from ..sense_node import Sense

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import numpy as np
import yaml


@dataclass
class HSVBounds:
    h_min: int
    s_min: int
    v_min: int
    h_max: int
    s_max: int
    v_max: int


@dataclass
class HSVRange:
    name: str
    low: np.ndarray  # shape (3,), dtype uint8
    high: np.ndarray  # shape (3,), dtype uint8


@dataclass
class HSVConfig:
    ranges: Dict[str, HSVRange]

    @staticmethod
    def load_hsv_config(config_path: str | Path) -> "HSVConfig":
        path = Path(config_path)
        with path.open("r") as f:
            raw: Dict[str, Dict[str, int]] = yaml.safe_load(f)

        ranges: Dict[str, HSVRange] = {}
        for name, vals in raw.items():
            bounds = HSVBounds(**vals)
            low = np.array([bounds.h_min, bounds.s_min, bounds.v_min], dtype=np.uint8)
            high = np.array([bounds.h_max, bounds.s_max, bounds.v_max], dtype=np.uint8)
            ranges[name] = HSVRange(name=name, low=low, high=high)

        return HSVConfig(ranges=ranges)


class HSVConfigManager:
    """
    Manages HSV configuration loading and access.
    """

    def __init__(self, node: "Sense", config_path: str | Path):
        self.node = node

        # Load default HSV config
        pkg_share = get_package_share_directory("pkg_sense")
        default_hsv_config_file = os.path.join(pkg_share, "config", config_path)
        self._hsv_config = HSVConfig.load_hsv_config(default_hsv_config_file)
        # TODO: add hsv config service client initialization here

    def get_hsv_range(self, name: str) -> HSVRange | None:
        return self._hsv_config.ranges.get(name, None)

    def get_hsv_config(self) -> HSVConfig:
        return self._hsv_config
