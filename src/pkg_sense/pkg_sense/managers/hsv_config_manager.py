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

from klotski_interfaces.srv import GetHSVRanges, SetHSVRanges, ExportHSVRangesYaml
from klotski_interfaces.msg import HSVRange as HSVRangeMsg, HSVRanges as HSVRangesMsg


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
    Manages HSV configuration loading, access, and runtime updates.
    Provides ROS2 services to get/set HSV values and export as YAML.
    Publishes current HSV ranges on a topic.
    """

    def __init__(self, node: "Sense", config_path: str | Path):
        self.node = node
        self._config_filename = config_path

        # Load default HSV config from installed share directory
        pkg_share = get_package_share_directory("pkg_sense")
        self._installed_config_path = os.path.join(pkg_share, "config", config_path)
        self._hsv_config = HSVConfig.load_hsv_config(self._installed_config_path)

        # Create HSV ranges publisher
        self._hsv_ranges_pub = node.create_publisher(
            HSVRangesMsg,
            "/sense/hsv_ranges",
            10,
        )

        # Create HSV config services
        self._get_hsv_srv = node.create_service(
            GetHSVRanges,
            "/sense/get_hsv_ranges",
            self._get_hsv_ranges_callback,
        )
        self._set_hsv_srv = node.create_service(
            SetHSVRanges,
            "/sense/set_hsv_ranges",
            self._set_hsv_ranges_callback,
        )
        self._export_yaml_srv = node.create_service(
            ExportHSVRangesYaml,
            "/sense/export_hsv_ranges_yaml",
            self._export_hsv_ranges_yaml_callback,
        )
        node.get_logger().info(
            "HSV config services ready: /sense/get_hsv_ranges, "
            "/sense/set_hsv_ranges, /sense/export_hsv_ranges_yaml"
        )
        node.get_logger().info("HSV ranges publisher ready: /sense/hsv_ranges")

        # Publish initial HSV ranges
        self.publish_hsv_ranges()

    def get_hsv_range(self, name: str) -> HSVRange | None:
        return self._hsv_config.ranges.get(name, None)

    def get_hsv_config(self) -> HSVConfig:
        return self._hsv_config

    def set_hsv_range(self, name: str, hsv_range: HSVRange) -> None:
        """Update a single HSV range at runtime and publish the update."""
        self._hsv_config.ranges[name] = hsv_range
        self.publish_hsv_ranges()

    def set_hsv_config(self, config: HSVConfig) -> None:
        """Replace the entire HSV config at runtime and publish the update."""
        self._hsv_config = config
        self.publish_hsv_ranges()

    def publish_hsv_ranges(self) -> None:
        """Publish current HSV ranges to the topic."""
        ranges_msg = HSVRangesMsg()
        ranges_msg.ranges = [
            self._hsv_range_to_msg(hsv_range)
            for hsv_range in self._hsv_config.ranges.values()
        ]
        self._hsv_ranges_pub.publish(ranges_msg)

    def to_yaml_string(self) -> str:
        """Convert current HSV config to a YAML formatted string."""
        yaml_data: Dict[str, Dict[str, int]] = {}
        for name, hsv_range in self._hsv_config.ranges.items():
            yaml_data[name] = {
                "h_min": int(hsv_range.low[0]),
                "h_max": int(hsv_range.high[0]),
                "s_min": int(hsv_range.low[1]),
                "s_max": int(hsv_range.high[1]),
                "v_min": int(hsv_range.low[2]),
                "v_max": int(hsv_range.high[2]),
            }
        return yaml.safe_dump(yaml_data, default_flow_style=False, sort_keys=False)

    def reload_from_yaml(self, file_path: str | Path | None = None) -> None:
        """Reload HSV config from YAML file."""
        path = file_path if file_path else self._installed_config_path
        self._hsv_config = HSVConfig.load_hsv_config(path)
        self.node.get_logger().info(f"HSV config reloaded from: {path}")

    def _hsv_range_to_msg(self, hsv_range: HSVRange) -> HSVRangeMsg:
        """Convert internal HSVRange to ROS message."""
        return HSVRangeMsg(
            name=hsv_range.name,
            h_min=int(hsv_range.low[0]),
            s_min=int(hsv_range.low[1]),
            v_min=int(hsv_range.low[2]),
            h_max=int(hsv_range.high[0]),
            s_max=int(hsv_range.high[1]),
            v_max=int(hsv_range.high[2]),
        )

    def _msg_to_hsv_range(self, msg: HSVRangeMsg) -> HSVRange:
        """Convert ROS message to internal HSVRange."""
        return HSVRange(
            name=msg.name,
            low=np.array([msg.h_min, msg.s_min, msg.v_min], dtype=np.uint8),
            high=np.array([msg.h_max, msg.s_max, msg.v_max], dtype=np.uint8),
        )

    def _get_hsv_ranges_callback(
        self, request: GetHSVRanges.Request, response: GetHSVRanges.Response
    ) -> GetHSVRanges.Response:
        """Service callback to get current HSV ranges."""
        try:
            ranges_msg = HSVRangesMsg()
            ranges_msg.ranges = [
                self._hsv_range_to_msg(hsv_range)
                for hsv_range in self._hsv_config.ranges.values()
            ]
            response.ranges = ranges_msg
            response.ok = True
            response.message = f"Retrieved {len(ranges_msg.ranges)} HSV ranges"
            self.node.get_logger().debug(response.message)
        except Exception as e:
            response.ok = False
            response.message = f"Failed to get HSV ranges: {e}"
            self.node.get_logger().error(response.message)

        return response

    def _set_hsv_ranges_callback(
        self, request: SetHSVRanges.Request, response: SetHSVRanges.Response
    ) -> SetHSVRanges.Response:
        """Service callback to set HSV ranges at runtime."""
        try:
            # Update runtime values
            updated_names = []
            for range_msg in request.ranges.ranges:
                hsv_range = self._msg_to_hsv_range(range_msg)
                self.set_hsv_range(range_msg.name, hsv_range)
                updated_names.append(range_msg.name)

            response.message = (
                f"Updated {len(updated_names)} HSV ranges (runtime only): "
                f"{', '.join(updated_names)}"
            )
            response.ok = True
            self.node.get_logger().info(response.message)

        except Exception as e:
            response.ok = False
            response.message = f"Failed to set HSV ranges: {e}"
            self.node.get_logger().error(response.message)

        return response

    def _export_hsv_ranges_yaml_callback(
        self,
        request: ExportHSVRangesYaml.Request,
        response: ExportHSVRangesYaml.Response,
    ) -> ExportHSVRangesYaml.Response:
        """Service callback to export current HSV ranges as YAML string."""
        try:
            response.yaml_content = self.to_yaml_string()
            response.ok = True
            response.message = "HSV ranges exported as YAML"
            self.node.get_logger().info(response.message)
        except Exception as e:
            response.ok = False
            response.yaml_content = ""
            response.message = f"Failed to export HSV ranges: {e}"
            self.node.get_logger().error(response.message)

        return response
