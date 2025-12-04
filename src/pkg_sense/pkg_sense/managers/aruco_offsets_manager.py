from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..sense_node import Sense
    from .transformation_manager import TransformationManager

from klotski_interfaces.srv import GetArucoOffsets, SetArucoOffsets
from klotski_interfaces.msg import ArucoOffsets as ArucoOffsetsMsg


class ArucoOffsetsManager:
    """
    Manages ArUco offset configuration for runtime adjustment.
    Provides ROS2 services to get/set ArUco offset values.
    Publishes current offsets on a topic for UI synchronization.
    """

    def __init__(
        self,
        node: "Sense",
        transformation_manager: "TransformationManager",
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_z: float = 0.0,
    ):
        self.node = node
        self.transformation_manager = transformation_manager

        # Store initial values from YAML config
        self._initial_x = initial_x
        self._initial_y = initial_y
        self._initial_z = initial_z

        # Create ArUco offsets publisher
        self._offsets_pub = node.create_publisher(
            ArucoOffsetsMsg,
            "/sense/aruco_offsets",
            10,
        )

        # Create ArUco offset services
        self._get_offsets_srv = node.create_service(
            GetArucoOffsets,
            "/sense/get_aruco_offsets",
            self._get_offsets_callback,
        )
        self._set_offsets_srv = node.create_service(
            SetArucoOffsets,
            "/sense/set_aruco_offsets",
            self._set_offsets_callback,
        )

        node.get_logger().info(
            "ArUco offsets services ready: /sense/get_aruco_offsets, "
            "/sense/set_aruco_offsets"
        )
        node.get_logger().info("ArUco offsets publisher ready: /sense/aruco_offsets")

        # Publish initial offsets
        self.publish_offsets()

    @property
    def x(self) -> float:
        return self.transformation_manager.aruco_offset_x_m

    @property
    def y(self) -> float:
        return self.transformation_manager.aruco_offset_y_m

    @property
    def z(self) -> float:
        return self.transformation_manager.aruco_offset_z_m

    def set_offsets(self, x: float, y: float, z: float) -> None:
        """Update ArUco offsets at runtime."""
        self.transformation_manager.aruco_offset_x_m = x
        self.transformation_manager.aruco_offset_y_m = y
        self.transformation_manager.aruco_offset_z_m = z
        self.publish_offsets()

    def reset_offsets(self) -> None:
        """Reset ArUco offsets to initial values from YAML config."""
        self.set_offsets(self._initial_x, self._initial_y, self._initial_z)

    def publish_offsets(self) -> None:
        """Publish current ArUco offsets to the topic."""
        msg = ArucoOffsetsMsg()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        self._offsets_pub.publish(msg)

    def _get_offsets_callback(
        self, request: GetArucoOffsets.Request, response: GetArucoOffsets.Response
    ) -> GetArucoOffsets.Response:
        """Service callback to get current ArUco offsets."""
        try:
            response.offsets = ArucoOffsetsMsg()
            response.offsets.x = self.x
            response.offsets.y = self.y
            response.offsets.z = self.z
            response.ok = True
            response.message = (
                f"ArUco offsets: x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}"
            )
            self.node.get_logger().debug(response.message)
        except Exception as e:
            response.ok = False
            response.message = f"Failed to get ArUco offsets: {e}"
            self.node.get_logger().error(response.message)

        return response

    def _set_offsets_callback(
        self, request: SetArucoOffsets.Request, response: SetArucoOffsets.Response
    ) -> SetArucoOffsets.Response:
        """Service callback to set ArUco offsets at runtime."""
        try:
            new_x = request.offsets.x
            new_y = request.offsets.y
            new_z = request.offsets.z

            self.set_offsets(new_x, new_y, new_z)

            response.ok = True
            response.message = (
                f"ArUco offsets updated: x={new_x:.4f}, y={new_y:.4f}, z={new_z:.4f}"
            )
            self.node.get_logger().info(response.message)

        except Exception as e:
            response.ok = False
            response.message = f"Failed to set ArUco offsets: {e}"
            self.node.get_logger().error(response.message)

        return response
