from typing import TYPE_CHECKING, Callable, Optional

import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from ..services.colour_mask import build_colour_masks

if TYPE_CHECKING:
    from ..sense_node import Sense
    from .hsv_config_manager import HSVConfig


class CameraManager:
    """
    Manages camera subscriptions and image processing.
    """

    def __init__(
        self,
        node: "Sense",
        world_frame_id: str,
        hsv_config_getter: Optional[Callable[[], "HSVConfig"]] = None,
    ):
        self.node = node
        self.world_frame_id = world_frame_id
        self.cv_bridge = CvBridge()
        self.cv_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self.intrinsics: Optional[rs.intrinsics] = None  # rs.intrinsics
        self._hsv_config_getter = hsv_config_getter

        # Subscribers for camera topics
        self.image_sub_topic = "/camera/camera/color/image_raw"
        self.point_cloud_sub_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self.cam_info_sub_topic = "/camera/camera/depth/camera_info"
        self.image_sub = self.node.create_subscription(
            Image, self.image_sub_topic, self.arm_image_callback, 10
        )
        self.point_cloud_sub = self.node.create_subscription(
            Image, self.point_cloud_sub_topic, self.arm_point_cloud_callback, 10
        )
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo, self.cam_info_sub_topic, self.arm_image_depth_info_callback, 10
        )

        # Publishers for overlay and rectified board images
        self.cells_overlay_image_pub_topic = "/sense/cells_overlay"
        self.rectified_board_image_pub_topic = "/sense/rectified_board"
        self.masked_image_pub_topic = "/sense/masked_image"
        self.cells_overlay_image_pub = self.node.create_publisher(
            Image, self.cells_overlay_image_pub_topic, 1
        )
        self.rectified_board_image_pub = self.node.create_publisher(
            Image, self.rectified_board_image_pub_topic, 1
        )
        self.masked_image_pub = self.node.create_publisher(
            Image, self.masked_image_pub_topic, 1
        )

        self.cache_overlay_image: Optional[np.ndarray] = None
        self.cache_rectified_image: Optional[np.ndarray] = None

        self.overlay_image_timer = self.node.create_timer(
            1.0, self.overlay_image_timer_callback
        )
        self.rectified_image_timer = self.node.create_timer(
            1.0, self.rectified_image_timer_callback
        )
        self.masked_image_timer = self.node.create_timer(
            0.1, self.masked_image_timer_callback  # 10 Hz for live preview
        )

    def get_color_image(self) -> Optional[np.ndarray]:
        return self.cv_image

    def get_depth_image(self) -> Optional[np.ndarray]:
        return self.depth_image

    def get_intrinsics(self) -> Optional[rs.intrinsics]:
        return self.intrinsics

    def arm_image_depth_info_callback(self, cameraInfo: CameraInfo):
        if self.intrinsics:
            return
        try:
            intr = rs.intrinsics()
            intr.width = cameraInfo.width
            intr.height = cameraInfo.height
            intr.ppx = cameraInfo.k[2]
            intr.ppy = cameraInfo.k[5]
            intr.fx = cameraInfo.k[0]
            intr.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == "plumb_bob":
                intr.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == "equidistant":
                intr.model = rs.distortion.kannala_brandt4
            else:
                intr.model = rs.distortion.none
            intr.coeffs = [float(i) for i in cameraInfo.d]
            self.intrinsics = intr
        except Exception as e:
            self.node.get_logger().error(f"camera info error: {e}")

    def arm_image_callback(self, image: Image):
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except Exception as e:
            self.node.get_logger().error(f"Error in arm_image_callback: {e}")

    def arm_point_cloud_callback(self, image: Image):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(image, image.encoding)
        except Exception as e:
            self.node.get_logger().error(f"Error in point_cloud_callback: {e}")

    def publish_overlay_image(self, image: np.ndarray):
        overlay_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        overlay_msg.header.stamp = self.node.get_clock().now().to_msg()
        overlay_msg.header.frame_id = self.world_frame_id
        self.cells_overlay_image_pub.publish(overlay_msg)
        self.cache_overlay_image = image

    def publish_warped_image(self, image: np.ndarray):
        warped_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        warped_msg.header.stamp = self.node.get_clock().now().to_msg()
        warped_msg.header.frame_id = self.world_frame_id
        self.rectified_board_image_pub.publish(warped_msg)
        self.cache_rectified_image = image

    def overlay_image_timer_callback(self):
        (
            self.publish_overlay_image(self.cache_overlay_image)
            if self.cache_overlay_image is not None
            else None
        )

    def rectified_image_timer_callback(self):
        (
            self.publish_warped_image(self.cache_rectified_image)
            if self.cache_rectified_image is not None
            else None
        )

    def masked_image_timer_callback(self):
        """Publish masked image using current HSV config."""
        if self.cv_image is None or self._hsv_config_getter is None:
            return

        try:
            hsv_config = self._hsv_config_getter()
            masked_image = self._create_masked_image(self.cv_image, hsv_config)
            self._publish_masked_image(masked_image)
        except Exception as e:
            self.node.get_logger().debug(f"Error in masked_image_timer_callback: {e}")

    def _create_masked_image(
        self, img: np.ndarray, hsv_config: "HSVConfig"
    ) -> np.ndarray:
        """Create a combined masked image showing all color detections."""
        masks = build_colour_masks(img, hsv_config)

        # Create colored overlay image
        result = img.copy()

        # Apply colored overlays for each detected color (BGR format)
        result[masks["red"] > 0] = [0, 0, 255]
        result[masks["yellow"] > 0] = [0, 255, 255]
        result[masks["green"] > 0] = [0, 255, 0]
        result[masks["blue"] > 0] = [255, 0, 0]

        # Blend with original for semi-transparent effect
        alpha = 0.6
        blended = cv2.addWeighted(result, alpha, img, 1 - alpha, 0)

        return blended

    def _publish_masked_image(self, image: np.ndarray):
        """Publish the masked image."""
        masked_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        masked_msg.header.stamp = self.node.get_clock().now().to_msg()
        masked_msg.header.frame_id = self.world_frame_id
        self.masked_image_pub.publish(masked_msg)
