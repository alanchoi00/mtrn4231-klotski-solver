"""ROS2 node for hand detection-based robot safety stopping."""

from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from klotski_interfaces.srv import GetSafetyZone, SetSafetyZone
from klotski_utils import declare_param
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .services.hand_detector import HandDetector, HandResult
from .services.safety_logic import SafetyLogic


class HandSafetyNode(Node):
    """
    ROS2 node that monitors camera feed for hands and publishes safety stop signals.

    Subscribes to camera images, detects hands using MediaPipe, and publishes
    a boolean stop signal. The stop is active when hands are detected and
    clears after a configurable number of consecutive frames without hands.
    """

    def __init__(self) -> None:
        super().__init__("hand_safety_monitor")

        # Declare and get parameters
        min_detection_conf = declare_param[float](
            self, "min_detection_confidence", "Minimum confidence for hand detection"
        )
        min_tracking_conf = declare_param[float](
            self, "min_tracking_confidence", "Minimum confidence for hand tracking"
        )
        clear_after_frames = declare_param[int](
            self,
            "clear_after_frames",
            "Number of consecutive frames without hands to clear safety stop",
        )
        self._enable_roi = declare_param[bool](
            self, "enable_roi", "Enable polygon ROI for hand detection"
        )
        # ROI defined by 4 points (normalized 0-1 coordinates)
        roi_p1_x = declare_param[float](self, "roi_p1_x", "ROI point 1 X (normalized)")
        roi_p1_y = declare_param[float](self, "roi_p1_y", "ROI point 1 Y (normalized)")
        roi_p2_x = declare_param[float](self, "roi_p2_x", "ROI point 2 X (normalized)")
        roi_p2_y = declare_param[float](self, "roi_p2_y", "ROI point 2 Y (normalized)")
        roi_p3_x = declare_param[float](self, "roi_p3_x", "ROI point 3 X (normalized)")
        roi_p3_y = declare_param[float](self, "roi_p3_y", "ROI point 3 Y (normalized)")
        roi_p4_x = declare_param[float](self, "roi_p4_x", "ROI point 4 X (normalized)")
        roi_p4_y = declare_param[float](self, "roi_p4_y", "ROI point 4 Y (normalized)")

        # Store ROI polygon points (normalized coordinates)
        self._roi_polygon_normalized: List[Tuple[float, float]] = [
            (roi_p1_x, roi_p1_y),
            (roi_p2_x, roi_p2_y),
            (roi_p3_x, roi_p3_y),
            (roi_p4_x, roi_p4_y),
        ]

        timer_period = declare_param[float](
            self, "timer_period_sec", "Timer period for processing in seconds"
        )

        # Initialize components
        self._hand_detector = HandDetector(
            min_detection_confidence=min_detection_conf,
            min_tracking_confidence=min_tracking_conf,
        )
        self._safety_logic = SafetyLogic(clear_after_frames=clear_after_frames)
        self._cv_bridge = CvBridge()

        # Latest image buffer (updated by subscription, processed by timer)
        self._latest_image: Optional[np.ndarray] = None
        self._image_processed: bool = True  # Start as True to avoid initial skip

        # QoS for best-effort image subscription (drop frames if needed)
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Subscriber for camera images
        self._image_sub = self.create_subscription(
            Image,
            "/image_raw",
            self._image_callback,
            image_qos,
        )

        # Publisher for safety stop signal
        self._stop_pub = self.create_publisher(
            Bool,
            "/safety/stop",
            10,
        )

        # Publisher for annotated image with hand detection overlay
        self._annotated_image_pub = self.create_publisher(
            Image,
            "/safety/hand_detection_image",
            1,
        )

        # Timer for processing (decouples from camera rate)
        self._process_timer = self.create_timer(
            timer_period,
            self._process_callback,
        )

        # Track last published state to avoid redundant publishes
        self._last_published_state: Optional[bool] = None

        # Create services for ROI management
        self._get_safety_zone_srv = self.create_service(
            GetSafetyZone, "/safety/get_zone", self._get_safety_zone_callback
        )
        self._set_safety_zone_srv = self.create_service(
            SetSafetyZone, "/safety/set_zone", self._set_safety_zone_callback
        )

        self.get_logger().info(
            f"Hand safety monitor initialized "
            f"(detection_conf={min_detection_conf}, "
            f"tracking_conf={min_tracking_conf}, "
            f"clear_frames={clear_after_frames}, "
            f"roi_enabled={self._enable_roi})"
        )

    def _image_callback(self, msg: Image) -> None:
        """
        Store the latest image for processing.

        Args:
            msg: ROS2 Image message from camera.
        """
        try:
            self._latest_image = self._cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            self._image_processed = False
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def _process_callback(self) -> None:
        """Process the latest image and publish safety state."""
        # Skip if no new image
        if self._latest_image is None or self._image_processed:
            return

        self._image_processed = True
        image = self._latest_image
        h, w = image.shape[:2]

        # Detect hands on full image
        hand_results = self._hand_detector.detect_hands(image)

        # Filter hands by ROI if enabled
        if self._enable_roi:
            roi_polygon_px = self._get_roi_polygon_pixels(w, h)
            hands_in_roi = self._filter_hands_in_roi(hand_results, roi_polygon_px, w, h)
        else:
            roi_polygon_px = None
            hands_in_roi = hand_results

        has_hands = len(hands_in_roi) > 0

        # Draw and publish annotated image (with ROI overlay)
        annotated_image = self._hand_detector.draw_annotations(image, hand_results)
        if roi_polygon_px is not None:
            annotated_image = self._draw_roi_overlay(
                annotated_image, roi_polygon_px, hands_in_roi
            )
        self._publish_annotated_image(annotated_image)

        # Log detection info
        if has_hands:
            confidences = [f"{h.confidence:.2f}" for h in hands_in_roi]
            self.get_logger().info(
                f"Detected {len(hands_in_roi)} hand(s) in ROI, "
                f"confidences: {', '.join(confidences)}"
            )

        # Update safety logic
        stop_active = self._safety_logic.update(has_hands)

        # Publish state (always publish to ensure downstream nodes stay updated)
        stop_msg = Bool()
        stop_msg.data = stop_active
        self._stop_pub.publish(stop_msg)

        # Log state changes
        if self._last_published_state != stop_active:
            state_str = "ACTIVE (STOP)" if stop_active else "CLEARED (OK)"
            self.get_logger().info(f"Safety state changed: {state_str}")
            self._last_published_state = stop_active

    def _get_roi_polygon_pixels(self, img_width: int, img_height: int) -> np.ndarray:
        """
        Convert normalized ROI polygon to pixel coordinates.

        Args:
            img_width: Image width in pixels.
            img_height: Image height in pixels.

        Returns:
            numpy array of shape (4, 2) with pixel coordinates.
        """
        points = [
            (int(p[0] * img_width), int(p[1] * img_height))
            for p in self._roi_polygon_normalized
        ]
        return np.array(points, dtype=np.int32)

    def _filter_hands_in_roi(
        self,
        hand_results: List[HandResult],
        roi_polygon_px: np.ndarray,
        img_width: int,
        img_height: int,
    ) -> List[HandResult]:
        """
        Filter hands to include those with ANY landmark inside the ROI polygon.

        This means if even a single finger or part of the hand enters the
        safety zone, the hand is considered to be in the ROI (unsafe).

        Args:
            hand_results: List of detected hands.
            roi_polygon_px: ROI polygon in pixel coordinates.
            img_width: Image width in pixels.
            img_height: Image height in pixels.

        Returns:
            List of hands with any landmark inside the ROI.
        """
        hands_in_roi = []
        for hand in hand_results:
            # Check ALL landmarks - if any is inside ROI, hand is considered in ROI
            for landmark in hand.landmarks:
                lm_x = int(landmark[0] * img_width)
                lm_y = int(landmark[1] * img_height)

                result = cv2.pointPolygonTest(
                    roi_polygon_px, (lm_x, lm_y), measureDist=False
                )
                if result >= 0:  # Inside or on edge
                    hands_in_roi.append(hand)
                    break  # No need to check more landmarks for this hand

        return hands_in_roi

    def _draw_roi_overlay(
        self,
        image: np.ndarray,
        roi_polygon_px: np.ndarray,
        hands_in_roi: List[HandResult],
    ) -> np.ndarray:
        """
        Draw the ROI polygon overlay on the image.

        Args:
            image: Input BGR image.
            roi_polygon_px: ROI polygon in pixel coordinates.
            hands_in_roi: List of hands detected inside ROI.

        Returns:
            Image with ROI overlay drawn.
        """
        annotated = image.copy()

        # Choose color based on whether hands are in ROI
        if len(hands_in_roi) > 0:
            # Red when hands detected in ROI (danger)
            color = (0, 0, 255)
            fill_color = (0, 0, 255)
        else:
            # Green when no hands in ROI (safe)
            color = (0, 255, 0)
            fill_color = (0, 255, 0)

        # Draw semi-transparent fill
        overlay = annotated.copy()
        cv2.fillPoly(overlay, [roi_polygon_px], fill_color)
        cv2.addWeighted(overlay, 0.2, annotated, 0.8, 0, annotated)

        # Draw polygon border
        cv2.polylines(
            annotated, [roi_polygon_px], isClosed=True, color=color, thickness=3
        )

        # Draw corner points
        for point in roi_polygon_px:
            cv2.circle(annotated, tuple(point), 8, color, -1)
            cv2.circle(annotated, tuple(point), 8, (255, 255, 255), 2)

        # Draw label
        label = "SAFETY ZONE"
        if len(hands_in_roi) > 0:
            label += f" - {len(hands_in_roi)} HAND(S) DETECTED!"

        # Position label at top of ROI
        label_pos = (
            int(roi_polygon_px[:, 0].mean()) - 80,
            roi_polygon_px[:, 1].min() - 10,
        )
        cv2.putText(
            annotated, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3
        )
        cv2.putText(
            annotated, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
        )

        return annotated

    def _publish_annotated_image(self, image: np.ndarray) -> None:
        """
        Publish the annotated image with hand detection overlay.

        Args:
            image: Annotated BGR image.
        """
        try:
            img_msg = self._cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_color_optical_frame"
            self._annotated_image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def _get_safety_zone_callback(
        self,
        request: GetSafetyZone.Request,
        response: GetSafetyZone.Response,
    ) -> GetSafetyZone.Response:
        """Handle GetSafetyZone service requests."""
        response.p1_x = self._roi_polygon_normalized[0][0]
        response.p1_y = self._roi_polygon_normalized[0][1]
        response.p2_x = self._roi_polygon_normalized[1][0]
        response.p2_y = self._roi_polygon_normalized[1][1]
        response.p3_x = self._roi_polygon_normalized[2][0]
        response.p3_y = self._roi_polygon_normalized[2][1]
        response.p4_x = self._roi_polygon_normalized[3][0]
        response.p4_y = self._roi_polygon_normalized[3][1]
        response.roi_enabled = self._enable_roi
        return response

    def _set_safety_zone_callback(
        self,
        request: SetSafetyZone.Request,
        response: SetSafetyZone.Response,
    ) -> SetSafetyZone.Response:
        """Handle SetSafetyZone service requests."""
        try:
            # Validate points are in 0-1 range
            points = [
                (request.p1_x, request.p1_y),
                (request.p2_x, request.p2_y),
                (request.p3_x, request.p3_y),
                (request.p4_x, request.p4_y),
            ]
            for i, (x, y) in enumerate(points):
                if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                    response.success = False
                    response.message = (
                        f"Point {i + 1} ({x}, {y}) is out of range [0, 1]"
                    )
                    return response

            # Update ROI polygon
            self._roi_polygon_normalized = points
            self._enable_roi = request.roi_enabled

            self.get_logger().info(
                f"Safety zone updated: enabled={self._enable_roi}, "
                f"points={self._roi_polygon_normalized}"
            )

            response.success = True
            response.message = "Safety zone updated successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to update safety zone: {e}"

        return response

    def destroy_node(self) -> None:
        """Clean up resources on shutdown."""
        self._hand_detector.close()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the hand safety monitor node."""
    rclpy.init(args=args)

    node = HandSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
