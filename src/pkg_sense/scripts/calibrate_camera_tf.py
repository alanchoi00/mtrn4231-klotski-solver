#!/usr/bin/env python3
"""
Camera TF Calibration Tool

This script helps calibrate the base_link -> camera_link transform by:
1. Detecting ArUco markers directly from the RealSense camera (no sense node needed)
2. Reading robot EE position from TF
3. Computing the optimal camera transform

Workflow:
  1. Start robot driver only (no need for full klotski system)
  2. Run this script in collect mode
  3. Position robot gripper at known marker positions
  4. Script computes optimal camera transform
  5. Use the output values in runKlotski.sh

Requirements:
  - Robot driver running (for EE position)
  - RealSense camera connected
  - ArUco markers visible to camera
"""

import argparse
import json
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ROS imports
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


@dataclass
class CalibrationSample:
    """A single calibration sample."""

    # Robot end-effector pose in base_link frame (from robot FK)
    ee_in_base: np.ndarray  # [x, y, z]
    ee_quat_in_base: np.ndarray  # [x, y, z, w]

    # Marker pose in camera frame (from ArUco detection)
    marker_in_cam: np.ndarray  # [x, y, z]
    marker_quat_in_cam: np.ndarray  # [x, y, z, w]

    # Marker ID
    marker_id: int

    timestamp: float = field(default_factory=lambda: datetime.now().timestamp())


@dataclass
class MarkerDetection:
    """Detected ArUco marker info."""

    marker_id: int
    corners: np.ndarray  # shape (4, 2)
    center: np.ndarray  # shape (2,)
    position: Optional[np.ndarray] = None  # 3D position in camera frame
    quaternion: Optional[np.ndarray] = None  # orientation quaternion


class CameraCalibrator(Node):
    """ROS2 node for camera TF calibration with direct ArUco detection."""

    # ArUco configuration
    ARUCO_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
    MARKER_LENGTH_M = 0.065  # 65mm marker side length

    def __init__(self):
        super().__init__("camera_calibrator")

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("target_marker_id", 2)  # Bottom-left marker
        self.declare_parameter("marker_length_m", self.MARKER_LENGTH_M)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_marker_id = self.get_parameter("target_marker_id").value
        self.marker_length = self.get_parameter("marker_length_m").value

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params
        )

        # Camera info
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_info_received = False

        # Latest detections
        self.latest_detections: Dict[int, MarkerDetection] = {}
        self.latest_image: Optional[np.ndarray] = None
        self.detection_lock = threading.Lock()

        # CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10,
        )

        self.samples: List[CalibrationSample] = []

        self.get_logger().info(
            "Camera Calibrator initialized (standalone ArUco detection)"
        )
        self.get_logger().info(f"  Base frame: {self.base_frame}")
        self.get_logger().info(f"  EE frame: {self.ee_frame}")
        self.get_logger().info(f"  Camera frame: {self.camera_frame}")
        self.get_logger().info(f"  Target marker: {self.target_marker_id}")
        self.get_logger().info(f"  Marker length: {self.marker_length*1000:.1f} mm")

    def camera_info_callback(self, msg: CameraInfo):
        """Process camera info to get intrinsics."""
        if self.camera_info_received:
            return

        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_info_received = True
        self.get_logger().info("✓ Camera intrinsics received")

    def image_callback(self, msg: Image):
        """Process incoming image and detect ArUco markers."""
        if not self.camera_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        # Detect markers
        corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)

        with self.detection_lock:
            self.latest_image = cv_image.copy()
            self.latest_detections.clear()

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    marker_corners = corners[i][0]
                    center = np.mean(marker_corners, axis=0)

                    detection = MarkerDetection(
                        marker_id=int(marker_id),
                        corners=marker_corners,
                        center=center,
                    )

                    # Compute 3D pose using PnP
                    position, quaternion = self._solve_marker_pose(marker_corners)
                    if position is not None:
                        detection.position = position
                        detection.quaternion = quaternion

                        # Broadcast TF for this marker
                        self._broadcast_marker_tf(marker_id, position, quaternion)

                    self.latest_detections[int(marker_id)] = detection

    def _solve_marker_pose(
        self, corners: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Solve PnP to get marker pose in camera frame."""
        if self.camera_matrix is None:
            return None, None

        # 3D object points (marker corners in marker frame)
        half_size = self.marker_length / 2
        obj_points = np.array(
            [
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0],
            ],
            dtype=np.float32,
        )

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            obj_points, corners.astype(np.float32), self.camera_matrix, self.dist_coeffs
        )

        if not success:
            return None, None

        # Convert rotation vector to matrix, then to quaternion
        R, _ = cv2.Rodrigues(rvec)
        rot = Rotation.from_matrix(R)
        quat = rot.as_quat()  # [x, y, z, w]

        position = tvec.flatten()
        return position, quat

    def _broadcast_marker_tf(
        self, marker_id: int, position: np.ndarray, quaternion: np.ndarray
    ):
        """Broadcast marker TF frame."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = f"aruco_{marker_id}"

        tf_msg.transform.translation.x = float(position[0])
        tf_msg.transform.translation.y = float(position[1])
        tf_msg.transform.translation.z = float(position[2])

        tf_msg.transform.rotation.x = float(quaternion[0])
        tf_msg.transform.rotation.y = float(quaternion[1])
        tf_msg.transform.rotation.z = float(quaternion[2])
        tf_msg.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(tf_msg)

    def wait_for_camera(self, timeout_sec: float = 10.0) -> bool:
        """Wait for camera info to be received."""
        self.get_logger().info("Waiting for camera...")
        start = time.time()
        while not self.camera_info_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                self.get_logger().error("Timeout waiting for camera info")
                return False
        return True

    def wait_for_tf(self, timeout_sec: float = 10.0) -> bool:
        """Wait for TF tree to be available."""
        self.get_logger().info("Waiting for robot TF tree...")

        # Spin a few times first to let TF buffer populate
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=timeout_sec),
            )
            self.get_logger().info(f"✓ Found {self.base_frame} -> {self.ee_frame}")
            return True
        except Exception as e:
            self.get_logger().error(
                f"✗ Cannot find {self.base_frame} -> {self.ee_frame}: {e}"
            )
            return False

    def get_detected_markers(self) -> Dict[int, MarkerDetection]:
        """Get currently detected markers."""
        with self.detection_lock:
            return dict(self.latest_detections)

    def get_ee_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Get current EE pose in base frame."""
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            pos = np.array(
                [
                    ee_tf.transform.translation.x,
                    ee_tf.transform.translation.y,
                    ee_tf.transform.translation.z,
                ]
            )
            quat = np.array(
                [
                    ee_tf.transform.rotation.x,
                    ee_tf.transform.rotation.y,
                    ee_tf.transform.rotation.z,
                    ee_tf.transform.rotation.w,
                ]
            )
            return pos, quat
        except Exception as e:
            self.get_logger().warn(f"Cannot get EE pose: {e}")
            return None

    def collect_sample(self, marker_id: int) -> Optional[CalibrationSample]:
        """Collect a calibration sample for specified marker."""
        # Get EE pose
        ee_result = self.get_ee_pose()
        if ee_result is None:
            self.get_logger().warn("Cannot get EE pose")
            return None

        ee_pos, ee_quat = ee_result

        # Get marker detection
        detections = self.get_detected_markers()
        if marker_id not in detections:
            self.get_logger().warn(f"Marker {marker_id} not detected")
            return None

        detection = detections[marker_id]
        if detection.position is None:
            self.get_logger().warn(f"Marker {marker_id} has no 3D pose")
            return None

        sample = CalibrationSample(
            ee_in_base=ee_pos,
            ee_quat_in_base=ee_quat,
            marker_in_cam=detection.position,
            marker_quat_in_cam=detection.quaternion,
            marker_id=marker_id,
        )

        self.get_logger().info(
            f"Sample: EE=({ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}), "
            f"Marker {marker_id}=({detection.position[0]:.4f}, {detection.position[1]:.4f}, {detection.position[2]:.4f})"
        )

        return sample

    def show_detection_preview(self, duration_sec: float = 0.0):
        """Show camera view with detected markers (optional)."""
        with self.detection_lock:
            if self.latest_image is None:
                return

            img = self.latest_image.copy()

            for mid, det in self.latest_detections.items():
                # Draw marker outline
                pts = det.corners.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(img, [pts], True, (0, 255, 0), 2)

                # Draw center and ID
                cx, cy = int(det.center[0]), int(det.center[1])
                cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    img,
                    f"ID:{mid}",
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2,
                )

                # Draw distance if available
                if det.position is not None:
                    dist = np.linalg.norm(det.position)
                    cv2.putText(
                        img,
                        f"{dist:.3f}m",
                        (cx + 10, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2,
                    )

        cv2.imshow("Calibration - ArUco Detection", img)
        if duration_sec > 0:
            cv2.waitKey(int(duration_sec * 1000))
        else:
            cv2.waitKey(1)


def compute_transform_from_samples(
    samples: List[CalibrationSample],
    known_marker_offset: np.ndarray = np.array([0.0, 0.0, 0.0]),
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute optimal base_link -> camera_link transform from calibration samples.
    """
    if len(samples) < 3:
        raise ValueError("Need at least 3 samples for calibration")

    def residuals(params):
        tx, ty, tz, qx, qy, qz, qw = params

        # Normalize quaternion
        q_norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx, qy, qz, qw = qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm

        # Build transform matrix: base_link -> camera_link
        rot = Rotation.from_quat([qx, qy, qz, qw])
        T_base_cam = np.eye(4)
        T_base_cam[:3, :3] = rot.as_matrix()
        T_base_cam[:3, 3] = [tx, ty, tz]

        errors = []
        for sample in samples:
            ee_rot = Rotation.from_quat(sample.ee_quat_in_base)
            expected_marker = sample.ee_in_base + ee_rot.apply(known_marker_offset)

            marker_cam = np.append(sample.marker_in_cam, 1.0)
            marker_base = T_base_cam @ marker_cam

            error = expected_marker - marker_base[:3]
            errors.extend(error.tolist())

        return errors

    # Initial guess
    x0 = [1.30938, 0.0206053, 0.670571, -0.398486, 0.00254305, 0.917119, 0.00974536]

    result = least_squares(residuals, x0, method="lm")

    tx, ty, tz, qx, qy, qz, qw = result.x
    q_norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)

    translation = np.array([tx, ty, tz])
    quaternion = np.array([qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm])

    return translation, quaternion


def generate_launch_command(translation: np.ndarray, quaternion: np.ndarray) -> str:
    """Generate the ros2 run command for the static transform publisher."""
    return (
        f"ros2 run tf2_ros static_transform_publisher "
        f"{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f} "
        f"{quaternion[0]:.6f} {quaternion[1]:.6f} {quaternion[2]:.6f} {quaternion[3]:.6f} "
        f"base_link camera_link"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Camera TF Calibration Tool (standalone ArUco detection)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Check if markers are visible (no robot needed)
  python3 calibrate_camera_tf.py --mode check-markers

  # Collect calibration samples (robot driver must be running)
  python3 calibrate_camera_tf.py --mode collect --num-samples 5

  # Compute optimal transform from collected samples
  python3 calibrate_camera_tf.py --mode compute

  # Show live camera preview with marker detection
  python3 calibrate_camera_tf.py --mode preview
        """,
    )
    parser.add_argument(
        "--mode",
        choices=["check-markers", "collect", "compute", "preview"],
        default="check-markers",
        help="Mode of operation",
    )
    parser.add_argument(
        "--samples-file",
        type=str,
        default="calibration_samples.json",
        help="File to save/load calibration samples",
    )
    parser.add_argument(
        "--num-samples",
        type=int,
        default=5,
        help="Number of samples to collect",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=2,
        help="Target marker ID for calibration (default: 2 = bottom-left)",
    )
    parser.add_argument(
        "--show-preview",
        action="store_true",
        help="Show camera preview window during collection",
    )
    args = parser.parse_args()

    rclpy.init()
    node = CameraCalibrator()

    try:
        # ============ CHECK-MARKERS MODE ============
        if args.mode == "check-markers":
            node.get_logger().info("\n=== CHECK MARKERS MODE ===")
            node.get_logger().info("Detecting ArUco markers directly from camera...")
            node.get_logger().info("(No sense node or robot driver needed)\n")

            if not node.wait_for_camera(timeout_sec=10.0):
                node.get_logger().error("Camera not available. Is RealSense connected?")
                return 1

            # Spin to detect markers
            node.get_logger().info("Scanning for markers...")
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.1)

            detections = node.get_detected_markers()

            marker_ids = [0, 1, 2, 3]
            marker_names = [
                "TL (top-left)",
                "TR (top-right)",
                "BL (bottom-left)",
                "BR (bottom-right)",
            ]

            node.get_logger().info("")
            for mid, mname in zip(marker_ids, marker_names):
                if mid in detections:
                    det = detections[mid]
                    if det.position is not None:
                        pos = det.position
                        dist = np.linalg.norm(pos)
                        node.get_logger().info(
                            f"✓ Marker {mid} {mname}: "
                            f"({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m, "
                            f"distance: {dist:.3f} m"
                        )
                    else:
                        node.get_logger().warn(
                            f"⚠ Marker {mid} {mname}: detected but no 3D pose"
                        )
                else:
                    node.get_logger().warn(f"✗ Marker {mid} {mname}: NOT VISIBLE")

            found = len([m for m in marker_ids if m in detections])
            node.get_logger().info(f"\nFound {found}/4 markers")

            if found >= 1:
                node.get_logger().info("✓ Ready for calibration")
                return 0
            else:
                node.get_logger().error("✗ No markers found - check camera view")
                return 1

        # ============ PREVIEW MODE ============
        elif args.mode == "preview":
            node.get_logger().info("\n=== PREVIEW MODE ===")
            node.get_logger().info("Showing live camera view with marker detection...")
            node.get_logger().info("Press 'q' to quit\n")

            if not node.wait_for_camera(timeout_sec=10.0):
                return 1

            while True:
                rclpy.spin_once(node, timeout_sec=0.05)
                node.show_detection_preview()
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            cv2.destroyAllWindows()
            return 0

        # ============ COLLECT MODE ============
        elif args.mode == "collect":
            node.get_logger().info("\n=== COLLECT MODE ===")
            node.get_logger().info(
                f"Will collect {args.num_samples} samples using marker {args.marker_id}"
            )
            node.get_logger().info(
                "Position EE directly above the marker center, press Enter to sample\n"
            )

            if not node.wait_for_camera(timeout_sec=10.0):
                return 1

            if not node.wait_for_tf(timeout_sec=10.0):
                node.get_logger().error(
                    "Robot TF not available. Is the robot driver running?"
                )
                return 1

            samples = []
            i = 0
            while i < args.num_samples:
                # Show preview if requested
                if args.show_preview:
                    for _ in range(10):
                        rclpy.spin_once(node, timeout_sec=0.05)
                        node.show_detection_preview()

                input(
                    f"\n[{i+1}/{args.num_samples}] Position EE above marker {args.marker_id} and press Enter..."
                )

                # Spin to get latest data
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if args.show_preview:
                        node.show_detection_preview()

                sample = node.collect_sample(args.marker_id)
                if sample:
                    samples.append(sample)
                    node.get_logger().info(f"✓ Sample {i+1} collected")
                    i += 1
                else:
                    node.get_logger().warn(
                        f"✗ Sample failed - make sure marker {args.marker_id} is visible"
                    )

            # Save samples
            samples_data = [
                {
                    "ee_in_base": s.ee_in_base.tolist(),
                    "ee_quat_in_base": s.ee_quat_in_base.tolist(),
                    "marker_in_cam": s.marker_in_cam.tolist(),
                    "marker_quat_in_cam": s.marker_quat_in_cam.tolist(),
                    "marker_id": s.marker_id,
                    "timestamp": s.timestamp,
                }
                for s in samples
            ]

            with open(args.samples_file, "w") as f:
                json.dump(samples_data, f, indent=2)

            node.get_logger().info(
                f"\n✓ Saved {len(samples)} samples to {args.samples_file}"
            )
            node.get_logger().info(
                f"Run with --mode compute to calculate the optimal transform"
            )

            if args.show_preview:
                cv2.destroyAllWindows()

            return 0

        # ============ COMPUTE MODE ============
        elif args.mode == "compute":
            node.get_logger().info("\n=== COMPUTE MODE ===")

            samples_path = Path(args.samples_file)
            if not samples_path.exists():
                node.get_logger().error(f"Samples file not found: {args.samples_file}")
                node.get_logger().info(
                    "Run with --mode collect first to gather samples"
                )
                return 1

            with open(samples_path) as f:
                samples_data = json.load(f)

            samples = [
                CalibrationSample(
                    ee_in_base=np.array(s["ee_in_base"]),
                    ee_quat_in_base=np.array(s["ee_quat_in_base"]),
                    marker_in_cam=np.array(s["marker_in_cam"]),
                    marker_quat_in_cam=np.array(s["marker_quat_in_cam"]),
                    marker_id=s["marker_id"],
                    timestamp=s.get("timestamp", 0),
                )
                for s in samples_data
            ]

            node.get_logger().info(f"Loaded {len(samples)} samples")

            if len(samples) < 3:
                node.get_logger().error("Need at least 3 samples for calibration")
                return 1

            translation, quaternion = compute_transform_from_samples(samples)

            node.get_logger().info("\n" + "=" * 50)
            node.get_logger().info("COMPUTED CAMERA TRANSFORM")
            node.get_logger().info("=" * 50)
            node.get_logger().info(
                f"Translation: x={translation[0]:.6f}, y={translation[1]:.6f}, z={translation[2]:.6f}"
            )
            node.get_logger().info(
                f"Quaternion:  x={quaternion[0]:.6f}, y={quaternion[1]:.6f}, z={quaternion[2]:.6f}, w={quaternion[3]:.6f}"
            )

            node.get_logger().info("\n" + "-" * 50)
            node.get_logger().info("COPY THIS COMMAND:")
            node.get_logger().info("-" * 50)
            cmd = generate_launch_command(translation, quaternion)
            print(f"\n{cmd}\n")

            node.get_logger().info("-" * 50)
            node.get_logger().info("OR USE WITH runKlotski.sh:")
            node.get_logger().info("-" * 50)
            print(
                f"\n./runKlotski.sh "
                f"{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f} "
                f"{quaternion[0]:.6f} {quaternion[1]:.6f} {quaternion[2]:.6f} {quaternion[3]:.6f}\n"
            )

            return 0

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
