from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, List, Optional, cast

import cv2
import numpy as np
import pyrealsense2 as rs
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_geometry_msgs  # noqa: F401

from ..services.aruco_detector import TARGET_IDS, ArucoInfo, ArucoInfoList
from ..services.transformation_math import (
    average_quaternions,
    depth_pixel_to_point,
    rotation_matrix_to_quaternion,
    compute_marker_pose_from_depth,
    align_quaternion_to_xy_plane,
)

if TYPE_CHECKING:
    from ..sense_node import Sense


@dataclass
class MarkerHistory:
    """Smoothing / bootstrap state for a single marker."""

    locked: bool = False
    positions: List[np.ndarray] = field(default_factory=list)  # each shape (3,)
    quaternions: List[List[float]] = field(default_factory=list)  # [x,y,z,w]
    bootstrap_count: int = 0


class TransformationManager:
    """
    Manages ArUco marker transformations and TF broadcasting.
    Public APIs:
    ```python
    TransformationManager.update_markers(...) -> dict[int, PoseStamped]
    TransformationManager.get_board_pose_in_base(...) -> Optional[PoseStamped]
    ```
    """

    TF_RESOLUTION_S = 0.1  # seconds
    TF_LOOKUP_TIMEOUT_DURATION = Duration(
        seconds=1
    )  # 1s - allow time for TF tree to populate
    TF_TRANSFORM_TIMEOUT_DURATION = Duration(nanoseconds=200_000_000)  # 0.2s

    def __init__(
        self,
        node: "Sense",
        *,
        base_frame: str,
        camera_frame: str,
        board_frame: str,
        board_marker_length_m: float,
        board_ref_marker_id: int,
        board_offset_x_m: float,
        board_offset_y_m: float,
        aruco_offset_x_m: float = 0.0,
        aruco_offset_y_m: float = 0.0,
        aruco_offset_z_m: float = 0.0,
        board_rotation_offset_deg: float = 0.0,
        bootstrap_samples: int = 15,
        bootstrap_spread_thresh: float = 0.01,
        outlier_thresh: float = 0.02,
    ) -> None:
        self.node = node
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.board_frame = board_frame
        self.board_marker_length_m = board_marker_length_m
        self.board_ref_marker_id = board_ref_marker_id
        self.board_offset_x_m = board_offset_x_m
        self.board_offset_y_m = board_offset_y_m

        # ArUco frame correction offsets (applied in base_link frame)
        self.aruco_offset_x_m = aruco_offset_x_m
        self.aruco_offset_y_m = aruco_offset_y_m
        self.aruco_offset_z_m = aruco_offset_z_m

        # Board rotation offset (degrees around Z-axis)
        self.board_rotation_offset_deg = board_rotation_offset_deg

        self.bootstrap_samples = bootstrap_samples
        self.bootstrap_spread_thresh = bootstrap_spread_thresh
        self.outlier_thresh = outlier_thresh

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Cached transforms for periodic rebroadcast
        self._cached_tf: Dict[str, TransformStamped] = {}

        # Per-marker smoothing state
        self._marker_hist: Dict[int, MarkerHistory] = {
            mid: MarkerHistory() for mid in TARGET_IDS
        }

        # Timer to keep TF tree alive in RViz
        self._tf_timer = self.node.create_timer(
            self.TF_RESOLUTION_S, self._tf_timer_callback
        )

    def wait_for_base_tf(self, timeout_sec: float = 5.0) -> bool:
        """
        Wait for the base TF tree (base_frame -> camera_frame) to become available.
        Should be called before attempting board capture.

        Returns True if the transform is available, False if timed out.
        """
        try:
            can_transform = self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                Time(seconds=0),
                timeout=Duration(seconds=int(timeout_sec)),
            )
            if can_transform:
                self.node.get_logger().info(
                    f"TF tree ready: {self.base_frame} <- {self.camera_frame}"
                )
                return True
            else:
                self.node.get_logger().warn(
                    f"TF tree not available after {timeout_sec}s: "
                    f"{self.base_frame} <- {self.camera_frame}"
                )
                return False
        except Exception as e:
            self.node.get_logger().warn(f"Error waiting for base TF: {e}")
            return False

    def update_markers(
        self,
        *,
        aruco_info: ArucoInfoList,
        depth_image: Optional[np.ndarray],
        intrinsics: Optional[rs.intrinsics],
        stamp: Time,
    ) -> Dict[int, PoseStamped]:
        """
        Main entrypoint called from the sensing pipeline.

        Uses depth camera to compute all ArUco marker poses for better accuracy.

        Returns: mapping marker_id -> PoseStamped (in camera_frame)

        Raises:
            RuntimeError: If intrinsics or depth image are not available
        """
        marker_poses_cam: Dict[int, PoseStamped] = {}

        if aruco_info is None:
            raise RuntimeError(
                "aruco_info is None. This should never happen in normal flow. "
                "Check that detect_aruco_markers() is being called correctly."
            )

        if intrinsics is None:
            raise RuntimeError(
                "Camera intrinsics not available. "
                "Ensure the camera info topic is publishing "
                f"(expected: /camera/camera/aligned_depth_to_color/camera_info)"
            )

        if depth_image is None:
            raise RuntimeError(
                "Depth image not available. "
                "Ensure the depth camera is publishing to the aligned depth topic."
            )

        # Keep camera matrices for fallback PnP if depth fails
        camera_matrix, dist_coeffs = self._build_camera_matrices(intrinsics)

        for marker_id in TARGET_IDS:
            info = aruco_info.get_by_id(marker_id)
            if info is None:
                continue

            # Try depth-based pose computation first (more accurate)
            depth_result = compute_marker_pose_from_depth(
                info.corners,
                depth_image,
                intrinsics,
                self.board_marker_length_m,
            )

            if depth_result is not None:
                t_raw, quat_raw = depth_result
                self.node.get_logger().debug(
                    f"[marker {marker_id}] Using depth-based pose: "
                    f"pos=({t_raw[0]:.4f}, {t_raw[1]:.4f}, {t_raw[2]:.4f})"
                )
            else:
                # Fallback to PnP if depth fails
                self.node.get_logger().warn(
                    f"[marker {marker_id}] Depth-based pose failed, falling back to PnP"
                )
                r_mat, t_raw, quat_raw = self._solve_pnp_for_marker(
                    info.corners,
                    camera_matrix,
                    dist_coeffs,
                    self.board_marker_length_m,
                )
                if r_mat is None or t_raw is None or quat_raw is None:
                    continue

            pos_smooth, quat_smooth = self._smooth_marker_pose(
                marker_id, t_raw, quat_raw
            )

            pose_cam = self._build_pose_stamped(
                pos_smooth, quat_smooth, stamp, self.camera_frame
            )
            marker_poses_cam[marker_id] = pose_cam

            self.broadcast_marker_tf(marker_id, pose_cam)

            # Log depth vs PnP comparison for debugging (only for ref marker)
            if marker_id == self.board_ref_marker_id:
                self._debug_log_marker_pose_in_base(
                    info, depth_image, intrinsics, stamp
                )

        # Validate that at least some markers were processed
        if not marker_poses_cam:
            self.node.get_logger().warn(
                "[update_markers] No marker poses computed. "
                "Depth-based computation and PnP fallback failed for all markers."
            )

        # Always try to publish board frame if we have the ref marker
        if self.board_ref_marker_id in marker_poses_cam:
            self.broadcast_tf_board(
                marker_id=self.board_ref_marker_id,
                stamp=stamp,
            )

        return marker_poses_cam

    def _tf_timer_callback(self) -> None:
        """Re-broadcast cached transforms so TF frames remain visible."""
        if not self._cached_tf:
            return
        now = self.node.get_clock().now().to_msg()
        for tf_msg in self._cached_tf.values():
            tf_msg.header.stamp = now
            self.tf_broadcaster.sendTransform(tf_msg)

    def broadcast_marker_tf(self, marker_id: int, pose_cam: PoseStamped) -> None:
        """
        Publish transform base_frame -> aruco_<marker_id>.

        The marker pose is first transformed to base_frame, then ArUco offset
        corrections are applied (configurable in yaml). The orientation is
        adjusted so that the ArUco frame's XY plane is parallel to base_link's
        XY plane (Z-axis up), regardless of the computed depth transformation.
        """
        # Transform marker pose to base frame
        try:
            pose_base = cast(
                PoseStamped,
                self.tf_buffer.transform(
                    pose_cam,
                    self.base_frame,
                    timeout=self.TF_TRANSFORM_TIMEOUT_DURATION,
                ),
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"[tf] Failed to transform marker {marker_id} to base frame: {e}"
            )
            # Fallback: broadcast in camera frame without offset correction
            tf_msg = TransformStamped()
            tf_msg.header.stamp = pose_cam.header.stamp
            tf_msg.header.frame_id = pose_cam.header.frame_id
            tf_msg.child_frame_id = f"aruco_{marker_id}"
            tf_msg.transform.translation.x = pose_cam.pose.position.x
            tf_msg.transform.translation.y = pose_cam.pose.position.y
            tf_msg.transform.translation.z = pose_cam.pose.position.z
            tf_msg.transform.rotation = pose_cam.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)
            self._cached_tf[tf_msg.child_frame_id] = tf_msg
            return

        # Apply ArUco offset corrections in base frame
        corrected_x = pose_base.pose.position.x + self.aruco_offset_x_m
        corrected_y = pose_base.pose.position.y + self.aruco_offset_y_m
        corrected_z = pose_base.pose.position.z + self.aruco_offset_z_m

        # Align orientation so XY plane is parallel to base_link XY plane
        # This ensures the ArUco frame is always horizontal regardless of
        # any tilt detected by the depth-based pose computation
        marker_quat_base = [
            pose_base.pose.orientation.x,
            pose_base.pose.orientation.y,
            pose_base.pose.orientation.z,
            pose_base.pose.orientation.w,
        ]
        aligned_quat = align_quaternion_to_xy_plane(marker_quat_base)

        # Broadcast corrected marker pose in base frame with aligned orientation
        tf_msg = TransformStamped()
        tf_msg.header.stamp = pose_cam.header.stamp
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = f"aruco_{marker_id}"
        tf_msg.transform.translation.x = corrected_x
        tf_msg.transform.translation.y = corrected_y
        tf_msg.transform.translation.z = corrected_z
        tf_msg.transform.rotation.x = float(aligned_quat[0])
        tf_msg.transform.rotation.y = float(aligned_quat[1])
        tf_msg.transform.rotation.z = float(aligned_quat[2])
        tf_msg.transform.rotation.w = float(aligned_quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)
        self._cached_tf[tf_msg.child_frame_id] = tf_msg

    def broadcast_tf_board(self, marker_id: int, stamp: Time) -> None:
        """
        Publish transform base_frame -> board_frame.

        The board frame is positioned at the marker center + offsets,
        with orientation ALIGNED to base_link (Z-axis up, XY plane horizontal).
        This makes the board frame easier to work with for manipulation.
        """
        # Get the marker TF from cache (already in base_frame with ArUco offsets applied)
        marker_tf_key = f"aruco_{marker_id}"
        if marker_tf_key not in self._cached_tf:
            self.node.get_logger().warn(
                f"[tf] Cannot broadcast board frame: marker {marker_id} TF not cached"
            )
            return

        marker_tf = self._cached_tf[marker_tf_key]

        from scipy.spatial.transform import Rotation

        # Get marker pose in base frame (already corrected with ArUco offsets and aligned)
        marker_pos_base = np.array(
            [
                marker_tf.transform.translation.x,
                marker_tf.transform.translation.y,
                marker_tf.transform.translation.z,
            ]
        )
        marker_quat_base = [
            marker_tf.transform.rotation.x,
            marker_tf.transform.rotation.y,
            marker_tf.transform.rotation.z,
            marker_tf.transform.rotation.w,
        ]

        # Get the marker rotation matrix in base frame (already aligned with base_link XY plane)
        marker_rot = Rotation.from_quat(marker_quat_base)
        marker_rot_mat = marker_rot.as_matrix()

        # Apply offset in marker's local frame to get board position
        local_offset = np.array([self.board_offset_x_m, self.board_offset_y_m, 0.0])
        board_pos_base = marker_pos_base + marker_rot_mat @ local_offset

        # Board orientation: marker is already aligned (Z-up), apply rotation offset
        marker_x_axis = marker_rot_mat[
            :, 0
        ]  # First column is X-axis (already in XY plane)

        # Apply rotation offset around Z-axis
        rotation_offset_rad = np.deg2rad(self.board_rotation_offset_deg)
        cos_offset = np.cos(rotation_offset_rad)
        sin_offset = np.sin(rotation_offset_rad)

        # Rotate the X-axis by the offset angle around Z
        board_x_axis = np.array(
            [
                cos_offset * marker_x_axis[0] - sin_offset * marker_x_axis[1],
                sin_offset * marker_x_axis[0] + cos_offset * marker_x_axis[1],
                0.0,  # Ensure Z is 0 for XY plane alignment
            ]
        )
        board_x_axis = board_x_axis / np.linalg.norm(board_x_axis)

        # Z-axis is always up (aligned with base_link)
        board_z_axis = np.array([0.0, 0.0, 1.0])

        # Y-axis = Z cross X (right-hand rule)
        board_y_axis = np.cross(board_z_axis, board_x_axis)
        board_y_axis = board_y_axis / np.linalg.norm(board_y_axis)

        # Build rotation matrix (columns are axis vectors)
        board_rot_mat = np.column_stack([board_x_axis, board_y_axis, board_z_axis])
        board_rot = Rotation.from_matrix(board_rot_mat)
        board_quat = board_rot.as_quat()  # [x, y, z, w]

        # Broadcast board frame as child of base_frame
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.board_frame

        tf_msg.transform.translation.x = float(board_pos_base[0])
        tf_msg.transform.translation.y = float(board_pos_base[1])
        tf_msg.transform.translation.z = float(board_pos_base[2])

        tf_msg.transform.rotation.x = float(board_quat[0])
        tf_msg.transform.rotation.y = float(board_quat[1])
        tf_msg.transform.rotation.z = float(board_quat[2])
        tf_msg.transform.rotation.w = float(board_quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)
        self._cached_tf[self.board_frame] = tf_msg

        self.node.get_logger().info(
            f"Published TF: {self.base_frame} -> {self.board_frame} "
            f"(pos: [{board_pos_base[0]:.3f}, {board_pos_base[1]:.3f}, {board_pos_base[2]:.3f}])"
        )

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: Optional[Time] = None
    ) -> Optional[TransformStamped]:
        """
        Lookup transform between frames.
        Returns None if unavailable.

        Note: If time is None, uses Time(seconds=0) to get the latest available
        transform rather than the current time, which avoids extrapolation issues.
        """
        # Use Time(seconds=0) for "latest available" to avoid timing issues
        # when looking up freshly published transforms
        lookup_time = time if time is not None else Time(seconds=0)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_time,
                timeout=self.TF_LOOKUP_TIMEOUT_DURATION,
            )
            return tf_msg

        except Exception as e:
            self.node.get_logger().warn(
                f"[tf] Failed to lookup {target_frame} <- {source_frame}: {e}"
            )
            return None

    def get_board_pose_in_base(
        self,
        base_frame: str,
        board_frame: str,
        stamp: Optional[Time] = None,
        wait_for_transform: bool = False,
    ) -> Optional[PoseStamped]:
        """
        Returns the board pose expressed in base frame coordinates.

        - Looks for transform: base_link <- board
        - Returns PoseStamped or None if TF unavailable

        Args:
            base_frame: Target frame (e.g., "base_link")
            board_frame: Source frame (e.g., "klotski_board")
            stamp: Time for lookup. If None, uses latest available transform.
            wait_for_transform: If True, wait for the transform to become available.
        """
        # Use Time(seconds=0) for "latest available" when stamp is None
        lookup_time = stamp if stamp is not None else Time(seconds=0)

        if wait_for_transform:
            # Wait for the transform to become available
            # Use a longer timeout and retry logic for robustness
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    can_transform = self.tf_buffer.can_transform(
                        base_frame,
                        board_frame,
                        lookup_time,
                        timeout=self.TF_LOOKUP_TIMEOUT_DURATION,
                    )
                    if can_transform:
                        break
                    self.node.get_logger().debug(
                        f"[tf] Attempt {attempt + 1}/{max_retries}: Transform not yet available"
                    )
                except Exception as e:
                    self.node.get_logger().debug(
                        f"[tf] Attempt {attempt + 1}/{max_retries}: {e}"
                    )
            else:
                self.node.get_logger().warn(
                    f"[tf] Timed out waiting for transform {base_frame} <- {board_frame} "
                    f"after {max_retries} attempts"
                )
                return None

        tf_msg = self.lookup_transform(
            target_frame=base_frame, source_frame=board_frame, time=stamp
        )

        if tf_msg is None:
            # already logged in lookup_transform
            return None

        # Convert TransformStamped → PoseStamped
        pose = PoseStamped()
        pose.header = tf_msg.header
        pose.header.frame_id = base_frame

        pose.pose.position.x = tf_msg.transform.translation.x
        pose.pose.position.y = tf_msg.transform.translation.y
        pose.pose.position.z = tf_msg.transform.translation.z

        pose.pose.orientation = tf_msg.transform.rotation
        return pose

    def compute_board_pose_in_base(
        self,
        marker_pose_cam: PoseStamped,
    ) -> Optional[PoseStamped]:
        """
        Compute the board pose in base frame directly from marker pose.

        This avoids the race condition of waiting for our own published
        klotski_board frame to appear in the TF buffer.

        The board is offset from the marker by (board_offset_x_m, board_offset_y_m)
        in the marker's local frame.

        The board orientation is ALIGNED with base_link:
        - Z-axis points up (same as base_link Z)
        - X-axis is the marker's X projected onto the XY plane
        - Y-axis completes the right-handed system

        Args:
            marker_pose_cam: The reference marker pose in camera frame

        Returns:
            PoseStamped of board in base frame, or None if transform fails
        """
        from scipy.spatial.transform import Rotation

        # First transform marker pose to base frame
        try:
            marker_base = cast(
                PoseStamped,
                self.tf_buffer.transform(
                    marker_pose_cam,
                    self.base_frame,
                    timeout=self.TF_LOOKUP_TIMEOUT_DURATION,
                ),
            )
        except Exception as e:
            self.node.get_logger().warn(
                f"[tf] Failed to transform marker pose to {self.base_frame}: {e}"
            )
            return None

        # Get marker pose in base frame
        marker_pos_base = np.array(
            [
                marker_base.pose.position.x,
                marker_base.pose.position.y,
                marker_base.pose.position.z,
            ]
        )
        marker_quat_base = [
            marker_base.pose.orientation.x,
            marker_base.pose.orientation.y,
            marker_base.pose.orientation.z,
            marker_base.pose.orientation.w,
        ]

        # Align orientation so XY plane is parallel to base_link XY plane
        aligned_quat = align_quaternion_to_xy_plane(marker_quat_base)

        # Get the aligned rotation matrix in base frame
        aligned_rot = Rotation.from_quat(aligned_quat)
        aligned_rot_mat = aligned_rot.as_matrix()

        # Apply offset in marker's local frame to get board position
        local_offset = np.array([self.board_offset_x_m, self.board_offset_y_m, 0.0])
        board_pos_base = marker_pos_base + aligned_rot_mat @ local_offset

        # Build board pose in base frame (already aligned)
        board_base = PoseStamped()
        board_base.header.stamp = marker_pose_cam.header.stamp
        board_base.header.frame_id = self.base_frame
        board_base.pose.position.x = float(board_pos_base[0])
        board_base.pose.position.y = float(board_pos_base[1])
        board_base.pose.position.z = float(board_pos_base[2])
        board_base.pose.orientation.x = float(aligned_quat[0])
        board_base.pose.orientation.y = float(aligned_quat[1])
        board_base.pose.orientation.z = float(aligned_quat[2])
        board_base.pose.orientation.w = float(aligned_quat[3])

        return board_base

    @staticmethod
    def _build_camera_matrices(
        intrinsics: Optional[rs.intrinsics],
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Build camera matrix and distortion coefficients from RealSense intrinsics.
        Returns a tuple of (camera_matrix, dist_coeffs), or (None, None) if intrinsics is None.
        """
        if intrinsics is None:
            return None, None
        camera_matrix = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)
        return camera_matrix, dist_coeffs

    @staticmethod
    def _solve_pnp_for_marker(
        corners: np.ndarray,
        camera_matrix: Optional[np.ndarray],
        dist_coeffs: Optional[np.ndarray],
        marker_length_m: float,
    ) -> tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[List[float]]]:
        """
        Solve PnP for a single ArUco marker.
        Returns rotation matrix, translation vector, quaternion.
        Any of the outputs may be None if PnP fails.
        """
        if camera_matrix is None or dist_coeffs is None:
            return None, None, None

        MarkerSideLength = float(marker_length_m)
        obj_points = np.array(
            [
                [-MarkerSideLength / 2, MarkerSideLength / 2, 0.0],
                [MarkerSideLength / 2, MarkerSideLength / 2, 0.0],
                [MarkerSideLength / 2, -MarkerSideLength / 2, 0.0],
                [-MarkerSideLength / 2, -MarkerSideLength / 2, 0.0],
            ],
            dtype=np.float32,
        )

        img_points = corners.astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs
        )
        if not success:
            return None, None, None

        R, _ = cv2.Rodrigues(rvec)
        quat = rotation_matrix_to_quaternion(R)

        t_vec = np.array(
            [float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])],
            dtype=float,
        )
        return R, t_vec, quat

    def _smooth_marker_pose(
        self,
        marker_id: int,
        position_raw: np.ndarray,
        quat_raw: List[float],
    ) -> tuple[np.ndarray, List[float]]:
        """
        Bootstrap-then-lock smoothing:
        * during bootstrap: collect samples, check spread, then lock
        * when locked: reject large outliers and average pose
        """
        hist = self._marker_hist[marker_id]

        if not hist.locked:
            hist.positions.append(position_raw)
            hist.quaternions.append(quat_raw)
            hist.bootstrap_count += 1

            if hist.bootstrap_count >= self.bootstrap_samples:
                arr = np.stack(hist.positions)
                mean_pos = np.mean(arr, axis=0)
                dists = np.linalg.norm(arr - mean_pos, axis=1)
                spread = float(np.max(dists))

                if spread < self.bootstrap_spread_thresh:
                    hist.locked = True
                else:
                    # reset bootstrap
                    hist.positions.clear()
                    hist.quaternions.clear()
                    hist.bootstrap_count = 0

            # while bootstrapping, just use raw pose
            return position_raw, quat_raw

        # locked phase
        if hist.positions:
            prev_avg = np.mean(np.stack(hist.positions), axis=0)
            dist = float(np.linalg.norm(position_raw - prev_avg))
            if dist > self.outlier_thresh:
                # outlier: do not add, just reuse previous average
                avg_pos = prev_avg
                avg_quat = average_quaternions(hist.quaternions)
                return avg_pos, avg_quat

        hist.positions.append(position_raw)
        hist.quaternions.append(quat_raw)
        avg_pos = np.mean(np.stack(hist.positions), axis=0)
        avg_quat = average_quaternions(hist.quaternions)
        return avg_pos, avg_quat

    @staticmethod
    def _build_pose_stamped(
        position: np.ndarray,
        quaternion: List[float],
        stamp: Time,
        frame_id: str,
    ) -> PoseStamped:
        """
        Build PoseStamped message from position + quaternion.
        """
        pose = PoseStamped()
        pose.header.stamp = stamp.to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        return pose

    def _debug_log_marker_pose_in_base(
        self,
        info: ArucoInfo,
        depth_image: np.ndarray,
        intrinsics: rs.intrinsics,
        stamp: Time,
    ) -> None:
        """
        For debugging: log the depth-based marker pose transformed to base frame.
        """
        try:
            center = info.center
            u, v = int(center[0]), int(center[1])

            depth_pt = depth_pixel_to_point(depth_image, intrinsics, u, v)
            if depth_pt is None:
                return

            Xd, Yd, Zd = depth_pt
            depth_cam = PoseStamped()
            depth_cam.header.stamp = stamp.to_msg()
            depth_cam.header.frame_id = self.camera_frame
            depth_cam.pose.position.x = Xd
            depth_cam.pose.position.y = Yd
            depth_cam.pose.position.z = Zd
            depth_cam.pose.orientation.w = 1.0  # others 0 by default

            depth_base = cast(
                PoseStamped,
                self.tf_buffer.transform(
                    depth_cam,
                    self.base_frame,
                    timeout=self.TF_TRANSFORM_TIMEOUT_DURATION,
                ),
            )

            self.node.get_logger().info(
                f"[marker {self.board_ref_marker_id}] "
                f"{self.base_frame} depth: "
                f"x={depth_base.pose.position.x:.3f}, "
                f"y={depth_base.pose.position.y:.3f}, "
                f"z={depth_base.pose.position.z:.3f}"
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(
                f"Failed depth debug for marker {self.board_ref_marker_id}: {exc}"
            )
