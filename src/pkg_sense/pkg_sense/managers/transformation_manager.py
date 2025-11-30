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

from ..services.aruco_detector import TARGET_IDS, ArucoInfoList
from ..services.transformation_math import (
    average_quaternions,
    depth_pixel_to_point,
    rotation_matrix_to_quaternion,
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
    TF_LOOKUP_TIMEOUT_DURATION = Duration(nanoseconds=200_000_000)  # 0.2s
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

        Returns: mapping marker_id -> PoseStamped (in camera_frame)
        """
        marker_poses_cam: Dict[int, PoseStamped] = {}

        if aruco_info is None:
            return marker_poses_cam

        camera_matrix, dist_coeffs = self._build_camera_matrices(intrinsics)

        for marker_id in TARGET_IDS:
            info = aruco_info.get_by_id(marker_id)
            if info is None:
                continue

            r_mat, t_raw, quat_raw = self._solve_pnp_for_marker(
                info.corners,
                camera_matrix,
                dist_coeffs,
                self.board_marker_length_m,
            )
            # ensure all outputs are present before using them
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

            if (
                marker_id == self.board_ref_marker_id
                and depth_image is not None
                and intrinsics is not None
            ):
                self._debug_log_pnp_vs_depth(info, depth_image, intrinsics, stamp)

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
        Publish transform camera_frame -> aruco_<marker_id>.
        """
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

    def broadcast_tf_board(self, marker_id: int, stamp: Time) -> None:
        """
        Publish transform aruco_<marker_id> -> board_frame, using fixed offsets.
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = f"aruco_{marker_id}"
        tf_msg.child_frame_id = self.board_frame

        tf_msg.transform.translation.x = self.board_offset_x_m
        tf_msg.transform.translation.y = self.board_offset_y_m
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_msg)
        self._cached_tf[self.board_frame] = tf_msg

        self.node.get_logger().debug(
            f"Published TF: aruco_{marker_id} -> {self.board_frame}"
        )

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: Optional[Time] = None
    ) -> Optional[TransformStamped]:
        """
        Lookup transform between frames.
        Returns None if unavailable.
        """
        time = time or self.node.get_clock().now()

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout=self.TF_LOOKUP_TIMEOUT_DURATION,
            )
            return tf_msg

        except Exception as e:
            self.node.get_logger().warn(
                f"[tf] Failed to lookup {target_frame} <- {source_frame}: {e}"
            )
            return None

    def get_board_pose_in_base(
        self, base_frame: str, board_frame: str, stamp: Time
    ) -> Optional[PoseStamped]:
        """
        Returns the board pose expressed in base frame coordinates.

        - Looks for transform: base_link <- board
        - Returns PoseStamped or None if TF unavailable
        """
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

    def _debug_log_pnp_vs_depth(
        self,
        info,
        depth_image: np.ndarray,
        intrinsics: rs.intrinsics,
        stamp: Time,
    ) -> None:
        """
        For debugging: compare PnP marker pose vs depth reading at marker center.
        """
        try:
            # PnP: transform smoothed pose to base_frame
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

            self.node.get_logger().debug(
                f"[marker {self.board_ref_marker_id}] "
                f"{self.base_frame} depth: "
                f"x={depth_base.pose.position.x:.3f}, "
                f"y={depth_base.pose.position.y:.3f}, "
                f"z={depth_base.pose.position.z:.3f}"
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(
                f"Failed PnP vs depth debug for marker {self.board_ref_marker_id}: {exc}"
            )
