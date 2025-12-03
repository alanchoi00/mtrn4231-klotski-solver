from __future__ import annotations

from typing import List, Optional

import numpy as np
import pyrealsense2 as rs


def rotation_matrix_to_quaternion(r_mat: np.ndarray) -> list[float]:
    """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
    trace = float(np.trace(r_mat))
    if trace > 0.0:
        s = float(np.sqrt(trace + 1.0) * 2.0)
        w = 0.25 * s
        x = (r_mat[2, 1] - r_mat[1, 2]) / s
        y = (r_mat[0, 2] - r_mat[2, 0]) / s
        z = (r_mat[1, 0] - r_mat[0, 1]) / s
    elif r_mat[0, 0] > r_mat[1, 1] and r_mat[0, 0] > r_mat[2, 2]:
        s = float(np.sqrt(1.0 + r_mat[0, 0] - r_mat[1, 1] - r_mat[2, 2]) * 2.0)
        w = (r_mat[2, 1] - r_mat[1, 2]) / s
        x = 0.25 * s
        y = (r_mat[0, 1] + r_mat[1, 0]) / s
        z = (r_mat[0, 2] + r_mat[2, 0]) / s
    elif r_mat[1, 1] > r_mat[2, 2]:
        s = float(np.sqrt(1.0 + r_mat[1, 1] - r_mat[0, 0] - r_mat[2, 2]) * 2.0)
        w = (r_mat[0, 2] - r_mat[2, 0]) / s
        x = (r_mat[0, 1] + r_mat[1, 0]) / s
        y = 0.25 * s
        z = (r_mat[1, 2] + r_mat[2, 1]) / s
    else:
        s = float(np.sqrt(1.0 + r_mat[2, 2] - r_mat[0, 0] - r_mat[1, 1]) * 2.0)
        w = (r_mat[1, 0] - r_mat[0, 1]) / s
        x = (r_mat[0, 2] + r_mat[2, 0]) / s
        y = (r_mat[1, 2] + r_mat[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def average_quaternions(quats: List[List[float]]) -> List[float]:
    """Average a list of quaternions [x,y,z,w] using Markley method."""
    if not quats:
        return [0.0, 0.0, 0.0, 1.0]

    M = np.zeros((4, 4), dtype=float)
    for q in quats:
        arr = np.array(q, dtype=float).reshape(4, 1)
        M += arr @ arr.T

    eigenvalues, eigenvectors = np.linalg.eigh(M)
    avg_q = eigenvectors[:, int(np.argmax(eigenvalues))]
    avg_q = avg_q / np.linalg.norm(avg_q)
    return avg_q.tolist()  # [x, y, z, w]


def depth_pixel_to_point(
    depth_image: np.ndarray,
    intrinsics: rs.intrinsics,
    u: int,
    v: int,
) -> Optional[List[float]]:
    """
    Project a depth pixel (u, v) into 3D camera frame coordinates [X, Y, Z] (m).
    Returns None if depth is invalid or indices are out of bounds.
    """
    if depth_image is None or intrinsics is None:
        return None

    height, width = depth_image.shape[:2]
    if u < 0 or v < 0 or u >= width or v >= height:
        return None

    depth_raw = float(depth_image[int(v), int(u)])
    if depth_raw <= 0.0:
        return None

    depth_m = depth_raw * 0.001  # RealSense mm -> m
    X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], depth_m)
    return [float(X), float(Y), float(Z)]


def compute_marker_pose_from_depth(
    corners: np.ndarray,
    depth_image: np.ndarray,
    intrinsics: rs.intrinsics,
    marker_length_m: float,
) -> Optional[tuple[np.ndarray, List[float]]]:
    """
    Compute marker pose (position and orientation) using depth camera.
    
    Uses the depth camera to get 3D positions of marker corners, then
    computes orientation from the corner positions.
    
    Args:
        corners: 4x2 array of marker corner pixel coordinates
        depth_image: Depth image from RealSense
        intrinsics: Camera intrinsics
        marker_length_m: Physical marker side length in meters
        
    Returns:
        Tuple of (position [3], quaternion [x,y,z,w]) or None if failed
    """
    # Get 3D positions for all 4 corners using depth
    corner_points_3d = []
    for i in range(4):
        u, v = int(corners[i, 0]), int(corners[i, 1])
        pt = depth_pixel_to_point(depth_image, intrinsics, u, v)
        if pt is None:
            # Try sampling nearby pixels for robustness
            pt = _sample_depth_nearby(depth_image, intrinsics, u, v, radius=3)
        if pt is None:
            return None
        corner_points_3d.append(pt)
    
    corner_points_3d = np.array(corner_points_3d)  # shape (4, 3)
    
    # Marker center is average of 4 corners
    center_3d = np.mean(corner_points_3d, axis=0)
    
    # Compute orientation from corner positions
    # ArUco corners are ordered: TL, TR, BR, BL (in marker frame)
    # Corner 0 = top-left, Corner 1 = top-right
    # Corner 2 = bottom-right, Corner 3 = bottom-left
    
    # X-axis: from left to right (corner 0->1 or corner 3->2)
    vec_top = corner_points_3d[1] - corner_points_3d[0]  # TL -> TR
    vec_bottom = corner_points_3d[2] - corner_points_3d[3]  # BL -> BR
    x_axis = (vec_top + vec_bottom) / 2.0
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    # Y-axis: from bottom to top (corner 3->0 or corner 2->1)
    vec_left = corner_points_3d[0] - corner_points_3d[3]  # BL -> TL
    vec_right = corner_points_3d[1] - corner_points_3d[2]  # BR -> TR
    y_axis = (vec_left + vec_right) / 2.0
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # Z-axis: normal to marker plane (pointing towards camera)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    
    # Re-orthogonalize Y to ensure orthonormal basis
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # Build rotation matrix (columns are axis vectors)
    R = np.column_stack([x_axis, y_axis, z_axis])
    
    # Convert to quaternion
    quat = rotation_matrix_to_quaternion(R)
    
    return center_3d, quat


def _sample_depth_nearby(
    depth_image: np.ndarray,
    intrinsics: rs.intrinsics,
    u: int,
    v: int,
    radius: int = 3,
) -> Optional[List[float]]:
    """
    Sample depth from nearby pixels if the exact pixel has invalid depth.
    Uses a small window around (u, v) and takes the median valid depth.
    """
    height, width = depth_image.shape[:2]
    
    valid_depths = []
    for du in range(-radius, radius + 1):
        for dv in range(-radius, radius + 1):
            nu, nv = u + du, v + dv
            if 0 <= nu < width and 0 <= nv < height:
                depth_raw = float(depth_image[nv, nu])
                if depth_raw > 0:
                    valid_depths.append(depth_raw)
    
    if not valid_depths:
        return None
    
    # Use median depth for robustness
    median_depth_raw = float(np.median(valid_depths))
    depth_m = median_depth_raw * 0.001
    
    X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], depth_m)
    return [float(X), float(Y), float(Z)]
