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
