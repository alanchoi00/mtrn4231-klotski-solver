from typing import Tuple

import cv2
import numpy as np

from .aruco_detector import ArucoInfoList


def calculate_marker_homography(
    aruco_infos: ArucoInfoList,
    board_width_cells: int,
    board_height_cells: int
) -> Tuple[np.ndarray, Tuple[int, int]]:
    """
    Compute H and rectified size directly from marker geometry (no px/mm).
    """

    # Map: TL=0, TR=1, BL=2, BR=3
    sorted_aruco_infos = aruco_infos.sort_by_target_id()
    centers = np.array([info.center for info in sorted_aruco_infos], dtype=np.float32)
    board_center = centers.mean(axis=0)

    def inner_corner(id: int):
        info = aruco_infos.get_by_id(id)
        if info is None:
            raise RuntimeError(f"Missing ArUco id {id}")
        pts = info.corners
        d = np.linalg.norm(pts - board_center[None, :], axis=1)
        return pts[np.argmin(d)]

    # inner corners in image px
    #   TL(0) ------ TR(1)
    #    |            |
    #    |            |
    #   BL(3) ------ BR(2)
    #
    # Note: inner_corner(id) returns the inner corner of the marker with that id
    # by choosing the corner closest to the board center.
    board_top_left_px = inner_corner(0)
    board_top_right_px = inner_corner(1)
    board_bottom_left_px = inner_corner(2)
    board_bottom_right_px = inner_corner(3)

    width_px = np.linalg.norm(board_top_right_px - board_top_left_px)
    height_px = np.linalg.norm(board_bottom_right_px - board_top_right_px)
    cell_px_w = width_px / board_width_cells
    cell_px_h = height_px / board_height_cells
    cell_px = (cell_px_w + cell_px_h) / 2.0

    out_w = int(round(board_width_cells * cell_px))
    out_h = int(round(board_height_cells * cell_px))
    dst_pts = np.array(
        [
            [0, 0],  # TL
            [out_w, 0],  # TR
            [out_w, out_h],  # BR
            [0, out_h],  # BL
        ],
        dtype=np.float32,
    )
    src_pts = np.array(
        [board_top_left_px, board_top_right_px, board_bottom_right_px, board_bottom_left_px],
        dtype=np.float32,
    )
    h_mat, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
    if h_mat is None:
        raise RuntimeError("Homography failed")
    return h_mat, (out_w, out_h)


def warp_image_to_board(image: np.ndarray, aruco_infos: ArucoInfoList, board_width_cells: int, board_height_cells: int):
    h_mat, size = calculate_marker_homography(aruco_infos, board_width_cells, board_height_cells)
    return cv2.warpPerspective(image, h_mat, size)
