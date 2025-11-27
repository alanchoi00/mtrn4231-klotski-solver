from klotski_interfaces.srv._capture_board import CaptureBoard_Response
import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.duration import Duration                     # <<< NEW
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped   # <<< NEW

from klotski_interfaces.msg import Board, BoardSpec, BoardState, Cell, Piece
from klotski_interfaces.srv import CaptureBoard

# ------------------------------
# CONFIG
W, H = 4, 5                     # grid: cols × rows
ARUCO_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
MIN_CELL_COLOUR_AREA = 400      # area in pixels to count a cell as filled

# TF / ARUCO CONFIG (MATCH OTHER GROUPS)
CAMERA_FRAME = "camera_color_optical_frame"  # RealSense color optical frame
BOARD_MARKER_ID = 2                          # bottom-left marker ID
BOARD_MARKER_LENGTH_M = 0.07                 # marker side length (m) – adjust if needed
BOARD_OFFSET_X_M = 0.040                     # 40 mm to the right (marker frame X)
BOARD_OFFSET_Y_M = 0.040                     # 40 mm up (marker frame Y)
BOARD_FRAME_NAME = "klotski_board"           # board frame (bottom-left of board)

# HSV thresholds (tune to your lighting)
HSV_RANGES = {
    "red1":   ((0,   100,  80), (10,  255, 255)),
    "red2":   ((170, 100,  80), (180, 255, 255)),
    "yellow": ((27, 59, 118), (79,  190, 255)),
    "green":  ((44,   32,  76), (94,  255, 255)),
    "blue":   ((95,   198,  0), (130, 255, 156)),
    "grey":   ((0,     0,  50), (180,  70, 230)),   # low-saturation greys
}

PIECE_TYPE_2_2 = getattr(Piece, "TYPE_2_2", 1)  # 2 by 2
PIECE_TYPE_1_2 = getattr(Piece, "TYPE_1_2", 2)  # 1 by 2 (horizontal)
PIECE_TYPE_2_1 = getattr(Piece, "TYPE_2_1", 3)  # 2 by 1 (vertical)
PIECE_TYPE_1_1 = getattr(Piece, "TYPE_1_1", 4)  # 1 by 1

COLOR_NONE   = getattr(Piece, "COLOR_NONE",   0)
COLOR_RED    = getattr(Piece, "COLOR_RED",    1)
COLOR_BLUE   = getattr(Piece, "COLOR_BLUE",   2)
COLOR_GREEN  = getattr(Piece, "COLOR_GREEN",  3)
COLOR_YELLOW = getattr(Piece, "COLOR_YELLOW", 4)


# ------------------------------
# SMALL HELPER FOR TF
def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


# ------------------------------
# ARUCO DETECTION
def detect_aruco_info(image_bgr):
    """Detect ArUco 0,1,2,3 (DICT_ARUCO_ORIGINAL) and return centers + corners."""
    aruco = cv2.aruco
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    try:
        detector = aruco.ArucoDetector(dictionary, params)
        corners_list, ids, _ = detector.detectMarkers(gray)
    except Exception:
        corners_list, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)

    info = {}
    if ids is not None:
        ids = ids.flatten()
        for corners, i in zip(corners_list, ids):
            if int(i) not in (0,1,2,3):
                continue  # ignore stray tags
            pts = corners.reshape(4, 2).astype(np.float32)
            center = pts.mean(axis=0)
            info[int(i)] = {"center": center, "corners": pts}

    # Removed disk writes; if you want to visualise, use the debug image publishers below
    return info

# ------------------------------
# HOMOGRAPHY
def compute_homography_auto(info):
    """Compute H and rectified size directly from marker geometry (no px/mm)."""
    for k in (0,1,2,3):
        if k not in info:
            raise RuntimeError(f"Missing ArUco id {k}. Found {sorted(info.keys())}")

    # Map: TL=0, TR=1, BL=2, BR=3
    centers = np.array([info[k]["center"] for k in (0,1,2,3)], dtype=np.float32)
    board_center = centers.mean(axis=0)

    def inner_corner(id_):
        pts = info[id_]["corners"]
        d = np.linalg.norm(pts - board_center[None, :], axis=1)
        return pts[np.argmin(d)]

    # inner corners in image px
    TL = inner_corner(0)
    TR = inner_corner(1)
    BR = inner_corner(3)
    BL = inner_corner(2)

    width_px  = np.linalg.norm(TR - TL)
    height_px = np.linalg.norm(BL - TL)
    cell_px_w = width_px / W
    cell_px_h = height_px / H
    cell_px   = (cell_px_w + cell_px_h) / 2.0

    out_w = int(round(W * cell_px))
    out_h = int(round(H * cell_px))

    dst_pts = np.float32([
        [0,      0     ],   # TL
        [out_w,  0     ],   # TR
        [out_w,  out_h ],   # BR
        [0,      out_h ],   # BL
    ])
    src_pts = np.float32([TL, TR, BR, BL])
    Hmat, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
    if Hmat is None:
        raise RuntimeError("Homography failed")
    return Hmat, (out_w, out_h)

def warp_board(image_bgr, Hmat, size):
    out_w, out_h = size
    return cv2.warpPerspective(image_bgr, Hmat, (out_w, out_h))

# ------------------------------
# COLOUR DETECTION
def build_colour_masks(warped_bgr):
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    def mask(lo, hi): return cv2.inRange(hsv, np.array(lo,np.uint8), np.array(hi,np.uint8))
    red = mask(*HSV_RANGES["red1"]) | mask(*HSV_RANGES["red2"])
    masks = {
        "red": red,
        "yellow": mask(*HSV_RANGES["yellow"]),
        "green":  mask(*HSV_RANGES["green"]),
        "blue":   mask(*HSV_RANGES["blue"]),
        "grey":   mask(*HSV_RANGES["grey"]),
    }
    kernel = np.ones((5,5), np.uint8)
    for k in masks:
        m = masks[k]
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        masks[k] = m
    return masks

# ------------------------------
# GRID CLASSIFICATION
def classify_cells(warped_bgr):
    """Return (grid_bottom_origin, overlay, masks)."""
    Himg, Wimg = warped_bgr.shape[:2]
    cw, ch = Wimg / W, Himg / H
    masks = build_colour_masks(warped_bgr)
    overlay = warped_bgr.copy()

    grid_top = [["empty"]*W for _ in range(H)]
    for r_top in range(H):
        y0 = int(round(r_top * ch))
        y1 = int(round((r_top+1)*ch))
        for c in range(W):
            x0 = int(round(c * cw))
            x1 = int(round((c+1)*cw))
            cell = (slice(y0,y1), slice(x0,x1))
            areas = {n:int(np.count_nonzero(m[cell])) for n,m in masks.items()}
            best = max(areas, key=areas.get)

            # Classification with grey treated as empty
            if areas[best] < MIN_CELL_COLOUR_AREA:
                colour = "empty"
            elif best == "grey":
                colour = "empty"
            else:
                colour = best

            grid_top[r_top][c] = colour
            cv2.rectangle(overlay, (x0,y0), (x1,y1), (0,0,0), 1)
            cv2.putText(overlay, colour, (x0+3, y0+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,0,0), 1)

    # flip to bottom-left origin
    grid_bottom = [["empty"]*W for _ in range(H)]
    for r_top in range(H):
        r_bot = H - 1 - r_top
        for c in range(W):
            grid_bottom[r_bot][c] = grid_top[r_top][c]
    return grid_bottom, overlay, masks

# ------------------------------
# PIECES (ENFORCED SHAPES) → BOARD
def _add_cell(pm: Piece, r: int, c: int):
    cell = Cell()
    cell.row = int(r)
    cell.col = int(c)
    pm.cells.append(cell)

def grid_to_board(grid, rectified_size_px=None) -> Board:
    """
    Enforce shapes:
      - red   2x2 blocks
      - green 2x1 horizontal (split long runs)
      - blue  1x2 vertical   (split long runs)
      - yellow 1x1 (every yellow cell)
    grid is bottom-left origin (H x W) with {'red','green','blue','yellow','empty'}.
    """
    b = Board()
    b.spec = BoardSpec(cols=W, rows=H, cell_size_m=0.05, board_thickness_m=0.08)

    visited = [[False]*W for _ in range(H)]

    # RED 2x2
    for r in range(H-1):
        for c in range(W-1):
            if any(visited[r+i][c+j] for i in (0,1) for j in (0,1)):
                continue
            if (grid[r][c] == "red" and grid[r+1][c] == "red" and
                grid[r][c+1] == "red" and grid[r+1][c+1] == "red"):
                pm = Piece()
                pm.type = PIECE_TYPE_2_2
                pm.color = COLOR_RED
                for (rr, cc) in [(r,c),(r+1,c),(r,c+1),(r+1,c+1)]:
                    _add_cell(pm, rr, cc)
                    visited[rr][cc] = True
                if hasattr(pm, "min_row"): pm.min_row = int(r)
                if hasattr(pm, "min_col"): pm.min_col = int(c)
                if hasattr(pm, "width"):   pm.width   = 2
                if hasattr(pm, "height"):  pm.height  = 2
                b.pieces.append(pm)

    # GREEN 2x1 (horizontal)
    for r in range(H):
        c = 0
        while c < W:
            if not visited[r][c] and grid[r][c] == "green":
                run_start = c
                while c < W and (not visited[r][c]) and grid[r][c] == "green":
                    c += 1
                run_len = c - run_start
                pairs = run_len // 2
                for k in range(pairs):
                    c0 = run_start + 2*k
                    pm = Piece()
                    pm.type = PIECE_TYPE_1_2
                    pm.color = COLOR_GREEN
                    _add_cell(pm, r, c0)
                    _add_cell(pm, r, c0+1)
                    visited[r][c0] = visited[r][c0+1] = True
                    if hasattr(pm, "min_row"): pm.min_row = int(r)
                    if hasattr(pm, "min_col"): pm.min_col = int(c0)
                    if hasattr(pm, "width"):   pm.width   = 2
                    if hasattr(pm, "height"):  pm.height  = 1
                    b.pieces.append(pm)
                # leftover odd green cell is ignored (invalid)
            else:
                c += 1

    # BLUE 1x2 (vertical)
    for c in range(W):
        r = 0
        while r < H:
            if not visited[r][c] and grid[r][c] == "blue":
                run_start = r
                while r < H and (not visited[r][c]) and grid[r][c] == "blue":
                    r += 1
                run_len = r - run_start
                pairs = run_len // 2
                for k in range(pairs):
                    r0 = run_start + 2*k
                    pm = Piece()
                    pm.type = PIECE_TYPE_2_1
                    pm.color = COLOR_BLUE
                    _add_cell(pm, r0, c)
                    _add_cell(pm, r0+1, c)
                    visited[r0][c] = visited[r0+1][c] = True
                    if hasattr(pm, "min_row"): pm.min_row = int(r0)
                    if hasattr(pm, "min_col"): pm.min_col = int(c)
                    if hasattr(pm, "width"):   pm.width   = 1
                    if hasattr(pm, "height"):  pm.height  = 2
                    b.pieces.append(pm)
                # leftover odd blue cell is ignored (invalid)
            else:
                r += 1

    # YELLOW 1x1
    for r in range(H):
        for c in range(W):
            if not visited[r][c] and grid[r][c] == "yellow":
                pm = Piece()
                pm.type = PIECE_TYPE_1_1
                pm.color = COLOR_YELLOW
                _add_cell(pm, r, c)
                visited[r][c] = True
                if hasattr(pm, "min_row"): pm.min_row = int(r)
                if hasattr(pm, "min_col"): pm.min_col = int(c)
                if hasattr(pm, "width"):   pm.width   = 1
                if hasattr(pm, "height"):  pm.height  = 1
                b.pieces.append(pm)

    # Diagnostics: leftover colored cells that didn't fit shapes
    leftovers = []
    for r in range(H):
        for c in range(W):
            if not visited[r][c] and grid[r][c] in ("red","green","blue","yellow"):
                leftovers.append((r, c, grid[r][c]))
    if leftovers:
        print(f"[klotski] WARNING: leftover colored cells not forming valid pieces: {leftovers}")

    return b


def validate_counts(grid, board: Board, node_logger=None) -> bool:
    cnt_red   = sum(1 for p in board.pieces if p.color == COLOR_RED   and p.type == PIECE_TYPE_2_2)
    cnt_green = sum(1 for p in board.pieces if p.color == COLOR_GREEN and p.type == PIECE_TYPE_1_2)
    cnt_blue  = sum(1 for p in board.pieces if p.color == COLOR_BLUE  and p.type == PIECE_TYPE_2_1)
    cnt_yel   = sum(1 for p in board.pieces if p.color == COLOR_YELLOW and p.type == PIECE_TYPE_1_1)
    empty_cells = sum(1 for r in range(H) for c in range(W) if grid[r][c] == "empty")
    ok = (cnt_blue == 4 and cnt_red == 1 and cnt_green == 1 and cnt_yel == 4 and empty_cells == 2)
    if not ok and node_logger:
        node_logger.warning(
            f"Invalid counts -> blue:{cnt_blue} red:{cnt_red} green:{cnt_green} yellow:{cnt_yel} empty:{empty_cells} (want 4,1,1,4,2)"
        )
    return ok


# ------------------------------
# NODE
class Sense(Node):

    def __init__(self):
        super().__init__('sense')

        # Subscriptions
        self.cv_bridge = CvBridge()
        self.cv_image = None
        self.depth_image = None
        self.intrinsics = None  # rs.intrinsics

        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.arm_image_callback, 10
        )
        self.point_cloud_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.arm_point_cloud_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.arm_image_depth_info_callback, 10
        )

        self.declare_parameter('frame_id', 'map')   # frame where you want marker_pose (e.g. map/base_link)

        # Publishers
        self.ui_pub = self.create_publisher(String, '/ui/events', 10)
        self.state_pub = self.create_publisher(BoardState, '/board_state', 10)

        # Debug image publishers for rqt_image_view  <<< NEW
        self.debug_overlay_pub = self.create_publisher(Image, '/sense/cells_overlay', 1)
        self.debug_warped_pub  = self.create_publisher(Image, '/sense/rectified_board', 1)

        # Service (no timer)
        self.srv = self.create_service(CaptureBoard, '/sense/capture_board', self.handle_capture_board)

        # TF (now actually used)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("sense ready: /sense/capture_board")

    # --- subs
    def arm_image_depth_info_callback(self, cameraInfo):
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
            if cameraInfo.distortion_model == 'plumb_bob':
                intr.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                intr.model = rs.distortion.kannala_brandt4
            else:
                intr.model = rs.distortion.none
            intr.coeffs = [float(i) for i in cameraInfo.d]
            self.intrinsics = intr
        except Exception as e:
            self.get_logger().error(f"camera info error: {e}")

    def arm_image_callback(self, msg):
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error in arm_image_callback: {e}")

    def arm_point_cloud_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {e}")

    def handle_capture_board(self, request, response: CaptureBoard_Response):
        # defaults in case we bail early
        response.state = BoardState()
        response.ok = False
        response.note = "Uninitialized"

        if self.cv_image is None:
            response.note = "No color frame yet"
            return response

        image = self.cv_image.copy()

        info = detect_aruco_info(image)
        if len(info) < 4:
            response.note = f"Need markers 0,1,2,3; detected: {sorted(info.keys()) if info else []}"
            return response

        # Timestamp used for marker poses & state
        now_stamp = self.get_clock().now().to_msg()

        # --- NEW: compute 3D poses for all 4 markers in CAMERA_FRAME ---
        marker_data_cam = {}   # id -> dict(pose=PoseStamped, R=3x3, tvec=3x1, quat=[x,y,z,w])

        if self.intrinsics is None:
            self.get_logger().warn("Intrinsics not ready; cannot compute marker 3D poses.")
        else:
            try:
                cam = self.intrinsics
                cameraMatrix = np.array([
                    [cam.fx, 0,      cam.ppx],
                    [0,      cam.fy, cam.ppy],
                    [0,      0,      1.0]
                ], dtype=np.float32)
                distCoeffs = np.array(cam.coeffs, dtype=np.float32)

                L = BOARD_MARKER_LENGTH_M
                obj_points = np.array([
                    [-L/2,  L/2, 0],
                    [ L/2,  L/2, 0],
                    [ L/2, -L/2, 0],
                    [-L/2, -L/2, 0]
                ], dtype=np.float32)

                for mid in (0, 1, 2, 3):
                    if mid not in info:
                        self.get_logger().warn(f"Marker {mid} not in detected info, skipping.")
                        continue

                    img_points = info[mid]["corners"].astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(obj_points, img_points, cameraMatrix, distCoeffs)
                    if not success:
                        self.get_logger().warn(f"solvePnP failed for marker {mid}, skipping.")
                        continue

                    R, _ = cv2.Rodrigues(rvec)
                    qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

                    pose_cam = PoseStamped()
                    pose_cam.header.stamp = now_stamp
                    pose_cam.header.frame_id = CAMERA_FRAME
                    pose_cam.pose.position.x = float(tvec[0])
                    pose_cam.pose.position.y = float(tvec[1])
                    pose_cam.pose.position.z = float(tvec[2])
                    pose_cam.pose.orientation.x = qx
                    pose_cam.pose.orientation.y = qy
                    pose_cam.pose.orientation.z = qz
                    pose_cam.pose.orientation.w = qw

                    marker_data_cam[mid] = {
                        "pose": pose_cam,
                        "R": R,
                        "tvec": tvec,
                        "quat": [qx, qy, qz, qw],
                    }
            except Exception as e:
                self.get_logger().error(f"Error computing marker 3D poses: {e}")

        # --- TF for ArUco 2 and board frame (same as before, but now using marker_data_cam) ---
        if BOARD_MARKER_ID in marker_data_cam:
            try:
                md = marker_data_cam[BOARD_MARKER_ID]
                tvec = md["tvec"]
                qx, qy, qz, qw = md["quat"]

                # 1) camera_color_optical_frame -> aruco_2
                t_marker = TransformStamped()
                t_marker.header.stamp = now_stamp
                t_marker.header.frame_id = CAMERA_FRAME
                t_marker.child_frame_id = f"aruco_{BOARD_MARKER_ID}"

                t_marker.transform.translation.x = float(tvec[0])
                t_marker.transform.translation.y = float(tvec[1])
                t_marker.transform.translation.z = float(tvec[2])

                t_marker.transform.rotation.x = qx
                t_marker.transform.rotation.y = qy
                t_marker.transform.rotation.z = qz
                t_marker.transform.rotation.w = qw

                self.tf_broadcaster.sendTransform(t_marker)

                # 2) aruco_2 -> klotski_board (bottom-left of board)
                t_board = TransformStamped()
                t_board.header.stamp = now_stamp
                t_board.header.frame_id = t_marker.child_frame_id  # "aruco_2"
                t_board.child_frame_id = BOARD_FRAME_NAME

                t_board.transform.translation.x = BOARD_OFFSET_X_M
                t_board.transform.translation.y = BOARD_OFFSET_Y_M
                t_board.transform.translation.z = 0.0

                # No extra rotation: board frame aligned with marker frame
                t_board.transform.rotation.x = 0.0
                t_board.transform.rotation.y = 0.0
                t_board.transform.rotation.z = 0.0
                t_board.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(t_board)

                self.get_logger().info(
                    f"Published TF: {CAMERA_FRAME} -> aruco_{BOARD_MARKER_ID} "
                    f"and aruco_{BOARD_MARKER_ID} -> {BOARD_FRAME_NAME}"
                )
            except Exception as e:
                self.get_logger().error(f"Error computing TF for board: {e}")
        else:
            self.get_logger().warn("No 3D data for BOARD_MARKER_ID; TF for aruco_2/board not published.")

        # --- Homography & grid detection (unchanged logic) ---
        try:
            Hmat, size = compute_homography_auto(info)
        except Exception as e:
            response.note = f"Homography failed: {e}"
            return response

        warped = warp_board(image, Hmat, size)

        grid, overlay, masks = classify_cells(warped)

        # Publish debug images to rqt instead of writing to disk  <<< NEW
        try:
            frame_id_param = self.get_parameter('frame_id').get_parameter_value().string_value

            overlay_msg = self.cv_bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header.stamp = now_stamp
            overlay_msg.header.frame_id = frame_id_param
            self.debug_overlay_pub.publish(overlay_msg)

            warped_msg = self.cv_bridge.cv2_to_imgmsg(warped, encoding="bgr8")
            warped_msg.header.stamp = now_stamp
            warped_msg.header.frame_id = frame_id_param
            self.debug_warped_pub.publish(warped_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug images: {e}")

        # Build Board
        board_msg = grid_to_board(grid, rectified_size_px=(size[0], size[1]))

        # --- Fill marker_pose[] in requested frame (map/base_link)  <<< NEW ---
        marker_poses_out = []
        target_frame = self.get_parameter('frame_id').get_parameter_value().string_value

        for mid in (0, 1, 2, 3):
            if mid not in marker_data_cam:
                continue
            pose_cam = marker_data_cam[mid]["pose"]
            try:
                pose_out = self.tf_buffer.transform(
                    pose_cam,
                    target_frame,
                    timeout=Duration(seconds=0.2)
                )
                # If you want to tag which marker is which, you can encode it in the frame_id or ignore.
                marker_poses_out.append(pose_out)
            except Exception as e:
                self.get_logger().warn(f"Failed to transform marker {mid} pose to {target_frame}: {e}")

        # Build BoardState according to new message definition:
        # builtin_interfaces/Time stamp
        # geometry_msgs/PoseStamped[] marker_pose
        # klotski_interfaces/Board board
        state_msg = BoardState()
        state_msg.stamp = now_stamp
        state_msg.board = board_msg
        state_msg.marker_pose = marker_poses_out

        counts_ok = validate_counts(grid, board_msg, self.get_logger())

        # publish for consumers
        self.state_pub.publish(state_msg)

        response.state = state_msg
        response.ok = bool(counts_ok)
        response.note = "Captured" if counts_ok else "Captured, but piece counts invalid (want 4B,1R,1G,4Y,2 empty)"
        return response


def main():
    rclpy.init()
    sense = Sense()
    rclpy.spin(sense)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
