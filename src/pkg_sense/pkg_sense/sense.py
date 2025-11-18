from klotski_interfaces.srv._capture_board import CaptureBoard_Response
import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from klotski_interfaces.msg import Board, BoardSpec, BoardState, Cell, Piece
from klotski_interfaces.srv import CaptureBoard

# ------------------------------

W, H = 4, 5                     # grid: cols × rows
ARUCO_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL
MIN_CELL_COLOUR_AREA = 300      # area in pixels to count a cell as filled

# Physical board geometry (mm)
BOARD_W_MM = 200.0              # 4 cells × 50 mm
BOARD_H_MM = 250.0              # 5 cells × 50 mm
OFFSET_MM  = 80.0               # marker centre offset from each edge


# HSV thresholds (tune to your lighting)
HSV_RANGES = {
    "red1":   ((0,   100,  80), (10,  255, 255)),
    "red2":   ((170, 100,  80), (180, 255, 255)),
    "yellow": ((19,   72, 0), (78,  136, 255)),
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
# ARUCO DETECTION
def detect_aruco_info(image_bgr):
    """Detect ArUco 0,1,2,3 with a multi-pass detector on a single frame."""
    aruco = cv2.aruco
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)

    # Base params
    base_params = aruco.DetectorParameters()
    base_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    # Different parameter tweaks to try on the SAME gray image
    trials = [
        dict(adaptiveThreshWinSizeMin=3,  adaptiveThreshWinSizeMax=23, adaptiveThreshConstant=7.0),
        dict(adaptiveThreshWinSizeMin=5,  adaptiveThreshWinSizeMax=35, adaptiveThreshConstant=7.0),
        dict(adaptiveThreshWinSizeMin=7,  adaptiveThreshWinSizeMax=45, adaptiveThreshConstant=5.0),
        dict(adaptiveThreshWinSizeMin=3,  adaptiveThreshWinSizeMax=23, adaptiveThreshConstant=3.0),
    ]

    best_info = {}
    last_ids = []

    for trial_idx, trial in enumerate(trials, start=1):
        params = aruco.DetectorParameters()
        # copy base
        params.cornerRefinementMethod = base_params.cornerRefinementMethod
        # apply overrides
        for k, v in trial.items():
            setattr(params, k, v)

        try:
            detector = aruco.ArucoDetector(dictionary, params)
            corners_list, ids, _ = detector.detectMarkers(gray)
        except Exception:
            corners_list, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)

        info = {}
        if ids is not None:
            ids_flat = ids.flatten()
            last_ids = sorted(int(i) for i in ids_flat)
            for corners, i in zip(corners_list, ids_flat):
                i = int(i)
                if i not in (0, 1, 2, 3):
                    continue
                pts = corners.reshape(4, 2).astype(np.float32)
                center = pts.mean(axis=0)
                info[i] = {"center": center, "corners": pts}
        else:
            last_ids = []

        # If we got all four tags, we are done
        if all(k in info for k in (0, 1, 2, 3)):
            best_info = info
            break

        # Otherwise remember the best we saw so far (if you want)
        if len(info) > len(best_info):
            best_info = info

    # debug overlay (optional, on the original image + last detection)
    try:
        dbg = image_bgr.copy()
        if last_ids:
            # reconstruct corners_list & ids from best_info for drawing
            corners_draw = [best_info[k]["corners"][None, :, :] for k in best_info]
            ids_draw = np.array([[k] for k in best_info], dtype=np.int32)
            try:
                aruco.drawDetectedMarkers(dbg, corners_draw, ids_draw)
            except Exception:
                aruco.drawDetectedMarkers(dbg, corners_draw)
        for i, item in best_info.items():
            c = item["center"].astype(int)
            cv2.putText(dbg, str(i), tuple(c),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imwrite("debug_aruco.png", dbg)
    except Exception:
        pass

    return best_info


# ------------------------------
# HOMOGRAPHY
def compute_homography_auto(info):
    """
    Compute homography using the *centres* of markers 0,1,2,3 and the known
    physical layout:
      - Board: BOARD_W_MM × BOARD_H_MM
      - Marker centres offset: OFFSET_MM from each edge
    Returns H_in2out (image px -> rectified px) and rectified size (w,h).
    """
    for k in (0, 1, 2, 3):
        if k not in info:
            raise RuntimeError(f"Missing ArUco id {k}. Found {sorted(info.keys())}")

    # Marker centres in input image pixels
    TLc = info[0]["center"].astype(np.float32)  # top-left marker centre
    TRc = info[1]["center"].astype(np.float32)  # top-right
    BLc = info[2]["center"].astype(np.float32)  # bottom-left
    BRc = info[3]["center"].astype(np.float32)  # bottom-right

    # --- 1) Homography from board-mm coords (marker centres) -> input pixels ---

    # Marker centre positions in board coordinates (mm),
    # with (0,0) at the *top-left board corner*.
    pts_mm = np.float32([
        [OFFSET_MM,                   OFFSET_MM                  ],  # TL marker
        [BOARD_W_MM - OFFSET_MM,     OFFSET_MM                  ],  # TR marker
        [OFFSET_MM,                   BOARD_H_MM - OFFSET_MM    ],  # BL marker
        [BOARD_W_MM - OFFSET_MM,     BOARD_H_MM - OFFSET_MM    ],  # BR marker
    ])

    pts_px = np.float32([
        TLc,
        TRc,
        BLc,
        BRc,
    ])

    H_mm2in, _ = cv2.findHomography(pts_mm, pts_px)
    if H_mm2in is None:
        raise RuntimeError("Homography mm->image failed")

    # --- 2) Choose output scale (px per mm) from centre distances ---

    # Horizontal px per mm from TL <-> TR
    d_px_x = np.linalg.norm(TRc - TLc)
    d_mm_x = BOARD_W_MM - 2.0 * OFFSET_MM
    px_per_mm_x = d_px_x / d_mm_x if d_mm_x != 0 else 1.0

    # Vertical px per mm from TL <-> BL
    d_px_y = np.linalg.norm(BLc - TLc)
    d_mm_y = BOARD_H_MM - 2.0 * OFFSET_MM
    px_per_mm_y = d_px_y / d_mm_y if d_mm_y != 0 else 1.0

    # Use the average so we don't distort aspect ratio too badly
    S = float((px_per_mm_x + px_per_mm_y) * 0.5)

    out_w = int(round(BOARD_W_MM * S))
    out_h = int(round(BOARD_H_MM * S))

    # --- 3) Homography from board-mm coords -> rectified pixel coords ---

    board_corners_mm = np.float32([
        [0.0,         0.0        ],          # top-left board corner
        [BOARD_W_MM,  0.0        ],          # top-right
        [BOARD_W_MM,  BOARD_H_MM ],          # bottom-right
        [0.0,         BOARD_H_MM ],          # bottom-left
    ])

    dst_pts = np.float32([
        [0.0,    0.0   ],         # TL in rectified image
        [out_w,  0.0   ],         # TR
        [out_w,  out_h ],         # BR
        [0.0,    out_h ],         # BL
    ])

    H_mm2out, _ = cv2.findHomography(board_corners_mm, dst_pts)
    if H_mm2out is None:
        raise RuntimeError("Homography mm->rectified failed")

    # --- 4) Combine to get input-px -> rectified-px homography ---

    H_in2mm = np.linalg.inv(H_mm2in)
    H_in2out = H_mm2out @ H_in2mm

    return H_in2out.astype(np.float32), (out_w, out_h)


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

def board_to_state(board: Board, rectified_size_px, frame_id: str) -> BoardState:
    st = BoardState()
    st.board = board
    st.board_pose_map.header.frame_id = frame_id
    try:
        st.rectified_width_px  = int(rectified_size_px[0])
        st.rectified_height_px = int(rectified_size_px[1])
    except Exception:
        pass
    return st


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
        self.intrinsics = None

        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.arm_image_callback, 10
        )
        self.point_cloud_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.arm_point_cloud_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.arm_image_depth_info_callback, 10
        )

        self.declare_parameter('frame_id', 'map')

        # Publishers
        self.ui_pub = self.create_publisher(String, '/ui/events', 10)
        self.state_pub = self.create_publisher(BoardState, '/board_state', 10)

        # Service (no timer)
        self.srv = self.create_service(CaptureBoard, '/sense/capture_board', self.handle_capture_board)

        # TF (not used yet)
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
            self.get_logger().error(f"Error in arm_image_callback: {str(e)}")

    def arm_point_cloud_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")

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

        try:
            Hmat, size = compute_homography_auto(info)
        except Exception as e:
            response.note = f"Homography failed: {e}"
            return response

        warped = warp_board(image, Hmat, size)

        grid, overlay, masks = classify_cells(warped)
        try:
            cv2.imwrite("cells_overlay.png", overlay)
            for k, m in masks.items():
                cv2.imwrite(f"mask_{k}.png", m)
            cv2.imwrite("rectified_board.png", warped)
        except Exception:
            pass

        board_msg = grid_to_board(grid, rectified_size_px=(size[0], size[1]))
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        state_msg = board_to_state(board_msg, (size[0], size[1]), frame_id)

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
