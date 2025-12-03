from __future__ import annotations

from klotski_interfaces.srv._capture_board import CaptureBoard_Response
from klotski_interfaces.msg import BoardState
from ..types import BoardConfig, PieceCounts
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from ..managers import (
    CameraManager,
    HSVConfigManager,
    TransformationManager,
    BoardStateManager,
)

from ..services import (
    detect_aruco_markers,
    get_missing_target_ids,
    warp_image_to_board,
    build_masks,
    annotate_cell_colors,
    render_annotated_grid_overlay_image,
    grid_to_board,
    validate_board_configuration,
)


class CaptureBoardHandler:
    """
    Runs the FULL perception pipeline:
    1) Get camera frame
    2) Detect ArUco markers
    3) Warp image to board
    4) Colour classification per cell
    5) Convert to Board message
    6) Generate TF transforms
    7) Validate board
    8) Publish results
    """

    def __init__(
        self,
        *,
        camera_manager: CameraManager,
        tf_manager: TransformationManager,
        board_state_manager: BoardStateManager,
        hsv_config_manager: HSVConfigManager,
        clock: Clock,
        logger: RcutilsLogger,
        board_config: BoardConfig,
        piece_counts: PieceCounts,
        base_frame: str,
        min_cell_colour_area: int,
    ):
        self.camera = camera_manager
        self.tf = tf_manager
        self.board_state_mgr = board_state_manager
        self.hsv_cfg = hsv_config_manager
        self.clock = clock
        self.logger = logger

        # Board parameters
        self.board_frame = board_config.frame_id
        self.base_frame = base_frame
        self.tl = board_config.tl_marker
        self.tr = board_config.tr_marker
        self.bl = board_config.bl_marker
        self.br = board_config.br_marker
        self.W = board_config.width
        self.H = board_config.height
        self.min_area = min_cell_colour_area
        self.piece_counts = piece_counts

    def handle(self) -> CaptureBoard_Response:
        """Top-level pipeline wrapper."""
        response = CaptureBoard_Response(ok=False, note="Unknown error")

        try:
            self.logger.info("Starting board capture pipeline...")

            # Ensure base TF tree is available before proceeding
            if not self.tf.wait_for_base_tf(timeout_sec=5.0):
                raise RuntimeError(
                    f"Base TF tree not available ({self.base_frame} <- camera). "
                    "Ensure the static transform publisher is running."
                )

            image = self._step_acquire_image()
            self.logger.info("Acquired image from camera")
            aruco_infos = self._step_detect_markers(image)
            self.logger.info(f"Detected {len(aruco_infos)} ArUco markers")
            warped = self._step_warp_board(image, aruco_infos)
            self.logger.info("Warped image to board perspective")
            _, grid = self._step_classify_cells(warped)
            self.logger.info("Classified cell colours")
            board_msg = self._step_build_board(grid)
            self.logger.info("Built Board message from grid")
            board_pose = self._step_update_transforms(aruco_infos)
            self.logger.info("Updated TF transforms for markers and board")
            self._step_validate_board(grid, board_msg)
            self.logger.info("Validated board configuration")
            state_msg = self._step_publish_output(board_msg, board_pose)
            self.logger.info("Board capture pipeline completed successfully")

            response.state = state_msg
            response.ok = True
            response.note = "Board captured successfully"

        except Exception as e:
            response.ok = False
            response.note = str(e)
            self.logger.error(f"Board capture pipeline failed: {e}")

        return response

    def _step_acquire_image(self):
        image = self.camera.get_color_image()
        if image is None:
            raise RuntimeError("No color image available")
        return image.copy()

    def _step_detect_markers(self, image):
        infos = detect_aruco_markers(image)
        missing = get_missing_target_ids(infos)
        if any(missing):
            raise RuntimeError(f"Missing ArUco markers: {missing}")
        return infos

    def _step_warp_board(self, image, aruco_infos):
        try:
            warped = warp_image_to_board(
                image,
                aruco_infos,
                board_width_cells=self.W,
                board_height_cells=self.H,
                board_tl_marker_id=self.tl,
                board_tr_marker_id=self.tr,
                board_bl_marker_id=self.bl,
                board_br_marker_id=self.br,
            )
        except Exception as e:
            raise RuntimeError(f"Warping failed: {e}")

        self.camera.publish_warped_image(warped)
        return warped

    def _step_classify_cells(self, warped_bgr):
        hsv_cfg = self.hsv_cfg.get_hsv_config()

        masks = build_masks(warped_bgr, hsv_cfg)
        grid = annotate_cell_colors(
            warped_bgr,
            masks,
            min_cell_colour_area=self.min_area,
            board_width_cells=self.W,
            board_height_cells=self.H,
        )

        overlay = render_annotated_grid_overlay_image(
            warped_bgr,
            grid_bottom=grid,
            board_width_cells=self.W,
            board_height_cells=self.H,
        )
        self.camera.publish_overlay_image(overlay)

        return masks, grid

    def _step_build_board(self, grid):
        return grid_to_board(grid, board_width_cells=self.W, board_height_cells=self.H)

    def _step_update_transforms(self, aruco_infos):
        # Check prerequisites before attempting transform computation
        intrinsics = self.camera.get_intrinsics()
        if intrinsics is None:
            raise RuntimeError(
                "Camera intrinsics not available. "
                "Ensure camera is publishing to camera info topic."
            )

        depth_image = self.camera.get_depth_image()
        if depth_image is None:
            raise RuntimeError(
                "Depth image not available. "
                "Depth camera is required for accurate ArUco frame computation. "
                "Ensure depth camera is publishing to the aligned depth topic."
            )

        marker_poses = self.tf.update_markers(
            aruco_info=aruco_infos,
            depth_image=depth_image,
            intrinsics=intrinsics,
            stamp=self.clock.now(),
        )

        # Validate that marker poses were computed
        if not marker_poses:
            raise RuntimeError(
                "No marker poses computed. "
                "PnP failed for all detected markers. "
                "Check camera calibration and marker detection."
            )

        # Check if the reference marker (used for board frame) was detected
        if self.tf.board_ref_marker_id not in marker_poses:
            raise RuntimeError(
                f"Reference marker {self.tf.board_ref_marker_id} pose not computed. "
                "Board frame transform cannot be published."
            )

        # Compute board pose directly from the marker pose
        # This avoids the race condition of waiting for our own published TF
        ref_marker_pose = marker_poses[self.tf.board_ref_marker_id]
        board_pose = self.tf.compute_board_pose_in_base(ref_marker_pose)

        if board_pose is None:
            raise RuntimeError(
                f"Failed to compute board pose in base frame. "
                f"Transform from camera to '{self.base_frame}' may not be available. "
                "Ensure the static transform publisher is running."
            )

        return board_pose

    def _step_validate_board(self, grid, board_msg):
        validate_board_configuration(grid, board_msg, self.W, self.H, self.piece_counts)

    def _step_publish_output(self, board_msg, board_pose):
        now = self.clock.now().to_msg()
        state_msg = BoardState(stamp=now, board=board_msg, board_pose=board_pose)
        self.board_state_mgr.publish_board_state(state_msg)
        return state_msg
