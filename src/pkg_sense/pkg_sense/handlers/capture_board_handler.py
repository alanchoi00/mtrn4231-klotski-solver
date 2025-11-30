from klotski_interfaces.srv._capture_board import CaptureBoard_Response
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from klotski_interfaces.msg import BoardState

from ..managers import BoardStateManager, CameraManager, TransformationManager
from ..services import (HSVConfig, annotate_cell_colors, build_masks,
                        detect_aruco_markers, get_missing_target_ids,
                        grid_to_board, render_annotated_grid_overlay_image,
                        validate_board_configuration, warp_image_to_board)


class CaptureBoardHandler:
    def __init__(
        self,
        camera_manager: CameraManager,
        tf_manager: TransformationManager,
        board_state_manager: BoardStateManager,
        clock: Clock,
        logger: RcutilsLogger,
        hsv_config: HSVConfig,
        board_width_cells: int,
        board_height_cells: int,
        min_cell_colour_area: int,
    ):
        self.camera_manager = camera_manager
        self.tf_manager = tf_manager
        self.board_state_manager = board_state_manager
        self.hsv_config = hsv_config
        self.clock = clock
        self.logger = logger
        self.board_width_cells = board_width_cells
        self.board_height_cells = board_height_cells
        self.min_cell_colour_area = min_cell_colour_area

    def handle(self) -> CaptureBoard_Response:
        response = CaptureBoard_Response(
            ok=False, note="Uninitialized"
        )  # default in case of early return

        image = self.camera_manager.get_color_image()
        if image is None:
            response.ok = False
            response.note = "No color image yet"
            return response

        image = image.copy()

        aruco_infos = detect_aruco_markers(image)
        missing_ids = get_missing_target_ids(aruco_infos)
        if any(missing_ids):
            response.ok = False
            response.note = f"Missing markers with IDs: {missing_ids}"
            return response

        now = self.clock.now().to_msg()

        try:
            warped_image = warp_image_to_board(
                image,
                aruco_infos,
                board_width_cells=self.board_width_cells,
                board_height_cells=self.board_height_cells,
            )
        except Exception as e:
            response.ok = False
            response.note = f"Error warping image: {e}"
            return response

        self.camera_manager.publish_warped_image(warped_image)
        self.logger.debug("Warped image published")

        colour_masks = build_masks(warped_image, self.hsv_config)
        annotated_grid = annotate_cell_colors(
            warped_image,
            colour_masks,
            min_cell_colour_area=self.min_cell_colour_area,
            board_width_cells=self.board_width_cells,
            board_height_cells=self.board_height_cells,
        )

        overlay_image = render_annotated_grid_overlay_image(
            warped_image,
            grid_bottom=annotated_grid,
            board_width_cells=self.board_width_cells,
            board_height_cells=self.board_height_cells,
        )

        self.camera_manager.publish_overlay_image(overlay_image)
        self.logger.debug("Overlay image published")

        try:
            board_msg = grid_to_board(
                annotated_grid,
                board_width_cells=self.board_width_cells,
                board_height_cells=self.board_height_cells,
            )
        except Exception as e:
            response.ok = False
            response.note = f"Error converting grid to board message: {e}"
            return response
        # board_pose_msg = self.tf_manager.get_board_pose()

        state_msg = BoardState(
            board=board_msg,
            stamp=now,
            # board_pose=board_pose_msg
        )

        try:
            validate_board_configuration(
                annotated_grid, board_msg, self.board_width_cells, self.board_height_cells
            )
        except Exception as e:
            response.ok = False
            response.note = f"Invalid board configuration: {e}"
            return response

        response.state = state_msg
        response.ok = True
        response.note = "Board captured successfully"

        self.board_state_manager.publish_board_state(state_msg)

        return response
