from .types import BoardConfig, PieceCounts, CameraConfig, FramesConfig
from rclpy.node import Node
import rclpy

from klotski_interfaces.srv import CaptureBoard
from klotski_utils import declare_param

from .handlers import CaptureBoardHandler
from .managers import (
    CameraManager,
    HSVConfigManager,
    BoardStateManager,
    TransformationManager,
)


class Sense(Node):
    """Node for sensing the board state."""

    def __init__(self):
        super().__init__("sense")

        # Board geometry
        board_width = declare_param[int](
            self,
            "board_width_cells",
            "Width of the board in cells",
        )
        board_height = declare_param[int](
            self,
            "board_height_cells",
            "Height of the board in cells",
        )
        offset_x = declare_param[float](
            self,
            "board_offset_x_m",
            "Board X offset",
        )
        offset_y = declare_param[float](
            self,
            "board_offset_y_m",
            "Board Y offset",
        )
        board_frame = declare_param[str](
            self,
            "board_frame_id",
            "Board frame ID",
        )
        tl = declare_param[int](
            self,
            "board_tl_marker_id",
            "TL marker ID",
        )
        tr = declare_param[int](
            self,
            "board_tr_marker_id",
            "TR marker ID",
        )
        bl = declare_param[int](
            self,
            "board_bl_marker_id",
            "BL marker ID",
        )
        br = declare_param[int](
            self,
            "board_br_marker_id",
            "BR marker ID",
        )
        marker_len = declare_param[float](
            self,
            "marker_length_m",
            "Marker size (m)",
        )

        self.board_config = BoardConfig(
            width=board_width,
            height=board_height,
            offset_x=offset_x,
            offset_y=offset_y,
            frame_id=board_frame,
            tl_marker=tl,
            tr_marker=tr,
            bl_marker=bl,
            br_marker=br,
            marker_length=marker_len,
        )

        # Camera
        self.camera_config = CameraConfig(
            frame_id=declare_param[str](self, "camera_frame_id", "Camera frame ID"),
            link_frame_id=declare_param[str](
                self,
                "camera_link_frame_id",
                "Camera link frame ID",
            ),
        )

        # Piece counts
        self.piece_counts = PieceCounts(
            red=declare_param[int](
                self,
                "board_piece_counts.red",
                "Red piece count",
            ),
            green=declare_param[int](
                self,
                "board_piece_counts.green",
                "Green piece count",
            ),
            blue=declare_param[int](
                self,
                "board_piece_counts.blue",
                "Blue piece count",
            ),
            yellow=declare_param[int](
                self,
                "board_piece_counts.yellow",
                "Yellow piece count",
            ),
            empty=declare_param[int](
                self,
                "board_piece_counts.empty",
                "Empty piece count",
            ),
        )

        # Other configs
        self.min_cell_colour_area = declare_param[int](
            self,
            "min_cell_colour_area",
            "Min cell color area",
        )

        self.frames_config = FramesConfig(
            world=declare_param[str](self, "world_frame_id", "World frame"),
            board=self.board_config.frame_id,
        )

        hsv_file = declare_param[str](self, "hsv_config_file", "HSV config file path")

        self.hsv_config_manager = HSVConfigManager(self, hsv_file)
        self.camera = CameraManager(
            self,
            self.frames_config.world,
            hsv_config_getter=self.hsv_config_manager.get_hsv_config,
        )
        self.board_state_manager = BoardStateManager(self)

        self.transformation_manager = TransformationManager(
            self,
            base_frame=self.frames_config.world,
            camera_frame=self.camera_config.frame_id,
            board_frame=self.board_config.frame_id,
            board_marker_length_m=self.board_config.marker_length,
            board_ref_marker_id=self.board_config.bl_marker,
            board_offset_x_m=self.board_config.offset_x,
            board_offset_y_m=self.board_config.offset_y,
        )

        self.capture_board_handler = CaptureBoardHandler(
            camera_manager=self.camera,
            tf_manager=self.transformation_manager,
            hsv_config_manager=self.hsv_config_manager,
            board_state_manager=self.board_state_manager,
            board_config=self.board_config,
            piece_counts=self.piece_counts,
            base_frame=self.frames_config.world,
            min_cell_colour_area=self.min_cell_colour_area,
            logger=self.get_logger(),
            clock=self.get_clock(),
        )

        self.create_service(
            CaptureBoard, "/sense/capture_board", self.capture_board_callback
        )
        self.get_logger().info("sense ready: /sense/capture_board")

    def capture_board_callback(self, request, response):
        return self.capture_board_handler.handle()


def main(args=None):
    rclpy.init(args=args)
    node = Sense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        node.get_logger().info(
            f"KeyboardInterrupt received, shutting down node...: {e}"
        )
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
