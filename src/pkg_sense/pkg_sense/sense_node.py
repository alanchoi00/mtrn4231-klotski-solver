import rclpy
from klotski_interfaces.srv._capture_board import (CaptureBoard,
                                                   CaptureBoard_Request,
                                                   CaptureBoard_Response)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from .handlers import CaptureBoardHandler
from .managers import (BoardStateManager, CameraManager, HSVConfigManager,
                       TransformationManager)


class Sense(Node):
    """
    Sense node for the Klotski solver.
    """

    def __init__(self):
        super().__init__("sense")

        # Declare parameters with descriptors
        self.declare_parameter(
            "board_width_cells",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Width of the board in cells",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_height_cells",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Height of the board in cells",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "min_cell_colour_area",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Minimum area for cell colour detection",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "camera_frame_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING.value,
                description="Frame ID for the camera",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "camera_link_frame_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING.value,
                description="Frame ID for the camera link",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_tl_marker_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Top-left corner marker ID for the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_tr_marker_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Top-right corner marker ID for the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_bl_marker_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Bottom-left corner marker ID for the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_br_marker_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Bottom-right corner marker ID for the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "marker_length_m",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.DOUBLE.value,
                description="Length of the marker in meters",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_offset_x_m",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.DOUBLE.value,
                description="X offset of the board in meters",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_offset_y_m",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.DOUBLE.value,
                description="Y offset of the board in meters",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_frame_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING.value,
                description="Frame ID for the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_piece_counts.red",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Counts of each red piece on the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_piece_counts.green",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Counts of each green piece on the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_piece_counts.blue",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Counts of each blue piece on the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_piece_counts.yellow",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Counts of each yellow piece on the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "board_piece_counts.empty",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.INTEGER.value,
                description="Counts of each empty piece on the board",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "world_frame_id",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING.value,
                description="Frame ID for the world",
                read_only=True,
            ),
        )
        self.declare_parameter(
            "hsv_config_file",
            descriptor=ParameterDescriptor(
                type=rclpy.Parameter.Type.STRING.value,
                description="Path to HSV configuration file",
                read_only=True,
            ),
        )

        self.board_width_cells = (
            self.get_parameter("board_width_cells").get_parameter_value().integer_value
        )
        self.board_height_cells = (
            self.get_parameter("board_height_cells").get_parameter_value().integer_value
        )
        self.min_cell_colour_area = (
            self.get_parameter("min_cell_colour_area")
            .get_parameter_value()
            .integer_value
        )
        self.camera_frame_id = (
            self.get_parameter("camera_frame_id").get_parameter_value().string_value
        )
        self.camera_link_frame_id = (
            self.get_parameter("camera_link_frame_id")
            .get_parameter_value()
            .string_value
        )
        self.board_tl_marker_id = (
            self.get_parameter("board_tl_marker_id")
            .get_parameter_value()
            .integer_value
        )
        self.board_tr_marker_id = (
            self.get_parameter("board_tr_marker_id")
            .get_parameter_value()
            .integer_value
        )
        self.board_bl_marker_id = (
            self.get_parameter("board_bl_marker_id")
            .get_parameter_value()
            .integer_value
        )
        self.board_br_marker_id = (
            self.get_parameter("board_br_marker_id")
            .get_parameter_value()
            .integer_value
        )
        self.marker_length_m = (
            self.get_parameter("marker_length_m").get_parameter_value().double_value
        )
        self.board_offset_x_m = (
            self.get_parameter("board_offset_x_m").get_parameter_value().double_value
        )
        self.board_offset_y_m = (
            self.get_parameter("board_offset_y_m").get_parameter_value().double_value
        )
        self.board_frame_id = (
            self.get_parameter("board_frame_id").get_parameter_value().string_value
        )
        self.board_piece_counts_red = (
            self.get_parameter("board_piece_counts.red")
            .get_parameter_value()
            .integer_value
        )
        self.board_piece_counts_green = (
            self.get_parameter("board_piece_counts.green")
            .get_parameter_value()
            .integer_value
        )
        self.board_piece_counts_blue = (
            self.get_parameter("board_piece_counts.blue")
            .get_parameter_value()
            .integer_value
        )
        self.board_piece_counts_yellow = (
            self.get_parameter("board_piece_counts.yellow")
            .get_parameter_value()
            .integer_value
        )
        self.board_piece_counts_empty = (
            self.get_parameter("board_piece_counts.empty")
            .get_parameter_value()
            .integer_value
        )
        self.world_frame_id = (
            self.get_parameter("world_frame_id").get_parameter_value().string_value
        )
        hsv_config_filename = (
            self.get_parameter("hsv_config_file").get_parameter_value().string_value
        )

        self.camera = CameraManager(self)
        self.transformation = TransformationManager(self)
        self.board_state_manager = BoardStateManager(self)
        self.hsv_config_manager = HSVConfigManager(self, hsv_config_filename)

        # Service
        self.capture_board_service = self.create_service(
            CaptureBoard, "/sense/capture_board", self.capture_board_callback
        )
        # Handlers
        self.capture_board_handler = CaptureBoardHandler(
            camera_manager=self.camera,
            tf_manager=self.transformation,
            hsv_config_manager=self.hsv_config_manager,
            board_state_manager=self.board_state_manager,
            clock=self.get_clock(),
            logger=self.get_logger(),
            board_tl_marker_id=self.board_tl_marker_id,
            board_tr_marker_id=self.board_tr_marker_id,
            board_bl_marker_id=self.board_bl_marker_id,
            board_br_marker_id=self.board_br_marker_id,
            board_width_cells=self.board_width_cells,
            board_height_cells=self.board_height_cells,
            min_cell_colour_area=self.min_cell_colour_area,
        )

        self.get_logger().info("sense ready: /sense/capture_board")

    def capture_board_callback(
        self,
        request: CaptureBoard_Request,
        response: CaptureBoard_Response
    ) -> CaptureBoard_Response:
        self.get_logger().info("capture_board_callback called")
        return self.capture_board_handler.handle()


def main():
    rclpy.init()
    sense = Sense()
    rclpy.spin(sense)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
