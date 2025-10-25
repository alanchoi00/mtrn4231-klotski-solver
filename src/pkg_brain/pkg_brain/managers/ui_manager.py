from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import String

from klotski_interfaces.msg import Board, UICommand

from ..ui_modes import UIMode


class UIManager:
    """Handles UI communication and mode management."""

    def __init__(self, node: Node):
        self.node = node
        self.ui_pub = node.create_publisher(String, "/ui/events", 10)

        # Set up subscriptions
        node.create_subscription(UICommand, "/ui/cmd", self.on_ui_cmd, 10)
        node.create_subscription(Board, "/ui/goal", self.on_goal, 10)

    def ui(self, msg: str) -> None:
        """Send message to UI and log it."""
        self.ui_pub.publish(String(data=msg))
        self.node.get_logger().info(msg)

    def debug(self, msg: str) -> None:
        """Log debug message."""
        self.node.get_logger().debug(msg)

    def warn(self, msg: str) -> None:
        """Log warning message."""
        self.node.get_logger().warn(msg)

    def on_ui_cmd(self, cmd: UICommand) -> None:
        """Handle UI command messages."""
        from ..task_brain import TaskBrain
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        mode = (cmd.mode or UIMode.IDLE)
        if UIMode.is_valid(mode):
            if mode == UIMode.RESET:
                self.ui("UI: reset -> pause + clear memory")
                brain.ctx.reset()
            elif mode == UIMode.PAUSE:
                self.ui("UI: pause")
                brain.ctx.mode = UIMode.PAUSE
                # If an action were active, you could cancel here.
            else:
                mode_name = UIMode.to_string(mode)
                self.ui(f"UI: mode={mode_name}")
                brain.ctx.mode = mode
        else:
            self.ui(f"UI: unknown mode '{cmd.mode}' (use auto|step|pause|reset)")

        brain.tick("ui_cmd")

    def on_goal(self, msg: Board) -> None:
        """Handle goal pattern updates."""
        from ..task_brain import TaskBrain
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        brain.ctx.goal = msg
        self.ui(f"Goal pattern received: {self._board_to_pattern(msg)}")
        brain.ctx.reset()
        brain.ctx.plan_received = False
        brain.tick("goal")

    def _board_to_pattern(self, board: Board) -> str:
        """Convert a Board message to a pattern string for display."""
        W = board.spec.cols
        H = board.spec.rows

        # Create top-origin grid of digits (0 = empty)
        grid = [[0 for _ in range(W)] for _ in range(H)]

        # Fill grid with piece types at their cell positions
        for piece in board.pieces:
            for cell in piece.cells:
                col, row = cell.col, cell.row

                # Convert bottom-left origin (ROS) to top-origin (pattern display)
                top_row = H - 1 - row
                left_col = col

                if 0 <= top_row < H and 0 <= left_col < W:
                    grid[top_row][left_col] = piece.type

        # Flatten to string (row-major, top row first)
        result = ""
        for r in range(H):
            for c in range(W):
                result += str(grid[r][c])

        return result
