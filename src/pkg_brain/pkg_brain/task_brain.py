from __future__ import annotations

import rclpy
from rclpy.node import Node

from klotski_interfaces.msg import BoardState

from .context import BrainContext
from .managers import (ActionExecutor, PipelineOrchestrator, ServiceManager,
                       UIManager)


class TaskBrain(Node):
    """
    Brain node with modular components.

    Uses manager classes to handle different responsibilities:
    - UIManager: UI communication and mode management
    - ServiceManager: Sense and plan service operations
    - ActionExecutor: 5-phase manipulation sequence
    - PipelineOrchestrator: Handler pipeline coordination
    """

    def __init__(self):
        super().__init__("task_brain")

        self.declare_parameter("relocalise_between_moves", True)

        # Initialize context
        self.ctx = BrainContext()

        # Initialize managers
        self.ui_manager = UIManager(self)
        self.service_manager = ServiceManager(self)
        self.action_executor = ActionExecutor(self)
        self.pipeline_orchestrator = PipelineOrchestrator(self)

        # Set up board state subscription
        self.create_subscription(BoardState, "/board_state", self.on_board_state, 10)

        self.ui_manager.ui("TaskBrain up. Modes: auto | step | pause | reset")

    def on_board_state(self, state: BoardState) -> None:
        """Handle board state updates from sensing."""
        self.ctx.sensed = state
        self.ui_manager.ui(f"BoardState received: {len(state.board.pieces)} pieces")
        # sensing updated -> invalidate plan to trigger replanning
        self.ctx.plan_received = False
        self.tick("board_state")

    def tick(self, source: str) -> None:
        """Delegate to pipeline orchestrator."""
        self.pipeline_orchestrator.tick(source)

    def ui(self, msg: str) -> None:
        """Send UI message."""
        self.ui_manager.ui(msg)

    def debug(self, msg: str) -> None:
        """Log debug message."""
        self.ui_manager.debug(msg)

    def warn(self, msg: str) -> None:
        """Log warning message."""
        self.ui_manager.warn(msg)

    def start_sense(self) -> bool:
        """Start sensing operation."""
        return self.service_manager.start_sense()

    def start_plan(self) -> bool:
        """Start planning operation."""
        return self.service_manager.start_plan()

    def start_execute_next_move(self) -> bool:
        """Start executing next move."""
        return self.action_executor.start_execute_next_move()


def main() -> None:
    rclpy.init()
    node = TaskBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
