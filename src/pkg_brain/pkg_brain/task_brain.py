from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from klotski_interfaces.msg import BoardState

from .context import BrainContext, ExecutionPhase
from .managers import ActionExecutor, PipelineOrchestrator, ServiceManager, UIManager
from .ui_modes import UIMode


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

        # Initialize context
        self.ctx = BrainContext()

        # Initialize managers
        self.ui_manager = UIManager(self)
        self.service_manager = ServiceManager(self)
        self.action_executor = ActionExecutor(self)
        self.pipeline_orchestrator = PipelineOrchestrator(self)

        # Set up board state subscription
        self.create_subscription(BoardState, "/board_state", self.on_board_state, 10)

        # Set up safety stop subscription
        self._safety_stop_active = False
        self._mode_before_safety_stop: int | None = None
        self._phase_before_safety_stop: ExecutionPhase | None = None
        self.create_subscription(Bool, "/safety/stop", self.on_safety_stop, 10)

        self.ui_manager.ui("TaskBrain up. Modes: auto | step | pause | reset")

    def on_board_state(self, state: BoardState) -> None:
        """Handle board state updates from sensing."""
        self.ctx.sensed = state
        self.ui_manager.ui(f"BoardState received: {len(state.board.pieces)} pieces")
        # sensing updated -> invalidate plan to trigger replanning
        self.ctx.plan_received = False
        self.tick("board_state")

    def on_safety_stop(self, msg: Bool) -> None:
        """Handle safety stop signals from hand detection."""
        stop_active = msg.data

        if stop_active and not self._safety_stop_active:
            # Safety stop triggered - save current mode/phase and pause
            self._safety_stop_active = True
            self._phase_before_safety_stop = self.ctx.current_phase
            if self.ctx.mode != UIMode.PAUSE:
                self._mode_before_safety_stop = self.ctx.mode
                self.ctx.mode = UIMode.PAUSE
                self.ui_manager.ui("⚠️ SAFETY STOP: Hand detected - pausing operations")
        elif not stop_active and self._safety_stop_active:
            # Safety stop cleared
            self._safety_stop_active = False

            # Determine which phase to resume at
            prev_phase = self._phase_before_safety_stop
            if prev_phase is None:
                # Was None before, stay at None (no phase change)
                pass
            elif prev_phase == ExecutionPhase.RETREAT:
                # Already at retreat, no need to change
                pass
            else:
                # Was mid-operation, go to retreat phase for safety
                self.ctx.current_phase = ExecutionPhase.RETREAT
                self.ui_manager.ui("🔄 Resuming at RETREAT phase for safety")

            # Restore previous mode
            if self._mode_before_safety_stop is not None:
                self.ctx.mode = self._mode_before_safety_stop
                mode_name = UIMode.to_string(self._mode_before_safety_stop)
                self.ui_manager.ui(f"✅ SAFETY CLEAR: Mode restored to {mode_name}")
                self._mode_before_safety_stop = None

            self._phase_before_safety_stop = None

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
