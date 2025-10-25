from __future__ import annotations

from rclpy.node import Node
from rclpy.task import Future

from klotski_interfaces.msg import MoveList
from klotski_interfaces.srv import CaptureBoard, SolveBoard

from ..context import ExecutionPhase


class ServiceManager:
    """Handles sense and plan service operations."""

    def __init__(self, node: Node):
        self.node = node

        # Service clients
        self.solve_cli = node.create_client(SolveBoard, "/plan/solve")
        self.sense_cli = node.create_client(CaptureBoard, "/sense/capture_board")

    def start_sense(self) -> bool:
        """Start sensing operation."""
        if not self.sense_cli.service_is_ready():
            self.node.get_logger().debug("[sense] /sense/capture_board service not ready")
            return False

        req = CaptureBoard.Request()
        fut = self.sense_cli.call_async(req)
        fut.add_done_callback(self._on_sense_result)
        self._ui("[sense] Requesting board capture from /sense/capture_board")
        return True

    def _on_sense_result(self, fut: Future) -> None:
        """Handle sense service result."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        try:
            res: CaptureBoard.Response = fut.result()
        except Exception as e:
            brain.ctx.last_error = f"Sense failed: {e}"
            self.node.get_logger().warn(brain.ctx.last_error)
            self._ui("[sense] Failed (see logs)")
            return

        if res.ok:
            brain.ctx.sensed = res.state
            self._ui(f"[sense] Captured board: {len(res.state.board.pieces)} pieces")
            # New sensing invalidates plan to trigger replanning
            brain.ctx.plan_received = False
            brain.tick("sense_done")
        else:
            self._ui(f"[sense] Capture failed: {res.note}")

    def start_plan(self) -> bool:
        """Start planning operation."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return False

        if not self.solve_cli.service_is_ready():
            return False

        req = SolveBoard.Request()
        req.state = brain.ctx.sensed
        req.goal = brain.ctx.goal

        fut = self.solve_cli.call_async(req)
        fut.add_done_callback(self._on_plan_result)
        self._ui("[plan] Requested plan from /plan/solve")
        return True

    def _on_plan_result(self, fut: Future) -> None:
        """Handle plan service result."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        try:
            res: SolveBoard.Response = fut.result()
        except Exception as e:
            brain.ctx.last_error = f"Plan failed: {e}"
            self.node.get_logger().warn(brain.ctx.last_error)
            self._ui("[plan] Failed (see logs)")
            # Leave ctx.plan as-is; stay paused
            return

        move_list: MoveList = res.plan
        brain.ctx.plan = list(move_list.moves)
        brain.ctx.plan_index = 0
        brain.ctx.current_phase = ExecutionPhase.APPROACH  # Reset to first phase
        brain.ctx.plan_received = True

        if len(brain.ctx.plan) == 0:
            if res.solved:
                self._ui(f"[plan] Goal already achieved! (0 moves needed)")
            else:
                self._ui(f"[plan] No solution found (0 moves, not solved)")
        else:
            self._ui(f"[plan] Received plan: {len(brain.ctx.plan)} move(s), solved={res.solved}")

        brain.tick("plan_done")

    def _ui(self, msg: str) -> None:
        """Send UI message via brain's UI manager."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if isinstance(brain, TaskBrain):
            brain.ui_manager.ui(msg)
