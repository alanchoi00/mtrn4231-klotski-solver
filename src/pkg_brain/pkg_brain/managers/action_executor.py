from __future__ import annotations

from typing import TYPE_CHECKING

from rclpy.action import ActionClient
from rclpy.task import Future

from klotski_interfaces.action import GripPiece, MovePiece
from klotski_interfaces.msg import Move

from ..context import ExecutionPhase
from ..ui_modes import UIMode

if TYPE_CHECKING:
    from rclpy.node import Node


class ActionExecutor:
    """Manages the 5-phase manipulation sequence execution."""

    def __init__(self, node: Node):
        self.node = node

        # Action clients
        self.move_client: ActionClient = ActionClient(node, MovePiece, "/arm_manipulation/move_piece")
        self.grip_client: ActionClient = ActionClient(node, GripPiece, "/gripper_manipulation/grip_piece")

    def start_execute_next_move(self) -> bool:
        """Start executing the next move in the current phase."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return False

        if brain.ctx.plan_index >= len(brain.ctx.plan):
            return False

        move: Move = brain.ctx.plan[brain.ctx.plan_index]

        # Execute the current phase of the 5-phase manipulation sequence
        if brain.ctx.current_phase == ExecutionPhase.APPROACH:
            return self._start_approach_phase(move)
        elif brain.ctx.current_phase == ExecutionPhase.GRIP_OPEN:
            return self._start_grip_open_phase(move)
        elif brain.ctx.current_phase == ExecutionPhase.PICK_PLACE:
            return self._start_pick_place_phase(move)
        elif brain.ctx.current_phase == ExecutionPhase.GRIP_CLOSE:
            return self._start_grip_close_phase(move)
        elif brain.ctx.current_phase == ExecutionPhase.RETREAT:
            return self._start_retreat_phase(move)
        else:
            self.node.get_logger().warn(f"[exec] Unknown execution phase: {ExecutionPhase.get_name(brain.ctx.current_phase)}")
            return False

    def _start_approach_phase(self, move: Move) -> bool:
        """Phase 1: Approach the piece."""
        if not self.move_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /arm_manipulation/move_piece action server not available")
            return False

        goal = MovePiece.Goal()
        goal.move = move
        goal.phase = MovePiece.Goal.PHASE_APPROACH

        self._set_busy(True)
        self._ui(f"[exec] Phase 1/5: Approaching piece type={move.piece.type} at ({move.piece.cells[0].col},{move.piece.cells[0].row})")
        send_fut = self.move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        send_fut.add_done_callback(self._on_move_goal_response)
        return True

    def _start_grip_open_phase(self, move: Move) -> bool:
        """Phase 2: Open gripper."""
        if not self.grip_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /gripper_manipulation/grip_piece action server not available")
            return False

        goal = GripPiece.Goal()
        goal.move = move
        goal.grip_action = GripPiece.Goal.GRIP_OPEN

        self._set_busy(True)
        self._ui(f"[exec] Phase 2/5: Opening gripper")
        send_fut = self.grip_client.send_goal_async(goal, feedback_callback=self._on_grip_feedback)
        send_fut.add_done_callback(self._on_grip_goal_response)
        return True

    def _start_pick_place_phase(self, move: Move) -> bool:
        """Phase 3: Pick and place the piece."""
        if not self.move_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /arm_manipulation/move_piece action server not available")
            return False

        goal = MovePiece.Goal()
        goal.phase = MovePiece.Goal.PHASE_PICK_PLACE

        self._set_busy(True)
        self._ui(f"[exec] Phase 3/5: Pick and place to ({move.to_cell.col},{move.to_cell.row})")
        send_fut = self.move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        send_fut.add_done_callback(self._on_move_goal_response)
        return True

    def _start_grip_close_phase(self, move: Move) -> bool:
        """Phase 4: Close gripper."""
        if not self.grip_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /gripper_manipulation/grip_piece action server not available")
            return False

        goal = GripPiece.Goal()
        goal.move = move
        goal.grip_action = GripPiece.Goal.GRIP_CLOSE

        self._set_busy(True)
        self._ui(f"[exec] Phase 4/5: Closing gripper")
        send_fut = self.grip_client.send_goal_async(goal, feedback_callback=self._on_grip_feedback)
        send_fut.add_done_callback(self._on_grip_goal_response)
        return True

    def _start_retreat_phase(self, move: Move) -> bool:
        """Phase 5: Retreat to home position."""
        if not self.move_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /arm_manipulation/move_piece action server not available")
            return False

        goal = MovePiece.Goal()
        goal.move = move
        goal.phase = MovePiece.Goal.PHASE_RETREAT

        self._set_busy(True)
        self._ui(f"[exec] Phase 5/5: Retreating to home position")
        send_fut = self.move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        send_fut.add_done_callback(self._on_move_goal_response)
        return True

    def _on_move_goal_response(self, goal_fut: Future) -> None:
        """Handle move action goal response."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        goal_handle = goal_fut.result()
        if not goal_handle.accepted:
            self._ui(f"[exec] MovePiece phase {brain.ctx.current_phase} goal rejected")
            self._set_busy(False)
            brain.ctx.mode = UIMode.PAUSE
            return
        self.node.get_logger().debug(f"[exec] MovePiece phase {brain.ctx.current_phase} accepted")
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._on_move_result)

    def _on_grip_goal_response(self, goal_fut: Future) -> None:
        """Handle grip action goal response."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        goal_handle = goal_fut.result()
        if not goal_handle.accepted:
            self._ui(f"[exec] GripPiece phase {brain.ctx.current_phase} goal rejected")
            self._set_busy(False)
            brain.ctx.mode = UIMode.PAUSE
            return
        self.node.get_logger().debug(f"[exec] GripPiece phase {brain.ctx.current_phase} accepted")
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._on_grip_result)

    def _on_move_feedback(self, fb: MovePiece.Feedback) -> None:
        """Handle move action feedback."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        phase_name = ExecutionPhase.get_name(brain.ctx.current_phase)
        self._ui(f"[exec] {phase_name} progress: {fb.progress:.2f}")

    def _on_grip_feedback(self, fb: GripPiece.Feedback) -> None:
        """Handle grip action feedback."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        phase_name = ExecutionPhase.get_name(brain.ctx.current_phase)
        self._ui(f"[exec] {phase_name} progress: {fb.progress:.2f}")

    def _on_move_result(self, res_fut: Future) -> None:
        """Handle move action result."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        try:
            result: MovePiece.Result = res_fut.result().result
            ok = result.success
        except Exception as e:
            ok = False
            self.node.get_logger().warn(f"[exec] MovePiece exception: {e}")

        self._set_busy(False)
        if ok:
            phase_name = ExecutionPhase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] {phase_name} phase OK")
            self._advance_to_next_phase()
        else:
            phase_name = ExecutionPhase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] MovePiece {phase_name} FAILED -> pause")
            brain.ctx.mode = UIMode.PAUSE
            brain.tick("exec_failed")

    def _on_grip_result(self, res_fut: Future) -> None:
        """Handle grip action result."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        try:
            result: GripPiece.Result = res_fut.result().result
            ok = result.success
        except Exception as e:
            ok = False
            self.node.get_logger().warn(f"[exec] GripPiece exception: {e}")

        self._set_busy(False)
        if ok:
            phase_name = ExecutionPhase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] {phase_name} OK")
            self._advance_to_next_phase()
        else:
            phase_name = ExecutionPhase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] GripPiece {phase_name} FAILED -> pause")
            brain.ctx.mode = UIMode.PAUSE
            brain.tick("exec_failed")

    def _advance_to_next_phase(self) -> None:
        """Advance to the next phase or complete the move."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        next_phase = ExecutionPhase.next_phase(brain.ctx.current_phase)
        if next_phase is not None:
            # Continue to next phase
            brain.ctx.current_phase = next_phase
            if brain.ctx.mode == UIMode.AUTO:
                brain.tick("exec_next_phase")
            elif brain.ctx.mode == UIMode.STEP:
                brain.ctx.mode = UIMode.PAUSE
                brain.tick("exec_phase_done")
        else:
            # All 5 phases complete
            self._ui(f"[exec] All phases complete for move {brain.ctx.plan_index + 1}/{len(brain.ctx.plan)}")
            brain.ctx.plan_index += 1
            brain.ctx.current_phase = ExecutionPhase.APPROACH  # Reset for next move

            # Continue in AUTO; pause in STEP
            if brain.ctx.mode == UIMode.AUTO:
                brain.tick("exec_next_auto")
            elif brain.ctx.mode == UIMode.STEP:
                brain.ctx.mode = UIMode.PAUSE
                brain.tick("exec_step_done")

    def _ui(self, msg: str) -> None:
        """Send UI message via brain's UI manager."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if isinstance(brain, TaskBrain):
            brain.ui_manager.ui(msg)

    def _set_busy(self, busy: bool) -> None:
        """Set busy state in brain context."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if isinstance(brain, TaskBrain):
            brain.ctx.busy = busy
