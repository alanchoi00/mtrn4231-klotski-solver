from __future__ import annotations

from typing import Optional

from klotski_interfaces.action._grip_piece import (
    GripPiece_FeedbackMessage, GripPiece_GetResult_Response)
from klotski_interfaces.action._move_piece import (
    MovePiece_FeedbackMessage, MovePiece_GetResult_Response)
from klotski_interfaces.msg._cell import Cell
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from klotski_interfaces.action import GripPiece, MovePiece
from klotski_interfaces.msg import Move

from ..context import Phase, TickSource
from ..ui_modes import UIMode


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
        if brain.ctx.current_phase == Phase.APPROACH:
            return self._start_approach_phase(move)
        elif brain.ctx.current_phase == Phase.GRIP_CLOSE:
            return self._start_grip_close_phase()
        elif brain.ctx.current_phase == Phase.PICK_PLACE:
            return self._start_pick_place_phase(move)
        elif brain.ctx.current_phase == Phase.GRIP_OPEN:
            return self._start_grip_open_phase()
        elif brain.ctx.current_phase == Phase.RETREAT:
            return self._start_retreat_phase(move)
        else:
            self.node.get_logger().warn(f"[exec] Unknown execution phase: {Phase.get_name(brain.ctx.current_phase)}")
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
        # 'move.piece.cells' may be a set (not indexable); get a representative cell via iteration.
        first_cell: Optional[Cell] = None
        try:
            first_cell = next(iter(move.piece.cells))
        except Exception:
            first_cell = None
        if first_cell is not None and isinstance(first_cell, Cell):
            cell_pos = f"({first_cell.col},{first_cell.row})"
        else:
            cell_pos = "(?,?)"
        self._ui(f"[exec] Phase 1/5: Approaching piece type={move.piece.type} at {cell_pos}")
        send_fut = self.move_client.send_goal_async(goal, feedback_callback=self._on_move_feedback)
        send_fut.add_done_callback(self._on_move_goal_response)
        return True

    def _start_grip_open_phase(self) -> bool:
        """Phase 2: Open gripper."""
        if not self.grip_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /gripper_manipulation/grip_piece action server not available")
            return False

        goal = GripPiece.Goal()
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

    def _start_grip_close_phase(self) -> bool:
        """Phase 4: Close gripper."""
        if not self.grip_client.wait_for_server(timeout_sec=0.2):
            self._ui("[exec] /gripper_manipulation/grip_piece action server not available")
            return False

        goal = GripPiece.Goal()
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
        if goal_handle is None or not isinstance(goal_handle, ClientGoalHandle) or not goal_handle.accepted:
            self._ui(f"[exec] MovePiece phase {brain.ctx.current_phase} goal rejected or no goal handle received or invalid type")
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
        if goal_handle is None or not isinstance(goal_handle, ClientGoalHandle) or not goal_handle.accepted:
            self._ui(f"[exec] GripPiece phase {brain.ctx.current_phase} goal rejected or no goal handle received or invalid type")
            self._set_busy(False)
            brain.ctx.mode = UIMode.PAUSE
            return
        self.node.get_logger().debug(f"[exec] GripPiece phase {brain.ctx.current_phase} accepted")
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._on_grip_result)

    def _on_move_feedback(self, fb: MovePiece_FeedbackMessage) -> None:
        """Handle move action feedback."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        phase_name = Phase.get_name(brain.ctx.current_phase)
        self._ui(f"[exec] {phase_name} progress: {fb.feedback.progress:.0%}")

    def _on_grip_feedback(self, fb: GripPiece_FeedbackMessage) -> None:
        """Handle grip action feedback."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        phase_name = Phase.get_name(brain.ctx.current_phase)
        self._ui(f"[exec] {phase_name} progress: {fb.feedback.progress:.0%}")

    def _on_move_result(self, res_fut: Future) -> None:
        """Handle move action result."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        try:
            move_action_result = res_fut.result()
            if move_action_result is None or not isinstance(move_action_result, MovePiece_GetResult_Response):
                raise RuntimeError(f"No result received from MovePiece action. Received type: {type(move_action_result)}")
            ok = move_action_result.result.success
        except Exception as e:
            ok = False
            self.node.get_logger().warn(f"[exec] MovePiece exception: {e}")

        self._set_busy(False)
        if ok:
            phase_name = Phase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] {phase_name} phase OK")
            self._advance_to_next_phase()
        else:
            phase_name = Phase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] MovePiece {phase_name} FAILED -> pause")
            brain.ctx.mode = UIMode.PAUSE
            brain.tick(TickSource.EXEC_FAILED)

    def _on_grip_result(self, res_fut: Future) -> None:
        """Handle grip action result."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        try:
            grip_action_result = res_fut.result()
            if grip_action_result is None or not isinstance(grip_action_result, GripPiece_GetResult_Response):
                raise RuntimeError(f"No result received from GripPiece action. Received type: {type(grip_action_result)}")
            ok = grip_action_result.result.success
        except Exception as e:
            ok = False
            self.node.get_logger().warn(f"[exec] GripPiece exception: {e}")

        self._set_busy(False)
        if ok:
            phase_name = Phase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] {phase_name} OK")
            self._advance_to_next_phase()
        else:
            phase_name = Phase.get_name(brain.ctx.current_phase)
            self._ui(f"[exec] GripPiece {phase_name} FAILED -> pause")
            brain.ctx.mode = UIMode.PAUSE
            brain.tick(TickSource.EXEC_FAILED)

    def _advance_to_next_phase(self) -> None:
        """Advance to the next phase or complete the move."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        next_phase = Phase.next_phase(brain.ctx.current_phase)
        self.node.get_logger().debug(f"[exec] Advancing from phase {Phase.get_name(brain.ctx.current_phase)} to {Phase.get_name(next_phase) if next_phase is not None else 'complete move'}")
        if next_phase is not None:
            # Continue to next phase
            brain.ctx.current_phase = next_phase
            if brain.ctx.mode == UIMode.AUTO:
                brain.tick(TickSource.EXEC_NEXT_PHASE)
            elif brain.ctx.mode == UIMode.STEP:
                brain.ctx.mode = UIMode.PAUSE
                brain.tick(TickSource.EXEC_NEXT_PHASE)
        else:
            # All 5 phases complete
            self._ui(f"[exec] All phases complete for move {brain.ctx.plan_index + 1}/{len(brain.ctx.plan)}")
            brain.ctx.plan_index += 1
            brain.ctx.current_phase = Phase.get_start_phase()  # Reset for revalidation
            self.node.get_logger().debug(f"[exec] Reset current_phase to {brain.ctx.current_phase} and advanced plan_index to {brain.ctx.plan_index}")

            # Continue in AUTO; pause in STEP
            if brain.ctx.mode == UIMode.AUTO:
                # Continue next move after configurable delay (brain.delay_secs seconds)
                self._ui(f"[exec] Scheduling next phase {Phase.get_name(brain.ctx.current_phase)} in AUTO mode after {brain.delay_secs} seconds")
                def _callback_exec_done():
                    if brain.exec_timer is not None:
                        brain.exec_timer.cancel()
                        brain.exec_timer = None

                    # Don’t auto-advance if user has paused or process is busy
                    if brain.ctx.mode == UIMode.AUTO and not brain.ctx.busy:
                        brain.tick(TickSource.EXEC_DONE)
                brain.callback_after_delay(brain.delay_secs, _callback_exec_done)
            elif brain.ctx.mode == UIMode.STEP:
                brain.ctx.mode = UIMode.PAUSE
                brain.tick(TickSource.EXEC_DONE)

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
