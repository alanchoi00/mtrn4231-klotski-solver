from __future__ import annotations

import copy
from typing import List

from rclpy.node import Node
from rclpy.task import Future

from klotski_interfaces.msg import Board, Cell, Move, MoveList
from klotski_interfaces.srv import CaptureBoard, SolveBoard

from ..context import Phase, TickSource


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
            res = fut.result()
            if res is None or not isinstance(res, CaptureBoard.Response):
                raise RuntimeError("No response received from /sense/capture_board")
        except Exception as e:
            brain.ctx.last_error = f"Sense failed: {e}"
            self.node.get_logger().warn(brain.ctx.last_error)
            self._ui("[sense] Failed (see logs)")
            return

        if res.ok:
            last_sensed_board = brain.ctx.sensed.board if brain.ctx.sensed else None
            last_move = brain.ctx.get_last_executed_move()
            brain.ctx.expected_board = self._derive_expected_board(last_sensed_board, last_move)
            brain.ctx.sensed = res.state
            self._ui(f"[sense] Captured board: {len(res.state.board.pieces)} pieces")
            # New sensing invalidates plan to trigger replanning
            next_phase = Phase.next_phase(brain.ctx.current_phase.value)
            if next_phase is not None:
                brain.ctx.current_phase = next_phase
                brain.get_logger().debug(f"[sense] Advanced to next phase {brain.ctx.current_phase} after sensing")
            else:
                brain.get_logger().warn(f"[sense] No next phase from current_phase={brain.ctx.current_phase}")
                raise RuntimeError("Invalid execution phase transition")
            brain.tick(TickSource.SENSE_DONE)
        else:
            self._ui(f"[sense] Capture failed: {res.note}")

    def _derive_expected_board(
        self,
        last_board: Board | None,
        last_move: Move | None,
    ) -> Board | None:
        """
        Derive the expected board state after applying the last move.
        """

        if last_board is None or last_move is None:
            return None

        if not last_board.pieces:
            return copy.deepcopy(last_board)
        expected = copy.deepcopy(last_board)
        move_piece = last_move.piece

        def _sorted_cells(cells):
            return sorted((c.col, c.row) for c in cells)

        target_cells = _sorted_cells(move_piece.cells)

        # 1) Find the piece in the board that corresponds to move_piece
        moving_piece = None
        for piece in expected.pieces:
            if piece.type != move_piece.type:
                continue
            if len(piece.cells) != len(move_piece.cells):
                continue
            if _sorted_cells(piece.cells) == target_cells:
                moving_piece = piece
                break

        if moving_piece is None:
            return None

        # 2) Find the anchor cell BEFORE the move.
        anchor_before = max(
            move_piece.cells,
            key=lambda c: (c.row, -c.col),
        )

        # 3) Compute translation in board coordinates to get to move.to_cell
        dc = last_move.to_cell.col - anchor_before.col
        dr = last_move.to_cell.row - anchor_before.row

        new_cells: List[Cell] = []
        for c in moving_piece.cells:
            new_cell = Cell()
            new_cell.col = c.col + dc
            new_cell.row = c.row + dr
            new_cells.append(new_cell)

        moving_piece.cells = new_cells
        return expected


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
            res = fut.result()
            if res is None or not isinstance(res, SolveBoard.Response):
                raise RuntimeError("No response received from /plan/solve")
        except Exception as e:
            brain.ctx.last_error = f"Plan failed: {e}"
            brain.debug(brain.ctx.last_error)
            self._ui("[plan] Failed (see logs)")
            # Leave ctx.plan as-is; stay paused
            return

        move_list: MoveList = res.plan
        brain.ctx.plan = list(move_list.moves)
        brain.ctx.plan_index = 0
        brain.debug(f"[plan] Reseted plan_index {brain.ctx.plan_index} after receiving plan {len(brain.ctx.plan)} moves")

        if len(brain.ctx.plan) == 0 and res.solved:
            self._ui(f"[plan] Solved: no moves needed (already at goal)")
            brain.ctx.set_goal_reached(True)
            brain.tick(TickSource.GOAL_REACHED)
            return

        if len(brain.ctx.plan) != 0 and not res.solved:
            self._ui(f"[plan] Warning: received plan with moves but solver indicates unsolved")
            raise RuntimeError("Inconsistent plan result from solver")

        self._ui(f"[plan] Received plan: {len(brain.ctx.plan)} move(s), solved={res.solved}")
        brain.ctx.current_phase = Phase.APPROACH
        brain.ctx.set_goal_reached(False)
        brain.tick(TickSource.PLAN_DONE)

    def _ui(self, msg: str) -> None:
        """Send UI message via brain's UI manager."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if isinstance(brain, TaskBrain):
            brain.ui_manager.ui(msg)
