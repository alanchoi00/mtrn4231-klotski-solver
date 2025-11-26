from __future__ import annotations

from klotski_interfaces.msg import Board

from ..context import BrainContext, Phase
from .base import BaseHandler, BrainNodeLike, HandlerResult
from .status import HandlerStatus


class PlanHandler(BaseHandler):
    name = "plan"

    def do_handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        if not Phase.is_plan_phase(ctx.current_phase):
            return HandlerResult(HandlerStatus.NEXT, f"not planning phase (current_phase={ctx.current_phase})")

        if ctx.goal is None:
            node.debug("[plan] No goal yet -> waiting")
            return HandlerResult(HandlerStatus.DONE, "waiting goal")

        if ctx.sensed is None:
            return HandlerResult(HandlerStatus.NEXT, "sense first")

        def _request_plan(reason: str) -> HandlerResult:
            node.debug(f"[plan] {reason} -> requesting plan")
            if not node.start_plan():
                node.ui("[plan] planner service not available")
                return HandlerResult(HandlerStatus.DONE, "planner not available")
            return HandlerResult(HandlerStatus.PENDING, "planning")

        # No plan
        if ctx.plan is None or len(ctx.plan) == 0:
            node.debug("[plan] No plan yet -> requesting initial plan")
            return _request_plan("initial plan")

        if ctx.expected_board is not None and self._boards_equal(ctx.sensed.board, ctx.expected_board):
            node.debug("[plan] Sensed matches expected -> no replanning needed")
            return HandlerResult(HandlerStatus.NEXT, "sense same as expected")

        node.ui(f"[plan] Sensed differs from expected or expected_board is None -> requesting replanning. plan_index={ctx.plan_index}, plan_len={len(ctx.plan)}, expected_board={'set' if ctx.expected_board else 'None'}")
        return _request_plan("request plan from /plan/solve")

    def _boards_equal(self, s1: Board, s2: Board) -> bool:
        """Check if two boards are equal."""
        if len(s1.pieces) != len(s2.pieces):
            return False
        # Sort pieces by their min col,row for comparison
        def piece_key(piece):
            if not piece.cells:
                return (0, 0)
            return (min(c.col for c in piece.cells), min(c.row for c in piece.cells))
        pieces1 = sorted(s1.pieces, key=piece_key)
        pieces2 = sorted(s2.pieces, key=piece_key)
        for p1, p2 in zip(pieces1, pieces2):
            if p1.type != p2.type or len(p1.cells) != len(p2.cells):
                return False
            cells1 = sorted(p1.cells, key=lambda c: (c.col, c.row))
            cells2 = sorted(p2.cells, key=lambda c: (c.col, c.row))
            for c1, c2 in zip(cells1, cells2):
                if c1.col != c2.col or c1.row != c2.row:
                    return False
        return True
