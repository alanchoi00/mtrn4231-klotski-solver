from __future__ import annotations

from ..context import BrainContext
from .base import BaseHandler, BrainNodeLike, HandlerResult


class SenseHandler(BaseHandler):
    name = "sense"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        # Need a board state if we want to plan/execute
        if ctx.sensed is None:
            node.debug("[sense] No BoardState yet → request capture")
            ok = node.start_sense()
            if not ok:
                node.ui("[sense] capture_board service not available")
                return ("done", "sense service not available")
            return ("pending", "capturing board")
        return ("next", "already have sensed board")
