from __future__ import annotations

from ..context import BrainContext
from .base import BaseHandler, BrainNodeLike, HandlerResult
from .status import HandlerStatus


class SenseHandler(BaseHandler):
    name = "sense"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        # Need a board state if we want to plan/execute
        if ctx.sensed is None:
            node.debug("[sense] No BoardState yet -> request capture")
            ok = node.start_sense()
            if not ok:
                node.ui("[sense] capture_board service not available")
                return HandlerResult(HandlerStatus.DONE, "sense service not available")
            return HandlerResult(HandlerStatus.PENDING, "capturing board")
        return HandlerResult(HandlerStatus.NEXT, "already have sensed board")
