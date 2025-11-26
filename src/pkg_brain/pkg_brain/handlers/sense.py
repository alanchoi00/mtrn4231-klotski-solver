from __future__ import annotations

from ..context import BrainContext, TickSource
from .base import BaseHandler, BrainNodeLike, HandlerResult
from .status import HandlerStatus


class SenseHandler(BaseHandler):
    name = "sense"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:

        if ctx.tick_source in TickSource.skip_sense():
            return HandlerResult(HandlerStatus.NEXT, f"skip sense for tick_source={ctx.tick_source}")

        ok = node.start_sense()
        if not ok:
            node.ui("[sense] capture_board service not available")
            return HandlerResult(HandlerStatus.DONE, "sense service not available")
        return HandlerResult(HandlerStatus.PENDING, "capturing board")
