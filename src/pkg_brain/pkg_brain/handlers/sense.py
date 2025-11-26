from __future__ import annotations

from ..context import BrainContext, Phase
from .base import BaseHandler, BrainNodeLike, HandlerResult
from .status import HandlerStatus


class SenseHandler(BaseHandler):
    name = "sense"

    def do_handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:

        if not Phase.is_sense_phase(ctx.current_phase):
            return HandlerResult(HandlerStatus.NEXT, f"not sensing phase (current_phase={ctx.current_phase})")

        ok = node.start_sense()
        if not ok:
            node.ui("[sense] capture_board service not available")
            return HandlerResult(HandlerStatus.DONE, "sense service not available")
        return HandlerResult(HandlerStatus.PENDING, "capturing board")
