from __future__ import annotations

from ..context import BrainContext
from .base import BaseHandler, BrainNodeLike, HandlerResult
from .status import HandlerStatus


class PlanHandler(BaseHandler):
    name = "plan"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        # Need both goal and sensed to plan
        if ctx.goal is None:
            node.debug("[plan] No goal yet → waiting")
            return HandlerResult(HandlerStatus.DONE, "waiting goal")

        if ctx.sensed is None:
            return HandlerResult(HandlerStatus.NEXT, "sense first")
        
        if ctx.sensed == ctx.plan[ctx.plan_index-1]:
            ctx.plan_index -= 1
            return HandlerResult(HandlerStatus.NEXT, "sense same as prev")


        node.debug("[plan] Request plan from /plan/solve")
        ok = node.start_plan()
        if not ok:
            node.ui("[plan] planner service not available")
            return HandlerResult(HandlerStatus.DONE, "planner not available")
        return HandlerResult(HandlerStatus.PENDING, "planning")
