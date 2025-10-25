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

        need_plan = ctx.replan_requested or (not ctx.plan_received)
        if not need_plan:
            return HandlerResult(HandlerStatus.NEXT, "have plan")

        node.debug("[plan] Request plan from /plan/solve")
        ok = node.start_plan()
        if not ok:
            node.ui("[plan] planner service not available")
            return HandlerResult(HandlerStatus.DONE, "planner not available")
        return HandlerResult(HandlerStatus.PENDING, "planning")
