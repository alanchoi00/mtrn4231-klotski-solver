from __future__ import annotations

from ..context import BrainContext
from .base import BaseHandler, BrainNodeLike, HandlerResult


class PlanHandler(BaseHandler):
    name = "plan"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        # Need both goal and sensed to plan
        if ctx.goal is None:
            node.debug("[plan] No goal yet → waiting")
            return ("done", "waiting goal")

        if ctx.sensed is None:
            return ("next", "sense first")

        need_plan = ctx.replan_requested or (not ctx.plan_received)
        if not need_plan:
            return ("next", "have plan")

        node.debug("[plan] Request plan from /plan/solve")
        ok = node.start_plan()
        if not ok:
            node.ui("[plan] planner service not available")
            return ("done", "planner not available")
        return ("pending", "planning")
