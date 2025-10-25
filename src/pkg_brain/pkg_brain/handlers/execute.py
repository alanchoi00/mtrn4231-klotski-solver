from __future__ import annotations

from ..context import BrainContext
from ..ui_modes import UIMode
from .base import BaseHandler, BrainNodeLike, HandlerResult


class ExecuteHandler(BaseHandler):
    name = "execute"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        # Only execute if we have received a plan
        if not ctx.plan_received:
            return ("done", "no plan")

        if ctx.plan_index >= len(ctx.plan):
            if len(ctx.plan) == 0:
                node.ui("[exec] Goal already achieved (0 moves needed)")
            else:
                node.ui("[exec] Plan complete")
            return ("done", "complete")

        if ctx.mode == UIMode.PAUSE or ctx.mode == UIMode.IDLE:
            return ("done", f"mode={UIMode.to_string(ctx.mode)}")

        if ctx.busy:
            return ("done", "busy with current action")

        # STEP mode: run exactly one move and then pause
        if ctx.mode == UIMode.STEP:
            node.debug("[exec] STEP: executing one move")
            if node.start_execute_next_move():
                return ("pending", "executing 1 step")
            else:
                node.ui("[exec] manipulation not implemented; skipping move")
                # emulate "done one step" even if not implemented:
                ctx.plan_index += 1
                ctx.mode = UIMode.PAUSE
                return ("done", "skipped one step (not implemented)")

        # AUTO mode: run next move; result callback will trigger next tick
        if ctx.mode == UIMode.AUTO:
            node.debug("[exec] AUTO: executing next move")
            if node.start_execute_next_move():
                return ("pending", "executing next")
            else:
                node.ui("[exec] manipulation not implemented; cannot execute")
                return ("done", "not implemented")

        return ("done", f"unknown mode={UIMode.to_string(ctx.mode)}")
