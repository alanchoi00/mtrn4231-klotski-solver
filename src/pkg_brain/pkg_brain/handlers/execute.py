from __future__ import annotations

from ..context import BrainContext, ExecutionPhase
from ..ui_modes import UIMode
from .base import BaseHandler, BrainNodeLike, HandlerResult
from .status import HandlerResult, HandlerStatus


class ExecuteHandler(BaseHandler):
    name = "execute"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        # Only execute if we have received a plan
        if not ctx.plan_received:
            return HandlerResult(HandlerStatus.DONE, "no plan")

        if ctx.plan_index >= len(ctx.plan):
            if len(ctx.plan) == 0:
                node.ui("[exec] Goal already achieved (0 moves needed)")
            else:
                node.ui("[exec] Plan complete")
            return HandlerResult(HandlerStatus.DONE, "complete")

        if ctx.mode == UIMode.PAUSE or ctx.mode == UIMode.IDLE:
            return HandlerResult(HandlerStatus.DONE, f"mode={UIMode.to_string(ctx.mode)}")

        if ctx.busy:
            return HandlerResult(HandlerStatus.DONE, "busy with current action")

        # STEP mode: run exactly one phase and then pause
        if ctx.mode == UIMode.STEP:
            current_phase_name = ExecutionPhase.get_name(ctx.current_phase)
            node.debug(f"[exec] STEP: executing {current_phase_name} phase")
            if node.start_execute_next_move():
                return HandlerResult(HandlerStatus.PENDING, f"executing {current_phase_name} phase")
            else:
                node.ui(f"[exec] manipulation not implemented; skipping phase {current_phase_name}")
                # emulate "done one phase" even if not implemented:
                next_phase = ExecutionPhase.next_phase(ctx.current_phase)
                if next_phase is not None:
                    ctx.current_phase = next_phase
                else:
                    # All phases complete for this move
                    ctx.plan_index += 1
                    ctx.current_phase = ExecutionPhase.APPROACH
                ctx.mode = UIMode.PAUSE
                return HandlerResult(HandlerStatus.DONE, "skipped one phase (not implemented)")

        # AUTO mode: run next phase; result callback will trigger next tick
        if ctx.mode == UIMode.AUTO:
            current_phase_name = ExecutionPhase.get_name(ctx.current_phase)
            node.debug(f"[exec] AUTO: executing {current_phase_name} phase")
            if node.start_execute_next_move():
                return HandlerResult(HandlerStatus.PENDING, f"executing {current_phase_name} phase")
            else:
                node.ui("[exec] manipulation not implemented; cannot execute")
                return HandlerResult(HandlerStatus.DONE, "not implemented")

        return HandlerResult(HandlerStatus.DONE, f"unknown mode={UIMode.to_string(ctx.mode)}")
