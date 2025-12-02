from __future__ import annotations

from typing import Protocol

from ..context import BrainContext
from .status import HandlerResult, HandlerStatus


# Minimal protocol to avoid importing rclpy in this file
class BrainNodeLike(Protocol):
    def ui(self, msg: str) -> None: ...
    def debug(self, msg: str) -> None: ...
    def warn(self, msg: str) -> None: ...
    # Async ops that the node implements:
    def start_sense(self) -> bool: ...
    def start_plan(self) -> bool: ...
    def start_execute_next_move(self) -> bool: ...

class BaseHandler:
    name = "base"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        """Public entrypoint for all handlers: includes shared pre-checks."""
        if ctx.goal_reached:
            node.ui(f"[{self.name}] goal reached -> DONE 🎊")
            return HandlerResult(HandlerStatus.DONE, "goal reached")

        # delegate to subclass logic
        return self.do_handle(ctx, node)

    def do_handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult:
        """Default implementation: no-op."""
        return HandlerResult(HandlerStatus.NEXT, "base noop")
