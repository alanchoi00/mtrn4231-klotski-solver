from __future__ import annotations

from typing import Protocol, Tuple

from ..context import BrainContext


# Minimal protocol to avoid importing rclpy in this file
class BrainNodeLike(Protocol):
    def ui(self, msg: str) -> None: ...
    def debug(self, msg: str) -> None: ...
    def warn(self, msg: str) -> None: ...
    # Async ops that the node implements:
    def start_sense(self) -> bool: ...
    def start_plan(self) -> bool: ...
    def start_execute_next_move(self) -> bool: ...


# Return values:
#  - "next"    : handled / nothing to do now, continue to next handler
#  - "pending" : async started, stop pipeline; a callback will re-tick
#  - "done"    : terminal for this tick (e.g., paused / no work)
HandlerResult = Tuple[str, str]  # (result, reason)


class BaseHandler:
    name = "base"

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult: # noqa: F841
        return ("next", "base noop")
