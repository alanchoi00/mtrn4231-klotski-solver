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

    def handle(self, ctx: BrainContext, node: BrainNodeLike) -> HandlerResult: # noqa: F841
        return HandlerResult(HandlerStatus.NEXT, "base noop")
