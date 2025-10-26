from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class HandlerStatus(Enum):
    """
    Enum representing the status of a handler in the processing pipeline.

    - NEXT: Handler completed successfully, continue to next handler
    - PENDING: Handler started async operation, pipeline should pause and wait for callback
    - DONE: Handler indicates pipeline should stop for this tick (paused/no work available)
    """
    NEXT = "next"
    PENDING = "pending"
    DONE = "done"


@dataclass
class HandlerResult:
    status: HandlerStatus
    reason: str
