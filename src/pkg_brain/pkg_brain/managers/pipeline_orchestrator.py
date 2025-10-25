from __future__ import annotations

from typing import TYPE_CHECKING, List

from ..handlers import (BaseHandler, ExecuteHandler, HandlerStatus,
                        PlanHandler, SenseHandler)
from ..ui_modes import UIMode

if TYPE_CHECKING:
    from rclpy.node import Node


class PipelineOrchestrator:
    """Coordinates the handler pipeline execution."""

    def __init__(self, node: Node):
        self.node = node
        self.pipeline: List[BaseHandler] = [SenseHandler(), PlanHandler(), ExecuteHandler()]

    def tick(self, source: str) -> None:
        """Execute the pipeline tick."""
        from ..task_brain import TaskBrain  # Avoid circular import
        brain = self.node
        if not isinstance(brain, TaskBrain):
            return

        brain.ctx.tick_source = source
        mode_name = UIMode.to_string(brain.ctx.mode)
        brain.ui_manager.debug(f"[tick] source={source} mode={mode_name} busy={brain.ctx.busy} plan_idx={brain.ctx.plan_index}/{len(brain.ctx.plan)}")

        for handler in self.pipeline:
            result = handler.handle(brain.ctx, brain)
            brain.ui_manager.debug(f"  -> handler={handler.name} status={result.status.value} reason={result.reason}")
            if result.status == HandlerStatus.PENDING or result.status == HandlerStatus.DONE:
                break
            # else HandlerStatus.NEXT -> continue
