from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from klotski_interfaces.msg import Board, BoardState, Move

from .ui_modes import UIMode


@dataclass
class BrainContext:
    # Inputs
    goal: Optional[Board] = None
    sensed: Optional[BoardState] = None

    # Planning
    plan: List[Move] = field(default_factory=list)
    plan_index: int = 0          # next move to execute
    plan_received: bool = False  # flag to distinguish no plan vs empty plan

    # Modes: UICommand constants (IDLE | AUTO | STEP | PAUSE)
    mode: int = UIMode.IDLE

    # Execution flags
    busy: bool = False           # currently sending action
    replan_requested: bool = False  # flag from UI "replan" or after sense

    # Book-keeping
    last_error: str = ""
    tick_source: str = ""        # who triggered last tick (debug/telemetry)

    def reset(self):
        self.sensed = None
        self.plan.clear()
        self.plan_index = 0
        self.plan_received = False
        self.busy = False
        self.replan_requested = False
        self.last_error = ""
        self.mode = UIMode.PAUSE     # after reset, remain paused
