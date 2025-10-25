from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional

from klotski_interfaces.msg import Board, BoardState, Move

from .ui_modes import UIMode


class ExecutionPhase(IntEnum):
    """Execution phases for the 5-step manipulation sequence."""

    APPROACH = 0     # MovePiece: approach the piece
    GRIP_CLOSE = 1    # GripPiece: close gripper
    PICK_PLACE = 2   # MovePiece: pick and place piece
    GRIP_OPEN = 3   # GripPiece: open gripper
    RETREAT = 4      # MovePiece: retreat to home position

    @classmethod
    def get_name(cls, phase: int) -> str:
        """Get human-readable name for phase."""
        names = {
            cls.APPROACH: "approach",
            cls.GRIP_OPEN: "grip_open",
            cls.PICK_PLACE: "pick_place",
            cls.GRIP_CLOSE: "grip_close",
            cls.RETREAT: "retreat"
        }
        return names.get(phase, f"unknown({phase})")

    @classmethod
    def is_grip_phase(cls, phase: int) -> bool:
        """Check if phase is a gripper operation."""
        return phase in (cls.GRIP_OPEN, cls.GRIP_CLOSE)

    @classmethod
    def is_move_phase(cls, phase: int) -> bool:
        """Check if phase is an arm movement operation."""
        return phase in (cls.APPROACH, cls.PICK_PLACE, cls.RETREAT)

    @classmethod
    def is_last_phase(cls, phase: int) -> bool:
        """Check if this is the last phase in the sequence."""
        return phase == cls.RETREAT

    @classmethod
    def next_phase(cls, current_phase: int) -> 'ExecutionPhase | None':
        """Get next phase safely, returns None if already at last phase."""
        if current_phase >= cls.RETREAT:
            return None
        return cls(current_phase + 1)


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
    current_phase: ExecutionPhase = ExecutionPhase.APPROACH  # current execution phase

    # Book-keeping
    last_error: str = ""
    tick_source: str = ""        # who triggered last tick (debug/telemetry)

    def reset(self):
        self.sensed = None
        self.plan.clear()
        self.plan_index = 0
        self.plan_received = False
        self.busy = False
        self.current_phase = ExecutionPhase.APPROACH
        self.last_error = ""
        self.mode = UIMode.PAUSE     # after reset, remain paused
