from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum
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
    def get_name(cls, phase: ExecutionPhase | int) -> str:
        """Get human-readable name for phase."""
        names = {
            cls.APPROACH: "approach",
            cls.GRIP_OPEN: "grip_open",
            cls.PICK_PLACE: "pick_place",
            cls.GRIP_CLOSE: "grip_close",
            cls.RETREAT: "retreat"
        }
        try:
            key = cls(phase)
        except (ValueError, TypeError):
            return f"unknown({phase})"
        return names.get(key, f"unknown({phase})")

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

class TickSource(Enum):
    """Source that can trigger a pipeline tick."""
    BOARD_STATE = "board_state"
    PLAN_DONE = "plan_done"
    SENSE_DONE = "sense_done"
    UI_COMMAND = "ui_command"
    GOAL_UPDATE = "goal_update"
    EXEC_FAILED = "exec_failed"
    EXEC_NEXT_PHASE = "exec_next_phase"
    EXEC_DONE = "exec_done"

    @classmethod
    def exec_related(cls) -> List[TickSource]:
        """Tick sources that are related to execution."""
        return [
            TickSource.EXEC_FAILED,
            TickSource.EXEC_NEXT_PHASE,
            TickSource.EXEC_DONE,
        ]

    @classmethod
    def exec_isolated(cls) -> List[TickSource]:
        """Tick sources that are isolated to execution only."""
        return [TickSource.EXEC_NEXT_PHASE]

    @classmethod
    def skip_sense(cls) -> List[TickSource]:
        """Tick sources that should skip sensing."""
        return [
            TickSource.SENSE_DONE,
            TickSource.BOARD_STATE,
            TickSource.PLAN_DONE,
            *cls.exec_isolated(),
        ]

    @classmethod
    def skip_plan(cls) -> List[TickSource]:
        """Tick sources that should skip planning."""
        return [
            TickSource.PLAN_DONE,
            *cls.exec_isolated(),
        ]


@dataclass
class BrainContext:
    # Inputs
    goal: Optional[Board] = None
    sensed: Optional[BoardState] = None

    expected_board: Optional[Board] = None  # expected board after last move

    # Planning
    plan: List[Move] = field(default_factory=list)
    plan_index: int = 0          # next move to execute

    # Modes: UICommand constants (IDLE | AUTO | STEP | PAUSE)
    mode: int = UIMode.IDLE

    # Execution flags
    busy: bool = False           # currently sending action
    current_phase: ExecutionPhase = ExecutionPhase.APPROACH  # current execution phase

    # Book-keeping
    last_error: str = ""
    tick_source: TickSource | None = None        # who triggered last tick (debug/telemetry)

    def reset(self):
        self.sensed = None
        self.plan.clear()
        self.plan_index = 0
        self.busy = False
        self.current_phase = ExecutionPhase.APPROACH
        self.revalidating = False
        self.last_error = ""
        self.mode = UIMode.PAUSE     # after reset, remain paused

    def get_last_executed_move(self) -> Optional[Move]:
        """Get the last executed move, if any."""
        if self.plan_index == 0:
            return None
        try:
            return self.plan[self.plan_index - 1]
        except Exception:
            return None
