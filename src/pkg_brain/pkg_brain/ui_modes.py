"""
UI Command mode constants and utilities for the brain module.

This module provides utilities for working with UICommand mode values
and converting between enum constants and string representations.
"""
from klotski_interfaces.msg import UICommand


class UIMode:
    """UICommand mode constants with utility methods."""

    IDLE = UICommand.MODE_IDLE      # 0
    AUTO = UICommand.MODE_AUTO      # 1
    STEP = UICommand.MODE_STEP      # 2
    PAUSE = UICommand.MODE_PAUSE    # 3
    RESET = UICommand.MODE_RESET    # 4

    # Mapping from enum values to readable names
    _MODE_NAMES = {
        IDLE: "idle",
        AUTO: "auto",
        STEP: "step",
        PAUSE: "paused",  # Note: using "paused" for consistency with existing code
        RESET: "reset",
    }

    # Reverse mapping from names to enum values
    _NAME_TO_MODE = {name: mode for mode, name in _MODE_NAMES.items()}

    @classmethod
    def to_string(cls, mode: int) -> str:
        """Convert UICommand mode constant to string name."""
        return cls._MODE_NAMES.get(mode, f"unknown({mode})")

    @classmethod
    def from_string(cls, name: str) -> int:
        """Convert string name to UICommand mode constant."""
        return cls._NAME_TO_MODE.get(name, cls.IDLE)

    @classmethod
    def is_valid(cls, mode: int) -> bool:
        """Check if mode is a valid UICommand constant."""
        return mode in cls._MODE_NAMES

    @classmethod
    def all_modes(cls) -> list[int]:
        """Get all valid mode constants."""
        return list(cls._MODE_NAMES.keys())

    @classmethod
    def all_names(cls) -> list[str]:
        """Get all valid mode names."""
        return list(cls._MODE_NAMES.values())
