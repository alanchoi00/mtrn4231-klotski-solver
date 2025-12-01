"""Safety logic for hand detection-based robot stopping."""


class SafetyLogic:
    """
    Manages safety stop state based on consecutive frames without hands.

    The safety stop is active (True) when hands are detected. It remains
    active until N consecutive frames have passed without any hands.
    """

    def __init__(self, clear_after_frames: int = 10) -> None:
        """
        Initialize the safety logic.

        Args:
            clear_after_frames: Number of consecutive frames without hands
                required to clear the safety stop.
        """
        if clear_after_frames < 1:
            raise ValueError("clear_after_frames must be >= 1")

        self._clear_after_frames = clear_after_frames
        self._no_hands_count: int = 0
        self._stop_active: bool = False

    @property
    def clear_after_frames(self) -> int:
        """Number of frames required to clear safety stop."""
        return self._clear_after_frames

    @property
    def no_hands_count(self) -> int:
        """Current count of consecutive frames without hands."""
        return self._no_hands_count

    @property
    def is_stop_active(self) -> bool:
        """Current safety stop state."""
        return self._stop_active

    def update(self, has_hands: bool) -> bool:
        """
        Update safety state based on whether hands were detected.

        Args:
            has_hands: True if at least one hand was detected in current frame.

        Returns:
            True if safety stop must be active, False otherwise.
        """
        if has_hands:
            # Hands detected - activate stop and reset counter
            self._stop_active = True
            self._no_hands_count = 0
        else:
            # No hands - increment counter
            self._no_hands_count += 1

            # Clear stop only after enough consecutive frames without hands
            if self._no_hands_count >= self._clear_after_frames:
                self._stop_active = False

        return self._stop_active

    def reset(self) -> None:
        """Reset the safety logic to initial state."""
        self._no_hands_count = 0
        self._stop_active = False
