"""Hand detection service using MediaPipe Hands."""

from dataclasses import dataclass
from typing import List, Tuple

import cv2
import mediapipe as mp
import numpy as np


@dataclass(frozen=True)
class HandResult:
    """Result of a single hand detection."""

    confidence: float
    """Detection confidence score (0.0 to 1.0)."""

    handedness: str
    """'Left' or 'Right' hand classification."""

    landmarks: Tuple[Tuple[float, float, float], ...]
    """Normalized (x, y, z) coordinates of 21 hand landmarks."""


class HandDetector:
    """
    Hand detection using MediaPipe Hands.

    Initializes the MediaPipe Hands solution once at startup to avoid
    reloading the graph on every frame.
    """

    def __init__(
        self,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        max_num_hands: int = 2,
    ) -> None:
        """
        Initialize the hand detector.

        Args:
            min_detection_confidence: Minimum confidence for hand detection.
            min_tracking_confidence: Minimum confidence for hand tracking.
            max_num_hands: Maximum number of hands to detect.
        """
        self._mp_hands = mp.solutions.hands
        self._hands = self._mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )

    def detect_hands(self, image_bgr: np.ndarray) -> List[HandResult]:
        """
        Detect hands in a BGR image.

        Args:
            image_bgr: Input image in BGR format (OpenCV default).

        Returns:
            List of HandResult objects for each detected hand.
        """
        # MediaPipe requires RGB input
        image_rgb = image_bgr[:, :, ::-1]

        results = self._hands.process(image_rgb)

        if not results.multi_hand_landmarks:
            return []

        hand_results: List[HandResult] = []

        for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
            # Get handedness info
            handedness_info = results.multi_handedness[idx]
            classification = handedness_info.classification[0]

            # Extract landmarks as tuple of tuples
            landmarks = tuple((lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark)

            hand_results.append(
                HandResult(
                    confidence=classification.score,
                    handedness=classification.label,
                    landmarks=landmarks,
                )
            )

        return hand_results

    def draw_annotations(
        self, image_bgr: np.ndarray, hand_results: List[HandResult]
    ) -> np.ndarray:
        """
        Draw hand landmarks and connections on the image.

        Args:
            image_bgr: Input image in BGR format.
            hand_results: List of HandResult objects from detect_hands().

        Returns:
            Annotated image with hand landmarks drawn.
        """
        annotated = image_bgr.copy()
        h, w = annotated.shape[:2]

        mp_drawing = mp.solutions.drawing_utils
        mp_drawing_styles = mp.solutions.drawing_styles

        for hand in hand_results:
            # Convert normalized landmarks to pixel coordinates for drawing
            # Create a landmark proto for MediaPipe drawing utils
            landmark_list = self._mp_hands.HandLandmark

            # Draw landmarks manually with custom style
            points = [(int(lm[0] * w), int(lm[1] * h)) for lm in hand.landmarks]

            # Draw connections
            connections = self._mp_hands.HAND_CONNECTIONS
            for connection in connections:
                start_idx, end_idx = connection
                cv2.line(
                    annotated,
                    points[start_idx],
                    points[end_idx],
                    (0, 255, 0),  # Green
                    2,
                )

            # Draw landmarks
            for point in points:
                cv2.circle(annotated, point, 5, (0, 0, 255), -1)  # Red filled
                cv2.circle(annotated, point, 5, (255, 255, 255), 1)  # White border

            # Draw label with confidence
            wrist = points[0]
            label = f"{hand.handedness}: {hand.confidence:.0%}"
            cv2.putText(
                annotated,
                label,
                (wrist[0] - 30, wrist[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                label,
                (wrist[0] - 30, wrist[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                1,
            )

        return annotated

    def close(self) -> None:
        """Release MediaPipe resources."""
        self._hands.close()

    def __enter__(self) -> "HandDetector":
        return self

    def __exit__(self, *args) -> None:
        self.close()
