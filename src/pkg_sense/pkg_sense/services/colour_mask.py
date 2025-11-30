from typing import Dict

import cv2
import numpy as np
from ..constants import ColourName
from .hsv_config import HSVConfig

Masks = Dict[ColourName, np.ndarray]


def build_colour_masks(img: np.ndarray, hsv: HSVConfig) -> Masks:
    """
    Build colour masks from an image using HSV ranges.
    """
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    def mask(range_name: str) -> np.ndarray:
        r = hsv.ranges[range_name]
        return cv2.inRange(hsv_img, r.low, r.high)

    return {
        "red": mask("red1") | mask("red2"),
        "yellow": mask("yellow"),
        "green": mask("green"),
        "blue": mask("blue"),
    }

def clean_masks(masks: Masks) -> Masks:
    """Clean noise from masks using morphological operations."""
    kernel_size = (5, 5) # sized 5x5 for more aggressive noise removal
    kernel = np.ones(kernel_size, np.uint8)
    for mask in  masks.values():
        cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return masks

def build_masks(img: np.ndarray, hsv: HSVConfig) -> Masks:
    """Build and clean colour masks from an image using HSV ranges."""
    masks = build_colour_masks(img, hsv)
    return clean_masks(masks)




