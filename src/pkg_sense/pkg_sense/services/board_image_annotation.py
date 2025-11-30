from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np

from ..constants import ColourName
from .colour_mask import Masks

Grid = List[List[ColourName]]

@dataclass
class CellBounds:
    x0: int
    y0: int
    x1: int
    y1: int

    @staticmethod
    def from_row_col(row: int, col: int, cell_w: float, cell_h: float) -> "CellBounds":
        x0 = int(round(col * cell_w))
        x1 = int(round((col + 1) * cell_w))
        y0 = int(round(row * cell_h))
        y1 = int(round((row + 1) * cell_h))
        return CellBounds(x0, y0, x1, y1)

    def to_slices(self) -> Tuple[slice, slice]:
        return slice(self.y0, self.y1), slice(self.x0, self.x1)

    def compute_colour_area(self, mask: np.ndarray) -> int:
        cell_slice = self.to_slices()
        return int(np.count_nonzero(mask[cell_slice]))



def determine_cell_color(
    areas: Dict[ColourName, int],
    min_area: int,
) -> ColourName:
    if not areas:
        return "empty"

    best_colour = max(areas, key=lambda k: areas[k])
    best_area = areas[best_colour]

    if best_area < min_area:
        return "empty"
    return best_colour

def annotate_cell_colors(
    image: np.ndarray,
    masks: Masks,
    min_cell_colour_area: int,
    board_width_cells: int,
    board_height_cells: int,
) -> Grid:
    """
    Classify each grid cell based on colour masks.

    Responsibility: return an HxW grid of logical colour labels
    (bottom-left origin). No drawing, no mask creation.
    """
    img_h, img_w = image.shape[:2]
    cell_w = img_w / board_width_cells
    cell_h = img_h / board_height_cells

    grid_top: Grid = []
    for r_top in range(board_height_cells):
        row: List[ColourName] = []
        for c in range(board_width_cells):
            bounds = CellBounds.from_row_col(r_top, c, cell_w, cell_h)
            areas: Dict[ColourName, int] = {name: bounds.compute_colour_area(mask) for name, mask in masks.items()}
            colour = determine_cell_color(areas, min_cell_colour_area)
            row.append(colour)
        grid_top.append(row)

    height = len(grid_top)
    width = len(grid_top[0]) if height > 0 else 0
    return [
        [grid_top[height - 1 - r][c] for c in range(width)]
        for r in range(height)
    ]


def render_annotated_grid_overlay_image(
    image: np.ndarray,
    grid_bottom: Grid,
    board_width_cells: int,
    board_height_cells: int,
) -> np.ndarray:
    """
    Build an annotated overlay image given a classified bottom-origin grid.
    Responsibility: drawing only.
    """
    img_h, img_w = image.shape[:2]
    cell_w = img_w / board_width_cells
    cell_h = img_h / board_height_cells

    overlay = image.copy()

    for r_bot, row in enumerate(grid_bottom):
        for c, colour in enumerate(row):
            r_top = board_height_cells - 1 - r_bot
            bounds = CellBounds.from_row_col(r_top, c, cell_w, cell_h)
            y_slice, x_slice = bounds.to_slices()
            cv2.rectangle(
                overlay,
                (x_slice.start, y_slice.start),
                (x_slice.stop, y_slice.stop),
                (0, 0, 0),
                1
            )
            cv2.putText(
                overlay,
                colour,
                (x_slice.start + 3, y_slice.start + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 0),
                1,
            )

    return overlay
