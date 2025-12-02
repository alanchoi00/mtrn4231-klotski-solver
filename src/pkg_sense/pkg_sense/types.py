from dataclasses import dataclass


@dataclass(frozen=True)
class BoardConfig:
    """Configuration for the marker board.
    Attributes:
        width: The width of the board in cells.
        height: The height of the board in cells.
        offset_x: The X offset of the board.
        offset_y: The Y offset of the board.
        frame_id: The frame ID of the board.
        tl_marker: The marker ID of the top-left corner.
        tr_marker: The marker ID of the top-right corner.
        bl_marker: The marker ID of the bottom-left corner.
        br_marker: The marker ID of the bottom-right corner.
        marker_length: The length of the markers in meters.
    """

    width: int
    height: int
    offset_x: float
    offset_y: float
    frame_id: str
    tl_marker: int
    tr_marker: int
    bl_marker: int
    br_marker: int
    marker_length: float


@dataclass(frozen=True)
class PieceCounts:
    red: int
    green: int
    blue: int
    yellow: int
    empty: int


@dataclass(frozen=True)
class CameraConfig:
    frame_id: str
    link_frame_id: str


@dataclass(frozen=True)
class FramesConfig:
    world: str
    board: str
