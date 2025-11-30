from klotski_interfaces.msg._board import Board
from klotski_interfaces.msg._piece import Piece

from ..types import PieceCounts
from ..exceptions import InvalidBoardConfiguration
from .board_image_annotation import Grid


def validate_board_configuration(
    grid: Grid,
    board: Board,
    board_width_cells: int,
    board_height_cells: int,
    piece_counts: PieceCounts,
):
    cnt_red = sum(
        1
        for p in board.pieces
        if p.color == Piece.COLOR_RED and p.type == Piece.TYPE_2_2
    )
    cnt_green = sum(
        1
        for p in board.pieces
        if p.color == Piece.COLOR_GREEN and p.type == Piece.TYPE_1_2
    )
    cnt_blue = sum(
        1
        for p in board.pieces
        if p.color == Piece.COLOR_BLUE and p.type == Piece.TYPE_2_1
    )
    cnt_yel = sum(
        1
        for p in board.pieces
        if p.color == Piece.COLOR_YELLOW and p.type == Piece.TYPE_1_1
    )
    empty_cells = sum(
        1
        for r in range(board_height_cells)
        for c in range(board_width_cells)
        if grid[r][c] == "empty"
    )
    ok = (
        cnt_blue == piece_counts.blue
        and cnt_red == piece_counts.red
        and cnt_green == piece_counts.green
        and cnt_yel == piece_counts.yellow
        and empty_cells == piece_counts.empty
    )
    if not ok:
        raise InvalidBoardConfiguration(
            f"Invalid counts -> blue:{cnt_blue} red:{cnt_red} green:{cnt_green} "
            f"yellow:{cnt_yel} empty:{empty_cells} (want 4,1,1,4,2)"
        )
