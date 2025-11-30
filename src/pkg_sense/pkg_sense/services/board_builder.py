from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

from klotski_interfaces.msg import Board, BoardSpec, Cell, Piece

from ..constants import ColourName
from ..exceptions import LeftoverCellsError


@dataclass(frozen=True)
class GridDimensions:
    rows: int
    cols: int


@dataclass(frozen=True)
class CellIndex:
    row: int
    col: int


@dataclass
class CellGrid:
    """
    Represents a grid of cell colours.

    Assumes (row, col) with row=0 at bottom and col=0 at left, matching
    the logical board coordinate system.
    """

    dims: GridDimensions
    data: List[List[ColourName]]  # shape [rows][cols]

    def colour_at(self, idx: CellIndex) -> ColourName:
        return self.data[idx.row][idx.col]


def append_cell_to_piece(piece: Piece, idx: CellIndex) -> None:
    cell = Cell()
    cell.row = int(idx.row)
    cell.col = int(idx.col)
    piece.cells = [*piece.cells, cell]


def extract_red_blocks(
    grid: CellGrid,
    visited: List[List[bool]],
) -> List[Piece]:
    """
    Find all 2x2 red blocks.

    Pattern:
    [r r]
    [r r]
    """
    pieces: List[Piece] = []
    rows, cols = grid.dims.rows, grid.dims.cols

    for r in range(rows - 1):
        for c in range(cols - 1):
            block_indices = [
                CellIndex(r, c),
                CellIndex(r + 1, c),
                CellIndex(r, c + 1),
                CellIndex(r + 1, c + 1),
            ]

            if any(visited[idx.row][idx.col] for idx in block_indices):
                continue

            if all(grid.colour_at(idx) == "red" for idx in block_indices):
                piece = Piece()
                piece.type = Piece.TYPE_2_2
                piece.color = Piece.COLOR_RED

                for idx in block_indices:
                    append_cell_to_piece(piece, idx)
                    visited[idx.row][idx.col] = True

                pieces.append(piece)

    return pieces


def extract_green_blocks(
    grid: CellGrid,
    visited: List[List[bool]],
) -> List[Piece]:
    """
    Find all horizontal 2x1 green blocks.

    Pattern in a row:
    [g g]
    """
    pieces: List[Piece] = []
    rows, cols = grid.dims.rows, grid.dims.cols

    for r in range(rows):
        c = 0
        while c < cols:
            if visited[r][c] or grid.data[r][c] != "green":
                c += 1
                continue

            run_start = c
            while c < cols and not visited[r][c] and grid.data[r][c] == "green":
                c += 1

            run_len = c - run_start
            num_pairs = run_len // 2

            for k in range(num_pairs):
                c0 = run_start + 2 * k
                idx1 = CellIndex(r, c0)
                idx2 = CellIndex(r, c0 + 1)

                piece = Piece()
                piece.type = Piece.TYPE_1_2
                piece.color = Piece.COLOR_GREEN

                append_cell_to_piece(piece, idx1)
                append_cell_to_piece(piece, idx2)
                visited[idx1.row][idx1.col] = True
                visited[idx2.row][idx2.col] = True
                pieces.append(piece)

    return pieces


def extract_blue_blocks(
    grid: CellGrid,
    visited: List[List[bool]],
) -> List[Piece]:
    """
    Find all vertical 1x2 blue blocks.

    Pattern in a column:
    [b]
    [b]
    """
    pieces: List[Piece] = []
    rows, cols = grid.dims.rows, grid.dims.cols

    for c in range(cols):
        r = 0
        while r < rows:
            if visited[r][c] or grid.data[r][c] != "blue":
                r += 1
                continue

            run_start = r
            while r < rows and not visited[r][c] and grid.data[r][c] == "blue":
                r += 1

            run_len = r - run_start
            num_pairs = run_len // 2

            for k in range(num_pairs):
                r0 = run_start + 2 * k
                idx1 = CellIndex(r0, c)
                idx2 = CellIndex(r0 + 1, c)

                piece = Piece()
                piece.type = Piece.TYPE_2_1
                piece.color = Piece.COLOR_BLUE

                append_cell_to_piece(piece, idx1)
                append_cell_to_piece(piece, idx2)
                visited[idx1.row][idx1.col] = True
                visited[idx2.row][idx2.col] = True
                pieces.append(piece)

    return pieces


def extract_yellow_blocks(
    grid: CellGrid,
    visited: List[List[bool]],
) -> List[Piece]:
    """
    Find all 1x1 yellow blocks.
    """
    pieces: List[Piece] = []
    rows, cols = grid.dims.rows, grid.dims.cols

    for r in range(rows):
        for c in range(cols):
            if visited[r][c] or grid.data[r][c] != "yellow":
                continue

            idx = CellIndex(r, c)
            piece = Piece()
            piece.type = Piece.TYPE_1_1
            piece.color = Piece.COLOR_YELLOW

            append_cell_to_piece(piece, idx)
            visited[r][c] = True

            pieces.append(piece)

    return pieces


def find_leftover_cells(
    grid: CellGrid,
    visited: List[List[bool]],
) -> List[Tuple[int, int, ColourName]]:
    """
    Return any coloured cells that were not consumed into a piece.
    """
    rows, cols = grid.dims.rows, grid.dims.cols
    leftovers: List[Tuple[int, int, ColourName]] = []

    for r in range(rows):
        for c in range(cols):
            colour = grid.data[r][c]
            if not visited[r][c] and colour != "empty":
                leftovers.append((r, c, colour))

    return leftovers


def grid_to_board(
    grid_raw: List[List[ColourName]],
    board_width_cells: int,
    board_height_cells: int,
    cell_size_m: float = 0.05,
    board_thickness_m: float = 0.08,
) -> Board:
    """
    Convert a 2D grid of colour labels into a klotski_interfaces/Board message.

    Args:
        board_width_cells: Number of columns in the board.
        board_height_cells: Number of rows in the board.
        grid_raw: 2D list of colour names with shape [rows][cols],
                  using ColourName literals.
        cell_size_m: Physical size of one cell in meters.
        board_thickness_m: Physical thickness of the board in meters.

    Returns:
        A populated Board message with spec and pieces.

    Raises:
        ValueError: If the input grid dimensions do not match the specified board size.
        LeftoverCellsError: If there are leftover coloured cells not forming valid pieces.
    """
    dims = GridDimensions(rows=board_height_cells, cols=board_width_cells)

    # Safety: basic shape check
    if len(grid_raw) != dims.rows:
        raise ValueError(
            f"grid_to_board: expected {dims.rows} rows, got {len(grid_raw)}"
        )
    for row in grid_raw:
        if len(row) != dims.cols:
            raise ValueError(
                f"grid_to_board: expected {dims.cols} cols in each row, got {len(row)}"
            )

    grid = CellGrid(dims=dims, data=grid_raw)

    visited: List[List[bool]] = [
        [False for _ in range(dims.cols)] for _ in range(dims.rows)
    ]

    board_msg = Board()
    board_msg.pieces = [
        *extract_red_blocks(grid, visited),
        *extract_green_blocks(grid, visited),
        *extract_blue_blocks(grid, visited),
        *extract_yellow_blocks(grid, visited),
    ]

    leftovers = find_leftover_cells(grid, visited)
    if leftovers:
        raise LeftoverCellsError(
            f"Leftover coloured cells not forming valid pieces: {leftovers}"
        )

    spec = BoardSpec()
    spec.cols = board_width_cells
    spec.rows = board_height_cells
    spec.cell_size_m = cell_size_m
    spec.board_thickness_m = board_thickness_m
    board_msg.spec = spec

    return board_msg
