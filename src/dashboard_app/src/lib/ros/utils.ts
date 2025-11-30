import { BoardMsg } from "./types";

/**
 * Convert a BoardMsg to a 20-character pattern string for display
 */
export const boardToPattern = (board: BoardMsg): string => {
  const W = board.spec.cols;
  const H = board.spec.rows;

  // Create top-origin grid of digits (0 = empty)
  const grid: number[][] = Array.from({ length: H }, () => Array(W).fill(0));

  // Fill grid with piece types at their cell positions
  for (const piece of board.pieces) {
    for (const cell of piece.cells) {
      const { col, row } = cell;

      // Convert bottom-left origin (ROS) to top-origin (pattern display)
      const topRow = H - 1 - row;
      const leftCol = col;

      if (topRow >= 0 && topRow < H && leftCol >= 0 && leftCol < W) {
        grid[topRow][leftCol] = piece.type;
      }
    }
  }

  // Flatten to string (row-major, top row first)
  let result = "";
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      result += String(grid[r][c] || 0);
    }
  }

  return result;
};
