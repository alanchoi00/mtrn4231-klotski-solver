import type { CellMsg, PieceMsg } from "../lib/ros/types";
import type { Block, BoardSpec, ShapeDefinition } from "../types/goal-editor";
import { COLOR, TYPE } from "../types/goal-editor";

/**
 * Convert a Block to ROS PieceMsg format
 */
export const blockToPieceMsg = (block: Block): PieceMsg => {
  const cells: CellMsg[] = [];
  for (let dx = 0; dx < block.w; dx++) {
    for (let dy = 0; dy < block.h; dy++) {
      cells.push({
        col: block.col + dx,
        row: block.row + dy,
      });
    }
  }

  return {
    type: block.type,
    color: block.color,
    cells,
  } as unknown as PieceMsg;
};

/**
 * Encode current blocks to 20-char pattern (row-major, TOP row first)
 */
export const blocksToPattern = (blocks: Block[], spec: BoardSpec): string => {
  const W = spec.cols;
  const H = spec.rows;

  // Create top-origin grid of digits (0 = empty)
  const grid: number[][] = Array.from({ length: H }, () => Array(W).fill(0));

  for (const block of blocks) {
    // Convert bottom-left origin to top-origin
    const topRow = H - 1 - block.row - (block.h - 1);
    const leftCol = block.col;

    for (let dy = 0; dy < block.h; dy++) {
      for (let dx = 0; dx < block.w; dx++) {
        const r = topRow + dy;
        const c = leftCol + dx;
        if (r < 0 || r >= H || c < 0 || c >= W) continue;
        grid[r][c] = block.type;
      }
    }
  }

  // Flatten to string
  let result = "";
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      result += String(grid[r][c] || 0);
    }
  }

  return result;
};

/**
 * Parse a 20-character pattern string into blocks
 */
export const parsePatternToBlocks = (
  pattern: string,
  spec: BoardSpec
): Block[] => {
  if (pattern.length !== spec.cols * spec.rows) {
    console.warn(
      `Invalid pattern length: ${pattern.length}, expected ${
        spec.cols * spec.rows
      }`
    );
    return [];
  }

  const W = spec.cols;
  const H = spec.rows;

  // Create top-origin grid
  const grid: number[][] = Array.from({ length: H }, (_, r) =>
    Array.from({ length: W }, (_, c) => Number(pattern[r * W + c]) || 0)
  );

  const used: boolean[][] = Array.from({ length: H }, () =>
    Array(W).fill(false)
  );

  // Shape definitions (top-origin coordinates)
  const SHAPE: Record<number, ShapeDefinition> = {
    1: { w: 2, h: 2, type: TYPE._2x2, color: COLOR.RED },
    2: { w: 2, h: 1, type: TYPE._1x2, color: COLOR.GREEN },
    3: { w: 1, h: 2, type: TYPE._2x1, color: COLOR.BLUE },
    4: { w: 1, h: 1, type: TYPE._1x1, color: COLOR.YELLOW },
  };

  const blocks: Block[] = [];
  let uid = 1;

  // Helper to check if rectangle is valid
  const isValidRect = (
    r0: number,
    c0: number,
    value: number,
    w: number,
    h: number
  ): boolean => {
    for (let dr = 0; dr < h; dr++) {
      for (let dc = 0; dc < w; dc++) {
        const r = r0 + dr;
        const c = c0 + dc;
        if (r < 0 || r >= H || c < 0 || c >= W) return false;
        if (used[r][c]) return false;
        if (grid[r][c] !== value) return false;
      }
    }
    return true;
  };

  // Helper to mark rectangle as used
  const markUsed = (r0: number, c0: number, w: number, h: number): void => {
    for (let dr = 0; dr < h; dr++) {
      for (let dc = 0; dc < w; dc++) {
        used[r0 + dr][c0 + dc] = true;
      }
    }
  };

  // Scan grid row-major
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      if (used[r][c]) continue;

      const value = grid[r][c];
      if (value === 0) continue;

      const shape = SHAPE[value];
      if (!shape) {
        console.warn(`Unknown digit ${value} at (${r},${c}); skipping`);
        used[r][c] = true;
        continue;
      }

      const { w, h, type, color } = shape;

      if (!isValidRect(r, c, value, w, h)) {
        console.warn(
          `Invalid ${value}-piece geometry at (${r},${c}) expecting ${w}x${h}; skipping`
        );
        used[r][c] = true;
        continue;
      }

      // Convert top-origin to bottom-left origin
      const rowBL = H - 1 - (r + h - 1);
      const colBL = c;

      blocks.push({
        uid: `P${uid++}`,
        color,
        type,
        col: colBL,
        row: rowBL,
        w,
        h,
      });

      markUsed(r, c, w, h);
    }
  }

  return blocks;
};
