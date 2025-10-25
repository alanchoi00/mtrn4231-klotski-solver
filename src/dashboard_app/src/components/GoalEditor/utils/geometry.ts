import type { Block, BoardSpec } from "../types";

/**
 * Check if a block is within the board bounds
 */
export const inBounds = (block: Block, spec: BoardSpec): boolean => {
  return (
    block.col >= 0 &&
    block.row >= 0 &&
    block.col + block.w <= spec.cols &&
    block.row + block.h <= spec.rows
  );
};

/**
 * Check if two blocks overlap
 */
export const overlaps = (a: Block, b: Block): boolean => {
  return !(
    a.col + a.w <= b.col ||
    b.col + b.w <= a.col ||
    a.row + a.h <= b.row ||
    b.row + b.h <= a.row
  );
};

/**
 * Check if a block collides with any other blocks in the array
 */
export const collideAny = (block: Block, allBlocks: Block[]): boolean => {
  return allBlocks.some(
    (other) => other.uid !== block.uid && overlaps(block, other)
  );
};

/**
 * Create a new block with updated position
 */
export const moveBlock = (
  block: Block,
  deltaCol: number,
  deltaRow: number
): Block => ({
  ...block,
  col: block.col + deltaCol,
  row: block.row + deltaRow,
});
