import { toast } from "sonner";
import { BOARD_SPEC } from "../constants";
import { Block } from "../types";
import { collideAny, inBounds, moveBlock } from "../utils/geometry";

export const useBlockMovement = (
  blocks: Block[],
  setBlocks: (blocks: Block[]) => void,
  validatePattern: (blocks: Block[]) => boolean,
  setIsValid: (valid: boolean) => void
) => {
  const handleBlockMove = (blockId: string, newCol: number, newRow: number) => {
    const originalBlock = blocks.find((b) => b.uid === blockId);
    if (!originalBlock) return;

    const deltaCol = newCol - originalBlock.col;
    const deltaRow = newRow - originalBlock.row;
    const movedBlock = moveBlock(originalBlock, deltaCol, deltaRow);

    // Validate bounds and collisions
    if (!inBounds(movedBlock, BOARD_SPEC) || collideAny(movedBlock, blocks)) {
      return;
    }

    // Create candidate layout
    const candidateBlocks = blocks.map((b) =>
      b.uid === blockId ? movedBlock : b
    );

    // Validate pattern
    if (validatePattern(candidateBlocks)) {
      setBlocks(candidateBlocks);
      setIsValid(true);
    } else {
      setIsValid(false);
      toast.warning("Pattern not possible 😢 reverting move.");
    }
  };

  return { handleBlockMove };
};
