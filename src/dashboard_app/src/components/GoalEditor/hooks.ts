import type { Block } from "@/types/goal-editor";
import { fetchJson, normalizeDigits } from "@/utils/common";
import { collideAny, inBounds, moveBlock } from "@/utils/geometry";
import {
  blocksToPattern,
  parsePatternToBlocks,
} from "@/utils/pattern-conversion";
import { useEffect, useState } from "react";
import { toast } from "sonner";
import { BOARD_SPEC, DEFAULT_BLOCKS, PATTERNS_DATA_URL } from "./constants";

export const usePatternValidation = () => {
  const [allowedPatterns, setAllowedPatterns] = useState<Set<string>>(
    new Set()
  );
  const [isValid, setIsValid] = useState<boolean>(true);

  useEffect(() => {
    const loadPatterns = async () => {
      const data = await fetchJson<string[]>(PATTERNS_DATA_URL);
      if (data) {
        const patterns = new Set(data.map(normalizeDigits));
        setAllowedPatterns(patterns);
        setIsValid(patterns.has(blocksToPattern(DEFAULT_BLOCKS, BOARD_SPEC)));
      }
    };

    loadPatterns();
  }, []);

  const validatePattern = (blocks: Block[]): boolean => {
    if (allowedPatterns.size === 0) return true;
    const pattern = blocksToPattern(blocks, BOARD_SPEC);
    return allowedPatterns.has(pattern);
  };

  return { allowedPatterns, isValid, setIsValid, validatePattern };
};

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

export const useRandomPattern = (
  setBlocks: (blocks: Block[]) => void,
  allowedPatterns: Set<string>,
  setIsValid: (valid: boolean) => void
) => {
  const loadRandomPattern = async () => {
    const data = await fetchJson<string[]>(PATTERNS_DATA_URL);
    if (!data || data.length === 0) return;

    const randomPattern = normalizeDigits(
      data[Math.floor(Math.random() * data.length)]
    );

    const newBlocks = parsePatternToBlocks(randomPattern, BOARD_SPEC);
    setBlocks(newBlocks);
    setIsValid(
      allowedPatterns.size ? allowedPatterns.has(randomPattern) : true
    );
  };

  return { loadRandomPattern };
};
