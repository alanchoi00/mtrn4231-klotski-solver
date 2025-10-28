import { fetchJson, normalizeDigits } from "@/lib/utils";
import { BOARD_SPEC, PATTERNS_DATA_URL } from "../constants";
import type { Block } from "../types";
import { parsePatternToBlocks } from "../utils/pattern-conversion";

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
