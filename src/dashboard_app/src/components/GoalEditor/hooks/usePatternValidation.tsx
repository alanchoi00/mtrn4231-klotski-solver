import { fetchJson, normalizeDigits } from "@/lib/utils";
import { useEffect, useState } from "react";
import { BOARD_SPEC, DEFAULT_BLOCKS, PATTERNS_DATA_URL } from "../constants";
import { Block } from "../types";
import { blocksToPattern } from "../utils/pattern-conversion";

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
