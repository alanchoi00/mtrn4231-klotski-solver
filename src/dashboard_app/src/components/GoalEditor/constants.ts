import type { Block, BoardSpec } from "../../types/goal-editor";
import { COLOR, TYPE } from "../../types/goal-editor";

// ---------- Board specification ----------
export const BOARD_SPEC: BoardSpec = {
  cols: 4,
  rows: 5,
  cell_size_m: 0.03,
  board_thickness_m: 0.02,
};

// ---------- UI Colors ----------
export const COLORS: Record<number, string> = {
  [COLOR.RED]: "#dc2626",
  [COLOR.GREEN]: "#16a34a",
  [COLOR.BLUE]: "#2563eb",
  [COLOR.YELLOW]: "#ca8a04",
  [COLOR.NONE]: "#6b7280",
};

// ---------- Default layout ----------
export const DEFAULT_BLOCKS: Block[] = [
  { uid: "u1", color: COLOR.BLUE, type: TYPE._2x1, col: 0, row: 3, w: 1, h: 2 },
  { uid: "u2", color: COLOR.RED, type: TYPE._2x2, col: 1, row: 3, w: 2, h: 2 },
  { uid: "u3", color: COLOR.BLUE, type: TYPE._2x1, col: 3, row: 3, w: 1, h: 2 },
  {
    uid: "u4",
    color: COLOR.GREEN,
    type: TYPE._1x2,
    col: 1,
    row: 2,
    w: 2,
    h: 1,
  },
  { uid: "u5", color: COLOR.BLUE, type: TYPE._2x1, col: 0, row: 0, w: 1, h: 2 },
  {
    uid: "u6",
    color: COLOR.YELLOW,
    type: TYPE._1x1,
    col: 1,
    row: 1,
    w: 1,
    h: 1,
  },
  {
    uid: "u7",
    color: COLOR.YELLOW,
    type: TYPE._1x1,
    col: 2,
    row: 1,
    w: 1,
    h: 1,
  },
  { uid: "u8", color: COLOR.BLUE, type: TYPE._2x1, col: 3, row: 0, w: 1, h: 2 },
  {
    uid: "u9",
    color: COLOR.YELLOW,
    type: TYPE._1x1,
    col: 1,
    row: 0,
    w: 1,
    h: 1,
  },
  {
    uid: "u10",
    color: COLOR.YELLOW,
    type: TYPE._1x1,
    col: 2,
    row: 0,
    w: 1,
    h: 1,
  },
];

// ---------- Data URLs ----------
export const PATTERNS_DATA_URL = "/data/possible_combinations.json";
