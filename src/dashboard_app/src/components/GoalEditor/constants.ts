import { PieceColorType, PieceType } from "@/lib/ros/types/piece.msg";
import type { Block, BoardSpec } from "./types";

export const BOARD_SPEC: BoardSpec = {
  cols: 4,
  rows: 5,
  cell_size_m: 0.03,
  board_thickness_m: 0.02,
};

export const COLORS: Record<number, string> = {
  [PieceColorType.COLOR_RED]: "#dc2626",
  [PieceColorType.COLOR_GREEN]: "#16a34a",
  [PieceColorType.COLOR_BLUE]: "#2563eb",
  [PieceColorType.COLOR_YELLOW]: "#ca8a04",
  [PieceColorType.COLOR_NONE]: "#6b7280",
};

export const DEFAULT_BLOCKS: Block[] = [
  {
    uid: "u1",
    color: PieceColorType.COLOR_BLUE,
    type: PieceType.TYPE_2_1,
    col: 0,
    row: 3,
    w: 1,
    h: 2,
  },
  {
    uid: "u2",
    color: PieceColorType.COLOR_RED,
    type: PieceType.TYPE_2_2,
    col: 1,
    row: 3,
    w: 2,
    h: 2,
  },
  {
    uid: "u3",
    color: PieceColorType.COLOR_BLUE,
    type: PieceType.TYPE_2_1,
    col: 3,
    row: 3,
    w: 1,
    h: 2,
  },
  {
    uid: "u4",
    color: PieceColorType.COLOR_GREEN,
    type: PieceType.TYPE_1_2,
    col: 1,
    row: 2,
    w: 2,
    h: 1,
  },
  {
    uid: "u5",
    color: PieceColorType.COLOR_BLUE,
    type: PieceType.TYPE_2_1,
    col: 0,
    row: 0,
    w: 1,
    h: 2,
  },
  {
    uid: "u6",
    color: PieceColorType.COLOR_YELLOW,
    type: PieceType.TYPE_1_1,
    col: 1,
    row: 1,
    w: 1,
    h: 1,
  },
  {
    uid: "u7",
    color: PieceColorType.COLOR_YELLOW,
    type: PieceType.TYPE_1_1,
    col: 2,
    row: 1,
    w: 1,
    h: 1,
  },
  {
    uid: "u8",
    color: PieceColorType.COLOR_BLUE,
    type: PieceType.TYPE_2_1,
    col: 3,
    row: 0,
    w: 1,
    h: 2,
  },
  {
    uid: "u9",
    color: PieceColorType.COLOR_YELLOW,
    type: PieceType.TYPE_1_1,
    col: 1,
    row: 0,
    w: 1,
    h: 1,
  },
  {
    uid: "u10",
    color: PieceColorType.COLOR_YELLOW,
    type: PieceType.TYPE_1_1,
    col: 2,
    row: 0,
    w: 1,
    h: 1,
  },
];

export const PATTERNS_DATA_URL = "/data/possible_combinations.json";
