import type { BoardSpecMsg } from "@/lib/ros/types";

// ---------- Enums ----------
export const TYPE = { _2x2: 1, _1x2: 2, _2x1: 3, _1x1: 4 } as const;
export const COLOR = { NONE: 0, RED: 1, BLUE: 2, GREEN: 3, YELLOW: 4 } as const;

export type PieceType = (typeof TYPE)[keyof typeof TYPE];
export type PieceColor = (typeof COLOR)[keyof typeof COLOR];

// ---------- Block (UI-only) ----------
export interface Block {
  uid: string; // dnd-kit needs a stable id; not sent to ROS
  color: PieceColor; // COLOR.*
  type: PieceType; // TYPE.*
  col: number; // bottom-left origin
  row: number; // bottom-left origin
  w: number;
  h: number;
}

// ---------- Shape definition for pattern parsing ----------
export interface ShapeDefinition {
  w: number;
  h: number;
  type: PieceType;
  color: PieceColor;
}

// ---------- Board specification ----------
export interface BoardSpec extends BoardSpecMsg {
  cols: number;
  rows: number;
  cell_size_m: number;
  board_thickness_m: number;
}

// ---------- Component props ----------
export interface GoalEditorProps {
  className?: string;
}

export interface DraggableBlockProps {
  block: Block;
  boardSpec: BoardSpec;
}

export interface GridProps {
  blocks: Block[];
  boardSpec: BoardSpec;
  onBlockMove: (blockId: string, newCol: number, newRow: number) => void;
}
