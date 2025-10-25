import type { BoardSpecMsg, PieceColorType, PieceType } from "@/lib/ros";

export interface Block {
  uid: string; // dnd-kit needs a stable id; not sent to ROS
  color: PieceColorType; // COLOR.*
  type: PieceType; // TYPE.*
  col: number; // bottom-left origin
  row: number; // bottom-left origin
  w: number;
  h: number;
}

export interface ShapeDefinition {
  w: number;
  h: number;
  type: PieceType;
  color: PieceColorType;
}

export interface BoardSpec extends BoardSpecMsg {
  cols: number;
  rows: number;
  cell_size_m: number;
  board_thickness_m: number;
}
