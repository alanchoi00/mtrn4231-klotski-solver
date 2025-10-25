import { CellMsg } from "./cell.msg";

export enum PieceType {
  TYPE_UNKNOWN = 0,
  TYPE_2_2 = 1,
  TYPE_1_2 = 2,
  TYPE_2_1 = 3,
  TYPE_1_1 = 4,
}

export enum PieceColorType {
  COLOR_NONE = 0,
  COLOR_RED = 1,
  COLOR_GREEN = 2,
  COLOR_BLUE = 3,
  COLOR_YELLOW = 4,
}

export interface PieceMsg {
  id: string;
  cells: CellMsg[];
  color: PieceColorType;
  type: PieceType;
}
