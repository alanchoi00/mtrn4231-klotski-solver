export type UIMode = "auto" | "step" | "pause" | "reset";

export interface UICommandMsg {
  mode: UIMode;
  replan: boolean;
}

export interface RosStringMsg {
  data: string;
}

export interface BoardSpecMsg {
  cols: number;
  rows: number;
  cell_size_m: number;
  board_thickness_m: number;
}

export interface CellMsg {
  col: number;
  row: number;
}

export interface PieceMsg {
  id: string;
  cells: CellMsg[];
  color: string;
}

export interface BoardMsg {
  spec: BoardSpecMsg;
  pieces: PieceMsg[];
}
