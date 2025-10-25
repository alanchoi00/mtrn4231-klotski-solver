import { BoardSpecMsg } from "./board-spec.msg";
import { PieceMsg } from "./piece.msg";

export interface BoardMsg {
  spec: BoardSpecMsg;
  pieces: PieceMsg[];
}
