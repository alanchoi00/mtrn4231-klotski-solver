import { PieceColorType } from "@/lib/ros";
import type { FC } from "react";
import { COLORS } from "./constants";

const Legend: FC = () => {
  return (
    <div className="mt-10 text-xs text-muted-foreground">
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{ background: COLORS[PieceColorType.COLOR_RED], color: "white" }}
      >
        red = 2×2
      </span>
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{
          background: COLORS[PieceColorType.COLOR_GREEN],
          color: "white",
        }}
      >
        green = 2×1 (h)
      </span>
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{
          background: COLORS[PieceColorType.COLOR_BLUE],
          color: "white",
        }}
      >
        blue = 1×2 (v)
      </span>
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{
          background: COLORS[PieceColorType.COLOR_YELLOW],
          color: "black",
        }}
      >
        yellow = 1×1
      </span>
    </div>
  );
};

export default Legend;
