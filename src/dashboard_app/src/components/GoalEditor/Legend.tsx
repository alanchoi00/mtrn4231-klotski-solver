import type { FC } from "react";
import { COLOR } from "../../types/goal-editor";
import { COLORS } from "./constants";

const Legend: FC = () => {
  return (
    <div className="mt-10 text-xs text-muted-foreground">
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{ background: COLORS[COLOR.RED], color: "white" }}
      >
        red = 2×2
      </span>
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{ background: COLORS[COLOR.GREEN], color: "white" }}
      >
        green = 2×1 (h)
      </span>
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{ background: COLORS[COLOR.BLUE], color: "white" }}
      >
        blue = 1×2 (v)
      </span>
      <span
        className="mr-2 inline-block rounded px-2 py-1"
        style={{ background: COLORS[COLOR.YELLOW], color: "black" }}
      >
        yellow = 1×1
      </span>
    </div>
  );
};

export default Legend;
