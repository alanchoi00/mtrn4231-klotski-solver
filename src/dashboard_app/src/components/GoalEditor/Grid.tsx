import {
  DndContext,
  DragEndEvent,
  PointerSensor,
  useSensor,
  useSensors,
} from "@dnd-kit/core";
import { restrictToParentElement } from "@dnd-kit/modifiers";
import { useMemo, type FC } from "react";
import type { GridProps } from "../../types/goal-editor";
import DraggableBlock from "./DraggableBlock";

const Grid: FC<GridProps> = ({ blocks, boardSpec, onBlockMove }) => {
  const sensors = useSensors(
    useSensor(PointerSensor, { activationConstraint: { distance: 4 } })
  );

  const gridCells = useMemo(
    () => Array.from({ length: boardSpec.cols * boardSpec.rows }, (_, i) => i),
    [boardSpec.cols, boardSpec.rows]
  );

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, delta } = event;
    if (!delta) return;

    const container = document.getElementById("klotski-grid");
    if (!container) return;

    const rect = container.getBoundingClientRect();
    const cellW = rect.width / boardSpec.cols;
    const cellH = rect.height / boardSpec.rows;

    const dxCells = Math.round(delta.x / cellW);
    const dyCells = Math.round(-delta.y / cellH); // flip Y

    const block = blocks.find((b) => b.uid === String(active.id));
    if (!block) return;

    onBlockMove(block.uid, block.col + dxCells, block.row + dyCells);
  };

  return (
    <DndContext
      sensors={sensors}
      onDragEnd={handleDragEnd}
      modifiers={[restrictToParentElement]}
    >
      <div
        id="klotski-grid"
        className={`
          relative mx-auto aspect-4/5 w-full max-w-md rounded-md border bg-white
        `}
      >
        {/* Grid lines */}
        <div
          className="absolute inset-0 grid"
          style={{
            gridTemplateColumns: `repeat(${boardSpec.cols}, 1fr)`,
            gridTemplateRows: `repeat(${boardSpec.rows}, 1fr)`,
          }}
        >
          {gridCells.map((i) => (
            <div key={i} className="border border-black/10" />
          ))}
        </div>

        {/* Column labels */}
        <div
          className={`
            pointer-events-none absolute right-0 bottom-0 left-0 grid
            text-[10px] text-muted-foreground
            sm:text-xs
          `}
          style={{
            gridTemplateColumns: `repeat(${boardSpec.cols}, 1fr)`,
            transform: "translateY(100%)",
          }}
        >
          {Array.from({ length: boardSpec.cols }, (_, c) => (
            <div key={c} className="py-0.5 text-center">
              {c}
            </div>
          ))}
        </div>

        {/* Row labels */}
        <div
          className={`
            pointer-events-none absolute top-0 bottom-0 left-0 grid text-[10px]
            text-muted-foreground
            sm:text-xs
          `}
          style={{
            gridTemplateRows: `repeat(${boardSpec.rows}, 1fr)`,
            transform: "translateX(-100%)",
          }}
        >
          {Array.from({ length: boardSpec.rows }, (_, rFromTop) => {
            const r = boardSpec.rows - 1 - rFromTop;
            return (
              <div key={r} className="flex items-center justify-end pr-1">
                {r}
              </div>
            );
          })}
        </div>

        {/* Axis labels */}
        <div
          className={`
            pointer-events-none absolute -bottom-8 left-0 text-xs font-medium
            text-muted-foreground
            sm:text-sm
          `}
        >
          col →
        </div>
        <div
          className={`
            pointer-events-none absolute bottom-8 -left-6 origin-top-left
            translate-x-1 rotate-90 text-xs font-medium text-muted-foreground
            sm:text-sm
          `}
        >
          ← row
        </div>

        {/* Draggable blocks */}
        {blocks.map((block) => (
          <DraggableBlock key={block.uid} block={block} boardSpec={boardSpec} />
        ))}
      </div>
    </DndContext>
  );
};

export default Grid;
