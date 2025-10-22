"use client";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import type {
  BoardMsg,
  BoardSpecMsg,
  CellMsg,
  PieceMsg,
} from "@/lib/ros/types";
import {
  DndContext,
  DragEndEvent,
  DragStartEvent,
  PointerSensor,
  useDraggable,
  useSensor,
  useSensors,
} from "@dnd-kit/core";
import { restrictToParentElement } from "@dnd-kit/modifiers";
import React, { useMemo, useState } from "react";
import { useROS } from "./ROSProvider";

const SPEC: BoardSpecMsg = {
  cols: 4,
  rows: 5,
  cell_size_m: 0.03,
  board_thickness_m: 0.02,
};

type Color = "red" | "green" | "blue" | "yellow";

const COLORS: Record<Color, string> = {
  red: "#dc2626",
  green: "#16a34a",
  blue: "#2563eb",
  yellow: "#ca8a04",
};

type Block = {
  id: string;
  color: Color;
  col: number; // bottom-left cell col
  row: number; // bottom-left cell row
  w: number;
  h: number;
};

const START: Block[] = [
  // These positions are examples: tune them to match your preferred initial goal
  { id: "a", color: "blue", col: 0, row: 3, w: 1, h: 2 },
  { id: "b", color: "red", col: 1, row: 3, w: 2, h: 2 },
  { id: "c", color: "blue", col: 3, row: 3, w: 1, h: 2 },
  { id: "d", color: "green", col: 1, row: 2, w: 2, h: 1 },
  { id: "e", color: "blue", col: 0, row: 0, w: 1, h: 2 },
  { id: "f", color: "yellow", col: 1, row: 1, w: 1, h: 1 },
  { id: "g", color: "yellow", col: 2, row: 1, w: 1, h: 1 },
  { id: "h", color: "blue", col: 3, row: 0, w: 1, h: 2 },
  { id: "i", color: "yellow", col: 1, row: 0, w: 1, h: 1 },
  { id: "j", color: "yellow", col: 2, row: 0, w: 1, h: 1 },
];

function inBounds(b: Block, spec = SPEC): boolean {
  return (
    b.col >= 0 &&
    b.row >= 0 &&
    b.col + b.w <= spec.cols &&
    b.row + b.h <= spec.rows
  );
}
function overlaps(a: Block, b: Block): boolean {
  return !(
    a.col + a.w <= b.col ||
    b.col + b.w <= a.col ||
    a.row + a.h <= b.row ||
    b.row + b.h <= a.row
  );
}
function collideAny(p: Block, all: Block[]): boolean {
  return all.some((q) => q.id !== p.id && overlaps(p, q));
}

function blockToPieceMsg(b: Block): PieceMsg {
  const cells: CellMsg[] = [];
  for (let dx = 0; dx < b.w; dx++) {
    for (let dy = 0; dy < b.h; dy++) {
      cells.push({ col: b.col + dx, row: b.row + dy });
    }
  }
  return { id: b.id, color: b.color, cells };
}

const DraggableBlock: React.FC<{ block: Block }> = ({ block }) => {
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: block.id,
    });

  const style: React.CSSProperties = {
    position: "absolute",
    left: `${(block.col / SPEC.cols) * 100}%`,
    bottom: `${(block.row / SPEC.rows) * 100}%`,
    width: `${(block.w / SPEC.cols) * 100}%`,
    height: `${(block.h / SPEC.rows) * 100}%`,
    background: COLORS[block.color],
    borderRadius: 10,
    boxShadow: isDragging
      ? "0 4px 16px rgba(0,0,0,0.4)"
      : "0 2px 8px rgba(0,0,0,0.22)",
    border: "1px solid rgba(0,0,0,0.15)",
    touchAction: "none",
    cursor: isDragging ? "grabbing" : "grab",
    userSelect: "none",
    transform: transform
      ? `translate3d(${transform.x}px, ${transform.y}px, 0)`
      : undefined,
    zIndex: isDragging ? 1000 : 1,
    opacity: isDragging ? 0.8 : 1,
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      {...listeners}
      {...attributes}
      className="flex items-center justify-center font-semibold text-white"
    >
      {block.id}
    </div>
  );
};

export const GoalEditor: React.FC = () => {
  const { connected, sendGoalBoard } = useROS();
  const [blocks, setBlocks] = useState<Block[]>(START);
  const [activeId, setActiveId] = useState<string | null>(null);

  const sensors = useSensors(
    useSensor(PointerSensor, { activationConstraint: { distance: 4 } })
  );

  const publishGoal = () => {
    const pieces = blocks.map(blockToPieceMsg);
    const goal: BoardMsg = { spec: SPEC, pieces };
    sendGoalBoard(goal);
  };

  const gridCells = useMemo(
    () => Array.from({ length: SPEC.cols * SPEC.rows }, (_, i) => i),
    []
  );

  const onDragStart = (e: DragStartEvent) => {
    setActiveId(String(e.active.id));
  };

  const onDragEnd = (e: DragEndEvent) => {
    setActiveId(null);
    const id = String(e.active.id);
    const container = document.getElementById("klotski-grid");
    if (!container) return;

    const rect = container.getBoundingClientRect();
    const cellW = rect.width / SPEC.cols;
    const cellH = rect.height / SPEC.rows;

    const m = blocks.find((b) => b.id === id);
    if (!m || !e.delta) return;

    const dxCells = Math.round(e.delta.x / cellW);
    const dyCells = Math.round(-e.delta.y / cellH); // flip Y

    const moved: Block = { ...m, col: m.col + dxCells, row: m.row + dyCells };
    if (!inBounds(moved)) return;
    if (collideAny(moved, blocks)) return;

    setBlocks((prev) => prev.map((b) => (b.id === id ? moved : b)));
  };

  return (
    <Card className="w-full max-w-3xl shadow-xl">
      <CardHeader className="flex items-center justify-between">
        <div>
          <CardTitle>Goal Pattern Editor</CardTitle>
          <CardDescription>
            Drag pieces to define the desired 4×5 layout.
          </CardDescription>
        </div>
        <Badge variant={connected ? "default" : "secondary"}>
          {connected ? "ROS Connected" : "ROS Offline"}
        </Badge>
      </CardHeader>

      <CardContent className="space-y-3">
        <div className="flex gap-2">
          <Button onClick={publishGoal}>Set Goal (publish /ui/goal)</Button>
          <Button variant="outline" onClick={() => setBlocks(START)}>
            Reset
          </Button>
        </div>

        <DndContext
          sensors={sensors}
          onDragStart={onDragStart}
          onDragEnd={onDragEnd}
          modifiers={[restrictToParentElement]}
        >
          <div
            id="klotski-grid"
            className={`
              relative mx-auto aspect-4/5 w-full max-w-md rounded-md border
              bg-white
            `}
          >
            {/* Grid lines */}
            <div
              className="absolute inset-0 grid"
              style={{
                gridTemplateColumns: `repeat(${SPEC.cols}, 1fr)`,
                gridTemplateRows: `repeat(${SPEC.rows}, 1fr)`,
              }}
            >
              {gridCells.map((i) => (
                <div key={i} className="border border-black/10" />
              ))}
            </div>
            <div
              className={`
                pointer-events-none absolute right-0 bottom-0 left-0 grid
                text-[10px] text-muted-foreground
                sm:text-xs
              `}
              style={{
                gridTemplateColumns: `repeat(${SPEC.cols}, 1fr)`,
                transform: "translateY(100%)", // sit just below grid
              }}
            >
              {Array.from({ length: SPEC.cols }, (_, c) => (
                <div key={c} className="py-0.5 text-center">
                  {c}
                </div>
              ))}
            </div>

            <div
              className={`
                pointer-events-none absolute top-0 bottom-0 left-0 grid
                text-[10px] text-muted-foreground
                sm:text-xs
              `}
              style={{
                gridTemplateRows: `repeat(${SPEC.rows}, 1fr)`,
                transform: "translateX(-100%)", // sit just left of grid
              }}
            >
              {Array.from({ length: SPEC.rows }, (_, rFromTop) => {
                const r = SPEC.rows - 1 - rFromTop; // flip so 0 is bottom
                return (
                  <div key={r} className="flex items-center justify-end pr-1">
                    {r}
                  </div>
                );
              })}
            </div>

            <div
              className={`
                pointer-events-none absolute -bottom-8 left-0 text-xs
                font-medium text-muted-foreground
                sm:text-sm
              `}
            >
              col →
            </div>

            <div
              className={`
                pointer-events-none absolute bottom-8 -left-6 origin-top-left
                translate-x-1 rotate-90 text-xs font-medium
                text-muted-foreground
                sm:text-sm
              `}
            >
              ← row
            </div>

            {/* Draggable blocks */}
            {blocks.map((block) => (
              <DraggableBlock key={block.id} block={block} />
            ))}
          </div>
        </DndContext>
      </CardContent>
    </Card>
  );
};
