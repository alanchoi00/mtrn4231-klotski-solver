"use client";
import React, { useMemo, useState } from "react";
import {
  DndContext,
  DragEndEvent,
  DragOverlay,
  DragStartEvent,
  PointerSensor,
  useDraggable,
  useSensor,
  useSensors,
} from "@dnd-kit/core";
import { restrictToParentElement } from "@dnd-kit/modifiers";
import {
  Card,
  CardHeader,
  CardTitle,
  CardContent,
  CardDescription,
} from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { useROS } from "./ROSProvider";
import type {
  BoardMsg,
  BoardSpecMsg,
  PieceMsg,
  CellMsg,
} from "@/lib/ros/types";

/** ====== CONFIG ====== */
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
  { id: "red", color: "red", col: 1, row: 3, w: 2, h: 2 },
  { id: "green", color: "green", col: 1, row: 2, w: 2, h: 1 },
  { id: "b1", color: "blue", col: 0, row: 2, w: 1, h: 2 },
  { id: "b2", color: "blue", col: 3, row: 2, w: 1, h: 2 },
  { id: "b3", color: "blue", col: 0, row: 0, w: 1, h: 2 },
  { id: "b4", color: "blue", col: 3, row: 0, w: 1, h: 2 },
  { id: "y1", color: "yellow", col: 0, row: 4, w: 1, h: 1 },
  { id: "y2", color: "yellow", col: 2, row: 1, w: 1, h: 1 },
  { id: "y3", color: "yellow", col: 1, row: 1, w: 1, h: 1 },
  { id: "y4", color: "yellow", col: 3, row: 4, w: 1, h: 1 },
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
      className="flex items-center justify-center text-white font-semibold"
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
            Drag pieces to define the desired 4×5 layout (color ⇒ shape).
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
            className="relative mx-auto aspect-4/5 w-full max-w-md rounded-md border bg-white"
          >
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

            {blocks.map((block) => (
              <DraggableBlock key={block.id} block={block} />
            ))}
          </div>

          <DragOverlay>
            {activeId ? (
              <div
                style={{
                  background:
                    COLORS[
                      blocks.find((b) => b.id === activeId)?.color || "red"
                    ],
                  borderRadius: 10,
                  boxShadow: "0 4px 16px rgba(0,0,0,0.4)",
                  border: "1px solid rgba(0,0,0,0.15)",
                  width: `${
                    ((blocks.find((b) => b.id === activeId)?.w || 1) /
                      SPEC.cols) *
                    100
                  }%`,
                  height: `${
                    ((blocks.find((b) => b.id === activeId)?.h || 1) /
                      SPEC.rows) *
                    100
                  }%`,
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "center",
                  color: "white",
                  fontWeight: "600",
                }}
              >
                {activeId}
              </div>
            ) : null}
          </DragOverlay>
        </DndContext>

        <div className="text-xs text-muted-foreground">
          <span
            className="inline-block rounded px-2 py-1 mr-2"
            style={{ background: COLORS.red, color: "white" }}
          >
            red = 2×2
          </span>
          <span
            className="inline-block rounded px-2 py-1 mr-2"
            style={{ background: COLORS.green, color: "white" }}
          >
            green = 2×1 (h)
          </span>
          <span
            className="inline-block rounded px-2 py-1 mr-2"
            style={{ background: COLORS.blue, color: "white" }}
          >
            blue = 1×2 (v)
          </span>
          <span
            className="inline-block rounded px-2 py-1 mr-2"
            style={{ background: COLORS.yellow, color: "black" }}
          >
            yellow = 1×1
          </span>
          <span
            className="inline-block rounded px-2 py-1"
            style={{ background: "#9ca3af", color: "black" }}
          >
            gray = exit / gap (not a piece)
          </span>
        </div>
      </CardContent>
    </Card>
  );
};
