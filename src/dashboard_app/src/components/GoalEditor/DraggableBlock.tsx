import { useDraggable } from "@dnd-kit/core";
import type { FC } from "react";
import type { DraggableBlockProps } from "../../types/goal-editor";
import { COLORS } from "./constants";

const DraggableBlock: FC<DraggableBlockProps> = ({ block, boardSpec }) => {
  const { attributes, listeners, setNodeRef, transform, isDragging } =
    useDraggable({
      id: block.uid,
    });

  const style: React.CSSProperties = {
    position: "absolute",
    left: `${(block.col / boardSpec.cols) * 100}%`,
    bottom: `${(block.row / boardSpec.rows) * 100}%`,
    width: `${(block.w / boardSpec.cols) * 100}%`,
    height: `${(block.h / boardSpec.rows) * 100}%`,
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
    opacity: isDragging ? 0.9 : 1,
  };

  return <div ref={setNodeRef} style={style} {...listeners} {...attributes} />;
};

export default DraggableBlock;
