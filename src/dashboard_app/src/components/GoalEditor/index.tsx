"use client";

import { useROS } from "@/components/ROSProvider";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import type { BoardMsg } from "@/lib/ros/types";
import type { GoalEditorProps } from "@/types/goal-editor";
import { blockToPieceMsg, blocksToPattern } from "@/utils/pattern-conversion";
import { useState, type FC } from "react";
import { BOARD_SPEC, DEFAULT_BLOCKS } from "./constants";
import Grid from "./Grid";
import {
  useBlockMovement,
  usePatternValidation,
  useRandomPattern,
} from "./hooks";
import Legend from "./Legend";

export const GoalEditor: FC<GoalEditorProps> = ({ className }) => {
  const { connected, sendGoalBoard } = useROS();
  const [blocks, setBlocks] = useState(DEFAULT_BLOCKS);

  // Custom hooks
  const { allowedPatterns, isValid, setIsValid, validatePattern } =
    usePatternValidation();
  const { handleBlockMove } = useBlockMovement(
    blocks,
    setBlocks,
    validatePattern,
    setIsValid
  );
  const { loadRandomPattern } = useRandomPattern(
    setBlocks,
    allowedPatterns,
    setIsValid
  );

  // Event handlers
  const handlePublishGoal = () => {
    const pieces = blocks.map(blockToPieceMsg);
    const goal: BoardMsg = { spec: BOARD_SPEC, pieces };
    sendGoalBoard(goal);
  };

  const handleReset = () => {
    setBlocks(DEFAULT_BLOCKS);
    setIsValid(
      allowedPatterns.has(blocksToPattern(DEFAULT_BLOCKS, BOARD_SPEC))
    );
  };

  return (
    <Card
      className={`
        w-full max-w-3xl shadow-xl
        ${className || ""}
      `}
    >
      <CardHeader className="flex items-center justify-between">
        <div>
          <CardTitle>Goal Pattern Editor</CardTitle>
          <CardDescription>
            Drag pieces to define the desired 4×5 layout.
          </CardDescription>
        </div>

        <div className="flex items-center gap-2">
          <Badge variant={connected ? "default" : "secondary"}>
            {connected ? "ROS Connected" : "ROS Offline"}
          </Badge>
          <Badge variant={isValid ? "default" : "destructive"}>
            {isValid ? "Valid pattern" : "Not in library"}
          </Badge>
        </div>
      </CardHeader>

      <CardContent className="space-y-3">
        {/* Action buttons */}
        <div className="flex gap-2">
          <Button onClick={handlePublishGoal}>
            Set Goal (publish /ui/goal)
          </Button>
          <Button variant="outline" onClick={handleReset}>
            Reset
          </Button>
          <Button variant="outline" onClick={loadRandomPattern}>
            Random pattern
          </Button>
        </div>

        {/* Interactive grid */}
        <Grid
          blocks={blocks}
          boardSpec={BOARD_SPEC}
          onBlockMove={handleBlockMove}
        />

        {/* Legend */}
        <Legend />
      </CardContent>
    </Card>
  );
};

export default GoalEditor;
