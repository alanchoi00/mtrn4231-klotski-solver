"use client";

import {
  blockToPieceMsg,
  blocksToPattern,
} from "@/components/GoalEditor/utils/pattern-conversion";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import {
  Sheet,
  SheetContent,
  SheetDescription,
  SheetHeader,
  SheetTitle,
} from "@/components/ui/sheet";
import { useROS, type BoardMsg } from "@/lib/ros";
import { Grid3X3 } from "lucide-react";
import { useState, type FC } from "react";
import { BOARD_SPEC, DEFAULT_BLOCKS } from "./constants";
import Grid from "./Grid";
import {
  useBlockMovement,
  usePatternValidation,
  useRandomPattern,
} from "./hooks";
import Legend from "./Legend";

interface GoalEditorSheetProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
}

export const GoalEditorSheet: FC<GoalEditorSheetProps> = ({
  open,
  onOpenChange,
}) => {
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
    <Sheet open={open} onOpenChange={onOpenChange}>
      <SheetContent
        side="right"
        className="flex h-full w-full flex-col p-6 sm:max-w-xl"
      >
        <SheetHeader className="shrink-0">
          <SheetTitle className="flex items-center gap-2">
            <Grid3X3 className="h-5 w-5" />
            Goal Pattern Editor
          </SheetTitle>
          <SheetDescription>
            Drag pieces to define the desired 4×5 layout.
          </SheetDescription>
        </SheetHeader>

        <div className="mt-6 flex min-h-0 flex-1 flex-col gap-4">
          {/* Status badges */}
          <div className="flex shrink-0 items-center gap-2">
            <Badge variant={connected ? "default" : "secondary"}>
              {connected ? "ROS Connected" : "ROS Offline"}
            </Badge>
            <Badge variant={isValid ? "default" : "destructive"}>
              {isValid ? "Valid pattern" : "Not in library"}
            </Badge>
          </div>

          {/* Action buttons */}
          <div className="flex shrink-0 flex-wrap gap-2">
            <Button onClick={handlePublishGoal} disabled={!connected}>
              Set Goal
            </Button>
            <Button variant="outline" onClick={handleReset}>
              Reset
            </Button>
            <Button variant="outline" onClick={loadRandomPattern}>
              Random
            </Button>
          </div>

          {/* Interactive grid */}
          <div className="min-h-0 flex-1 overflow-y-auto">
            <div className="space-y-4 pb-6">
              <Grid
                blocks={blocks}
                boardSpec={BOARD_SPEC}
                onBlockMove={handleBlockMove}
              />

              {/* Legend */}
              <Legend />
            </div>
          </div>
        </div>
      </SheetContent>
    </Sheet>
  );
};
