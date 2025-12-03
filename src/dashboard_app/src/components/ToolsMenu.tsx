"use client";

import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Crosshair, Grid3X3, Hand, Menu, Palette } from "lucide-react";
import { useState } from "react";
import { ArucoOffsetsSheet } from "./ArucoOffsets/ArucoOffsetsSheet";
import { ColourMaskerSheet } from "./ColourMasker/ColourMaskerSheet";
import { GoalEditorSheet } from "./GoalEditor/GoalEditorSheet";
import { HandDetectionViewer } from "./HandDetectionViewer";

export const ToolsMenu: React.FC = () => {
  const [isColourMaskerOpen, setIsColourMaskerOpen] = useState(false);
  const [isGoalEditorOpen, setIsGoalEditorOpen] = useState(false);
  const [isHandDetectionOpen, setIsHandDetectionOpen] = useState(false);
  const [isArucoOffsetsOpen, setIsArucoOffsetsOpen] = useState(false);

  return (
    <>
      <DropdownMenu>
        <DropdownMenuTrigger asChild>
          <Button
            className="fixed right-6 bottom-6 z-50 h-14 w-14 rounded-full shadow-lg"
            size="icon"
          >
            <Menu className="h-6 w-6" />
          </Button>
        </DropdownMenuTrigger>
        <DropdownMenuContent align="end" side="top" className="mb-2">
          <DropdownMenuItem onClick={() => setIsGoalEditorOpen(true)}>
            <Grid3X3 className="mr-2 h-4 w-4" />
            Goal Editor
          </DropdownMenuItem>
          <DropdownMenuItem onClick={() => setIsColourMaskerOpen(true)}>
            <Palette className="mr-2 h-4 w-4" />
            Colour Masker
          </DropdownMenuItem>
          <DropdownMenuItem onClick={() => setIsHandDetectionOpen(true)}>
            <Hand className="mr-2 h-4 w-4" />
            Hand Detection
          </DropdownMenuItem>
          <DropdownMenuItem onClick={() => setIsArucoOffsetsOpen(true)}>
            <Crosshair className="mr-2 h-4 w-4" />
            ArUco Offsets
          </DropdownMenuItem>
        </DropdownMenuContent>
      </DropdownMenu>

      <GoalEditorSheet
        open={isGoalEditorOpen}
        onOpenChange={setIsGoalEditorOpen}
      />

      <ColourMaskerSheet
        open={isColourMaskerOpen}
        onOpenChange={setIsColourMaskerOpen}
      />

      <HandDetectionViewer
        open={isHandDetectionOpen}
        onClose={() => setIsHandDetectionOpen(false)}
      />

      <ArucoOffsetsSheet
        open={isArucoOffsetsOpen}
        onOpenChange={setIsArucoOffsetsOpen}
      />
    </>
  );
};
