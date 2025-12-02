"use client";

import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { Grid3X3, Menu, Palette } from "lucide-react";
import { useState } from "react";
import { ColourMaskerSheet } from "./ColourMasker/ColourMaskerSheet";
import { GoalEditorSheet } from "./GoalEditor/GoalEditorSheet";

export const ToolsMenu: React.FC = () => {
  const [isColourMaskerOpen, setIsColourMaskerOpen] = useState(false);
  const [isGoalEditorOpen, setIsGoalEditorOpen] = useState(false);

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
    </>
  );
};
