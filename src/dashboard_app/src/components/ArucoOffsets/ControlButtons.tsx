"use client";

import { Button } from "@/components/ui/button";
import { RotateCcw, Save } from "lucide-react";

interface ControlButtonsProps {
  connected: boolean;
  hasLocalOffsets: boolean;
  isSaving: boolean;
  isResetting: boolean;
  onApply: () => void;
  onReset: () => void;
}

export const ControlButtons: React.FC<ControlButtonsProps> = ({
  connected,
  hasLocalOffsets,
  isSaving,
  isResetting,
  onApply,
  onReset,
}) => {
  return (
    <div className="flex gap-2">
      <Button
        onClick={onApply}
        disabled={!connected || !hasLocalOffsets || isSaving}
        className="flex-1"
      >
        <Save className="mr-2 h-4 w-4" />
        {isSaving ? "Applying..." : "Apply"}
      </Button>
      <Button
        variant="outline"
        onClick={onReset}
        disabled={!connected || isResetting}
      >
        <RotateCcw className="mr-2 h-4 w-4" />
        {isResetting ? "Resetting..." : "Reset to Zero"}
      </Button>
    </div>
  );
};
