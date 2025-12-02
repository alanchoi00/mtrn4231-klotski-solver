"use client";

import { Button } from "@/components/ui/button";
import { Download, RotateCcw, Save } from "lucide-react";

interface ControlButtonsProps {
  connected: boolean;
  hasLocalRanges: boolean;
  isSaving: boolean;
  isResetting: boolean;
  isExporting: boolean;
  onApply: () => void;
  onReset: () => void;
  onExport: () => void;
}

export const ControlButtons: React.FC<ControlButtonsProps> = ({
  connected,
  hasLocalRanges,
  isSaving,
  isResetting,
  isExporting,
  onApply,
  onReset,
  onExport,
}) => {
  return (
    <div className="flex shrink-0 flex-wrap gap-2">
      <Button
        onClick={onApply}
        disabled={!connected || isSaving || !hasLocalRanges}
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
        {isResetting ? "Resetting..." : "Reset to YAML"}
      </Button>
      <Button
        variant="outline"
        onClick={onExport}
        disabled={!connected || isExporting}
      >
        <Download className="mr-2 h-4 w-4" />
        {isExporting ? "Exporting..." : "Export YAML"}
      </Button>
    </div>
  );
};
