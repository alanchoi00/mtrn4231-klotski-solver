"use client";

import {
  Sheet,
  SheetContent,
  SheetDescription,
  SheetHeader,
  SheetTitle,
} from "@/components/ui/sheet";
import { useHSVConfig } from "@/lib/ros/hooks/useHSVConfig";
import { useROS } from "@/lib/ros";
import type { HSVRangeMsg, HSVRangesMsg } from "@/lib/ros/types";
import { Palette } from "lucide-react";
import { useCallback, useEffect, useState } from "react";
import { toast } from "sonner";
import { ControlButtons } from "./ControlButtons";
import { HSVSliderGroup } from "./HSVSliderGroup";
import { MaskedImagePreview } from "./MaskedImagePreview";
import { YamlExportDialog } from "./YamlExportDialog";

interface ColourMaskerSheetProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
}

export const ColourMaskerSheet: React.FC<ColourMaskerSheetProps> = ({
  open,
  onOpenChange,
}) => {
  const { ros, connected } = useROS();
  const {
    hsvRanges,
    isSubscribed,
    isLoading,
    setHSVRanges,
    resetHSVRanges,
    exportHSVRangesYaml,
  } = useHSVConfig({ ros });

  const [localRanges, setLocalRanges] = useState<HSVRangesMsg | null>(null);
  const [yamlContent, setYamlContent] = useState<string>("");
  const [isYamlDialogOpen, setIsYamlDialogOpen] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [isResetting, setIsResetting] = useState(false);
  const [isExporting, setIsExporting] = useState(false);
  const [imageError, setImageError] = useState(false);

  // Sync local state when hsvRanges changes
  useEffect(() => {
    if (hsvRanges) {
      setLocalRanges(hsvRanges);
    }
  }, [hsvRanges]);

  // Reset image error state when sheet opens
  useEffect(() => {
    if (open) {
      setImageError(false);
    }
  }, [open]);

  const handleRangeChange = useCallback((updated: HSVRangeMsg) => {
    setLocalRanges((prev) => {
      if (!prev) return prev;
      return {
        ranges: prev.ranges.map((r) => (r.name === updated.name ? updated : r)),
      };
    });
  }, []);

  const handleApply = useCallback(async () => {
    if (!localRanges) return;

    setIsSaving(true);
    try {
      const response = await setHSVRanges(localRanges);
      if (response.ok) {
        toast.success("HSV ranges updated", {
          description: response.message,
        });
      } else {
        toast.error("Failed to update HSV ranges", {
          description: response.message,
        });
      }
    } catch (error) {
      toast.error("Failed to update HSV ranges", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
    } finally {
      setIsSaving(false);
    }
  }, [localRanges, setHSVRanges]);

  const handleReset = useCallback(async () => {
    setIsResetting(true);
    try {
      const response = await resetHSVRanges();
      if (response.ok) {
        setLocalRanges(response.ranges);
        toast.success("HSV ranges reset", {
          description: "Reset to YAML config values",
        });
      } else {
        toast.error("Failed to reset HSV ranges", {
          description: response.message,
        });
      }
    } catch (error) {
      toast.error("Failed to reset HSV ranges", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
    } finally {
      setIsResetting(false);
    }
  }, [resetHSVRanges]);

  const handleExport = useCallback(async () => {
    setIsExporting(true);
    try {
      const response = await exportHSVRangesYaml();
      if (response.ok) {
        setYamlContent(response.yaml_content);
        setIsYamlDialogOpen(true);
        toast.success("YAML exported");
      } else {
        toast.error("Failed to export YAML", {
          description: response.message,
        });
      }
    } catch (error) {
      toast.error("Failed to export YAML", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
    } finally {
      setIsExporting(false);
    }
  }, [exportHSVRangesYaml]);

  const maskedImageUrl = connected
    ? `http://${window.location.hostname}:8080/stream?topic=/sense/masked_image`
    : "";

  return (
    <>
      <Sheet open={open} onOpenChange={onOpenChange}>
        <SheetContent
          side="right"
          className="flex h-full w-full flex-col p-6 sm:max-w-xl"
        >
          <SheetHeader className="shrink-0">
            <SheetTitle className="flex items-center gap-2">
              <Palette className="h-5 w-5" />
              Colour Masker
            </SheetTitle>
            <SheetDescription>
              Adjust HSV color ranges for board detection.
              {!connected && " (Disconnected from ROS)"}
            </SheetDescription>
          </SheetHeader>

          <div className="mt-6 flex min-h-0 flex-1 flex-col gap-4">
            <MaskedImagePreview
              connected={connected}
              imageError={imageError}
              maskedImageUrl={maskedImageUrl}
              onImageError={() => setImageError(true)}
            />

            <ControlButtons
              connected={connected}
              hasLocalRanges={!!localRanges}
              isSaving={isSaving}
              isResetting={isResetting}
              isExporting={isExporting}
              onApply={handleApply}
              onReset={handleReset}
              onExport={handleExport}
            />

            {/* HSV Sliders */}
            <div className="min-h-0 flex-1 overflow-y-auto pr-2">
              <div className="space-y-4 pb-6">
                {localRanges?.ranges.map((range) => (
                  <HSVSliderGroup
                    key={range.name}
                    range={range}
                    onChange={handleRangeChange}
                  />
                ))}
                {!localRanges && (
                  <div className="py-8 text-center text-muted-foreground">
                    {!connected
                      ? "Connect to ROS to load HSV configuration"
                      : isLoading
                      ? "Loading HSV ranges..."
                      : isSubscribed
                      ? "Waiting for HSV ranges..."
                      : "Subscribing to HSV ranges..."}
                  </div>
                )}
              </div>
            </div>
          </div>
        </SheetContent>
      </Sheet>

      <YamlExportDialog
        open={isYamlDialogOpen}
        onOpenChange={setIsYamlDialogOpen}
        yamlContent={yamlContent}
      />
    </>
  );
};
