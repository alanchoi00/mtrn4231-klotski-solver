"use client";

import {
  Sheet,
  SheetContent,
  SheetDescription,
  SheetHeader,
  SheetTitle,
} from "@/components/ui/sheet";
import { useArucoOffsets } from "@/lib/ros/hooks/useArucoOffsets";
import { useROS } from "@/lib/ros";
import type { ArucoOffsetsMsg } from "@/lib/ros/types";
import { Crosshair } from "lucide-react";
import { useCallback, useEffect, useState } from "react";
import { toast } from "sonner";
import { ControlButtons } from "./ControlButtons";
import { OffsetSlidersGroup } from "./OffsetSliders";

interface ArucoOffsetsSheetProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
}

export const ArucoOffsetsSheet: React.FC<ArucoOffsetsSheetProps> = ({
  open,
  onOpenChange,
}) => {
  const { ros, connected } = useROS();
  const { offsets, isSubscribed, isLoading, setArucoOffsets } = useArucoOffsets(
    { ros }
  );

  const [localOffsets, setLocalOffsets] = useState<ArucoOffsetsMsg | null>(
    null
  );
  const [isSaving, setIsSaving] = useState(false);
  const [isResetting, setIsResetting] = useState(false);

  // Sync local state when offsets changes
  useEffect(() => {
    if (offsets) {
      setLocalOffsets(offsets);
    }
  }, [offsets]);

  const handleApply = useCallback(async () => {
    if (!localOffsets) return;

    setIsSaving(true);
    try {
      const response = await setArucoOffsets(localOffsets);
      if (response.ok) {
        toast.success("ArUco offsets updated", {
          description: response.message,
        });
      } else {
        toast.error("Failed to update ArUco offsets", {
          description: response.message,
        });
      }
    } catch (error) {
      toast.error("Failed to update ArUco offsets", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
    } finally {
      setIsSaving(false);
    }
  }, [localOffsets, setArucoOffsets]);

  const handleReset = useCallback(async () => {
    setIsResetting(true);
    try {
      const zeroOffsets: ArucoOffsetsMsg = { x: 0, y: 0, z: 0 };
      const response = await setArucoOffsets(zeroOffsets);
      if (response.ok) {
        setLocalOffsets(zeroOffsets);
        toast.success("ArUco offsets reset", {
          description: "Offsets reset to zero",
        });
      } else {
        toast.error("Failed to reset ArUco offsets", {
          description: response.message,
        });
      }
    } catch (error) {
      toast.error("Failed to reset ArUco offsets", {
        description: error instanceof Error ? error.message : "Unknown error",
      });
    } finally {
      setIsResetting(false);
    }
  }, [setArucoOffsets]);

  return (
    <Sheet open={open} onOpenChange={onOpenChange}>
      <SheetContent
        side="right"
        className="flex h-full w-full flex-col p-6 sm:max-w-md"
      >
        <SheetHeader className="shrink-0">
          <SheetTitle className="flex items-center gap-2">
            <Crosshair className="h-5 w-5" />
            ArUco Offsets
          </SheetTitle>
          <SheetDescription>
            Adjust ArUco marker position offsets in the base frame.
            {!connected && " (Disconnected from ROS)"}
          </SheetDescription>
        </SheetHeader>

        <div className="mt-6 flex min-h-0 flex-1 flex-col gap-4">
          <ControlButtons
            connected={connected}
            hasLocalOffsets={!!localOffsets}
            isSaving={isSaving}
            isResetting={isResetting}
            onApply={handleApply}
            onReset={handleReset}
          />

          {/* Offset Sliders */}
          <div className="min-h-0 flex-1 overflow-y-auto pr-2">
            <div className="space-y-4 pb-6">
              {localOffsets ? (
                <OffsetSlidersGroup
                  offsets={localOffsets}
                  onChange={setLocalOffsets}
                />
              ) : (
                <div className="py-8 text-center text-muted-foreground">
                  {!connected
                    ? "Connect to ROS to load ArUco offsets"
                    : isLoading
                    ? "Loading ArUco offsets..."
                    : isSubscribed
                    ? "Waiting for ArUco offsets..."
                    : "Subscribing to ArUco offsets..."}
                </div>
              )}
            </div>
          </div>

          {/* Info section */}
          <div className="shrink-0 rounded-lg bg-muted p-3 text-sm text-muted-foreground">
            <p>
              <strong>Tip:</strong> These offsets are applied to the ArUco
              marker position in the base_link frame. Use positive X to move the
              marker forward, positive Y to move left, and positive Z to move
              up.
            </p>
          </div>
        </div>
      </SheetContent>
    </Sheet>
  );
};
