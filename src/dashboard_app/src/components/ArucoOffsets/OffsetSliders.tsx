"use client";

import { Slider } from "@/components/ui/slider";
import type { ArucoOffsetsMsg } from "@/lib/ros/types";

interface OffsetSliderProps {
  label: string;
  axis: "X" | "Y" | "Z";
  value: number;
  onChange: (value: number) => void;
  min?: number;
  max?: number;
  step?: number;
}

const AXIS_COLORS = {
  X: "bg-red-500",
  Y: "bg-green-500",
  Z: "bg-blue-500",
} as const;

export const OffsetSlider: React.FC<OffsetSliderProps> = ({
  label,
  axis,
  value,
  onChange,
  min = -0.1,
  max = 0.1,
  step = 0.001,
}) => {
  // Convert slider value (0-1000) to offset value (-0.1 to 0.1)
  const sliderToOffset = (sliderVal: number) => {
    return min + (sliderVal / 1000) * (max - min);
  };

  // Convert offset value to slider value (0-1000)
  const offsetToSlider = (offsetVal: number) => {
    return Math.round(((offsetVal - min) / (max - min)) * 1000);
  };

  return (
    <div className="space-y-3 rounded-lg border p-4">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <div className={`h-4 w-4 rounded ${AXIS_COLORS[axis]}`} />
          <h4 className="font-semibold">{label}</h4>
        </div>
        <span className="font-mono text-sm">
          {(value * 1000).toFixed(1)} mm
        </span>
      </div>

      <Slider
        value={[offsetToSlider(value)]}
        min={0}
        max={1000}
        step={1}
        onValueChange={([v]) => onChange(sliderToOffset(v))}
      />

      <div className="flex justify-between text-xs text-muted-foreground">
        <span>{(min * 1000).toFixed(0)} mm</span>
        <span>0 mm</span>
        <span>{(max * 1000).toFixed(0)} mm</span>
      </div>
    </div>
  );
};

interface OffsetSlidersGroupProps {
  offsets: ArucoOffsetsMsg;
  onChange: (offsets: ArucoOffsetsMsg) => void;
}

export const OffsetSlidersGroup: React.FC<OffsetSlidersGroupProps> = ({
  offsets,
  onChange,
}) => {
  return (
    <div className="space-y-4">
      <OffsetSlider
        label="X Offset"
        axis="X"
        value={offsets.x}
        onChange={(x) => onChange({ ...offsets, x })}
      />
      <OffsetSlider
        label="Y Offset"
        axis="Y"
        value={offsets.y}
        onChange={(y) => onChange({ ...offsets, y })}
      />
      <OffsetSlider
        label="Z Offset"
        axis="Z"
        value={offsets.z}
        onChange={(z) => onChange({ ...offsets, z })}
      />
    </div>
  );
};
