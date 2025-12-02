"use client";

import { Slider } from "@/components/ui/slider";
import type { HSVRangeMsg } from "@/lib/ros/types";
import { COLOR_CONFIG } from "./constants";
import { cn } from "@/lib/utils";

interface HSVSliderGroupProps {
  range: HSVRangeMsg;
  onChange: (updated: HSVRangeMsg) => void;
}

export const HSVSliderGroup: React.FC<HSVSliderGroupProps> = ({
  range,
  onChange,
}) => {
  const config = COLOR_CONFIG[range.name] || {
    label: range.name,
    bgColor: "bg-gray-500",
    textColor: "text-white",
  };

  const handleChange = (field: keyof HSVRangeMsg, value: number) => {
    onChange({ ...range, [field]: value });
  };

  return (
    <div className="space-y-3 rounded-lg border p-4">
      <div className="flex items-center gap-2">
        <div className={cn("h-4 w-4 rounded", config.bgColor)} />
        <h4 className="font-semibold">{config.label}</h4>
      </div>

      <div className="grid grid-cols-2 gap-4">
        {/* Hue */}
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">H Min</span>
            <span className="font-mono">{range.h_min}</span>
          </div>
          <Slider
            value={[range.h_min]}
            min={0}
            max={180}
            step={1}
            onValueChange={([v]) => handleChange("h_min", v)}
          />
        </div>
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">H Max</span>
            <span className="font-mono">{range.h_max}</span>
          </div>
          <Slider
            value={[range.h_max]}
            min={0}
            max={180}
            step={1}
            onValueChange={([v]) => handleChange("h_max", v)}
          />
        </div>

        {/* Saturation */}
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">S Min</span>
            <span className="font-mono">{range.s_min}</span>
          </div>
          <Slider
            value={[range.s_min]}
            min={0}
            max={255}
            step={1}
            onValueChange={([v]) => handleChange("s_min", v)}
          />
        </div>
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">S Max</span>
            <span className="font-mono">{range.s_max}</span>
          </div>
          <Slider
            value={[range.s_max]}
            min={0}
            max={255}
            step={1}
            onValueChange={([v]) => handleChange("s_max", v)}
          />
        </div>

        {/* Value */}
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">V Min</span>
            <span className="font-mono">{range.v_min}</span>
          </div>
          <Slider
            value={[range.v_min]}
            min={0}
            max={255}
            step={1}
            onValueChange={([v]) => handleChange("v_min", v)}
          />
        </div>
        <div className="space-y-2">
          <div className="flex justify-between text-sm">
            <span className="text-muted-foreground">V Max</span>
            <span className="font-mono">{range.v_max}</span>
          </div>
          <Slider
            value={[range.v_max]}
            min={0}
            max={255}
            step={1}
            onValueChange={([v]) => handleChange("v_max", v)}
          />
        </div>
      </div>
    </div>
  );
};
