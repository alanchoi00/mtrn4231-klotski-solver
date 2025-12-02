// Color configuration for HSV slider display
export const COLOR_CONFIG: Record<
  string,
  { label: string; bgColor: string; textColor: string }
> = {
  red1: {
    label: "Red (Low Hue)",
    bgColor: "bg-red-500",
    textColor: "text-white",
  },
  red2: {
    label: "Red (High Hue)",
    bgColor: "bg-red-700",
    textColor: "text-white",
  },
  yellow: {
    label: "Yellow",
    bgColor: "bg-yellow-400",
    textColor: "text-black",
  },
  green: {
    label: "Green",
    bgColor: "bg-green-500",
    textColor: "text-white",
  },
  blue: {
    label: "Blue",
    bgColor: "bg-blue-500",
    textColor: "text-white",
  },
};
