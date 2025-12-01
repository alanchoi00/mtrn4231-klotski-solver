/**
 * ROS Topics and Services Configuration
 *
 * Centralized configuration for all ROS topics and services used in the dashboard.
 * Edit these values to match your ROS setup.
 */

// =============================================================================
// Web Video Server
// =============================================================================
export const WEB_VIDEO_SERVER_PORT = 8080;

// =============================================================================
// Image Topics
// =============================================================================
export const TOPICS = {
  // Camera/Vision
  /** Annotated image with hand detection overlay */
  HAND_DETECTION_IMAGE: "/safety/hand_detection_image",
  /** Raw camera image without annotations */
  RAW_IMAGE: "/camera/camera/colour/image_raw",
  /** Masked image for colour detection */
  MASKED_IMAGE: "/sense/masked_image",
  /** HSV ranges configuration */
  HSV_RANGES: "/sense/hsv_ranges",

  // Safety
  /** Safety stop signal (Bool) */
  SAFETY_STOP: "/safety/stop",

  // UI
  /** UI command topic */
  UI_COMMAND: "/ui/cmd",
  /** UI goal board topic */
  UI_GOAL: "/ui/goal",
  /** UI events topic */
  UI_EVENTS: "/ui/events",
} as const;

// =============================================================================
// Services
// =============================================================================
export const SERVICES = {
  // Safety Zone
  /** Get current safety zone ROI configuration */
  GET_SAFETY_ZONE: "/safety/get_zone",
  /** Set safety zone ROI configuration */
  SET_SAFETY_ZONE: "/safety/set_zone",

  // HSV Configuration
  /** Get current HSV ranges */
  GET_HSV_RANGES: "/sense/get_hsv_ranges",
  /** Set HSV ranges */
  SET_HSV_RANGES: "/sense/set_hsv_ranges",
  /** Reset HSV ranges to YAML defaults */
  RESET_HSV_RANGES: "/sense/reset_hsv_ranges",
  /** Export HSV ranges as YAML */
  EXPORT_HSV_RANGES_YAML: "/sense/export_hsv_ranges_yaml",
} as const;

// =============================================================================
// Service Types
// =============================================================================
export const SERVICE_TYPES = {
  // Safety Zone
  GET_SAFETY_ZONE: "klotski_interfaces/srv/GetSafetyZone",
  SET_SAFETY_ZONE: "klotski_interfaces/srv/SetSafetyZone",

  // HSV Configuration
  GET_HSV_RANGES: "klotski_interfaces/srv/GetHSVRanges",
  SET_HSV_RANGES: "klotski_interfaces/srv/SetHSVRanges",
  RESET_HSV_RANGES: "klotski_interfaces/srv/ResetHSVRanges",
  EXPORT_HSV_RANGES_YAML: "klotski_interfaces/srv/ExportHSVRangesYaml",
} as const;

// =============================================================================
// Message Types
// =============================================================================
export const MESSAGE_TYPES = {
  // HSV
  HSV_RANGES: "klotski_interfaces/msg/HSVRanges",

  // UI
  UI_COMMAND: "klotski_interfaces/msg/UICommand",
  BOARD: "klotski_interfaces/msg/Board",
  STRING: "std_msgs/String",
} as const;
