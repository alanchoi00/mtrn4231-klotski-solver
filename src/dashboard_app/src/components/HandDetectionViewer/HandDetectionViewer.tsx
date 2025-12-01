"use client";

import { Button } from "@/components/ui/button";
import {
  SERVICES,
  SERVICE_TYPES,
  TOPICS,
  WEB_VIDEO_SERVER_PORT,
} from "@/lib/constants";
import { useROS } from "@/lib/ros";
import { Check, Edit, Maximize2, Minimize2, RotateCcw, X } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import ROSLIB from "roslib";

interface HandDetectionViewerProps {
  open: boolean;
  onClose: () => void;
}

interface ROIPoint {
  x: number;
  y: number;
}

export const HandDetectionViewer: React.FC<HandDetectionViewerProps> = ({
  open,
  onClose,
}) => {
  const { ros, connected } = useROS();
  const [imageError, setImageError] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);
  const [position, setPosition] = useState({ x: 100, y: 100 });
  const [size, setSize] = useState({ width: 640, height: 480 });
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });

  // Edit mode state
  const [isEditMode, setIsEditMode] = useState(false);
  const [roiPoints, setRoiPoints] = useState<ROIPoint[]>([
    { x: 0.1, y: 0.1 },
    { x: 0.9, y: 0.1 },
    { x: 0.9, y: 0.9 },
    { x: 0.1, y: 0.9 },
  ]);
  const [roiEnabled, setRoiEnabled] = useState(false);
  const [draggingPointIndex, setDraggingPointIndex] = useState<number | null>(
    null
  );
  const [isSaving, setIsSaving] = useState(false);

  // Image aspect ratio (width / height)
  const [imageAspectRatio, setImageAspectRatio] = useState(640 / 480);
  const imageRef = useRef<HTMLImageElement>(null);

  const containerRef = useRef<HTMLDivElement>(null);
  const imageContainerRef = useRef<HTMLDivElement>(null);

  // Build the image URL for web_video_server
  const currentTopic = isEditMode
    ? TOPICS.RAW_IMAGE
    : TOPICS.HAND_DETECTION_IMAGE;
  const imageUrl = useMemo(
    () =>
      `http://${
        typeof window !== "undefined" ? window.location.hostname : "localhost"
      }:${WEB_VIDEO_SERVER_PORT}/stream?topic=${currentTopic}&type=mjpeg&quality=80`,
    [currentTopic]
  );

  // Fetch current ROI from service
  const fetchCurrentROI = useCallback(() => {
    if (!ros || !connected) return;

    const service = new ROSLIB.Service({
      ros,
      name: SERVICES.GET_SAFETY_ZONE,
      serviceType: SERVICE_TYPES.GET_SAFETY_ZONE,
    });

    service.callService(new ROSLIB.ServiceRequest({}), (response) => {
      setRoiPoints([
        { x: response.p1_x, y: response.p1_y },
        { x: response.p2_x, y: response.p2_y },
        { x: response.p3_x, y: response.p3_y },
        { x: response.p4_x, y: response.p4_y },
      ]);
      setRoiEnabled(response.roi_enabled);
    });
  }, [ros, connected]);

  // Apply ROI changes via service
  const applyROIChanges = useCallback(() => {
    if (!ros || !connected) return;

    setIsSaving(true);

    const service = new ROSLIB.Service({
      ros,
      name: SERVICES.SET_SAFETY_ZONE,
      serviceType: SERVICE_TYPES.SET_SAFETY_ZONE,
    });

    const request = new ROSLIB.ServiceRequest({
      p1_x: roiPoints[0].x,
      p1_y: roiPoints[0].y,
      p2_x: roiPoints[1].x,
      p2_y: roiPoints[1].y,
      p3_x: roiPoints[2].x,
      p3_y: roiPoints[2].y,
      p4_x: roiPoints[3].x,
      p4_y: roiPoints[3].y,
      roi_enabled: roiEnabled,
    });

    service.callService(request, (response) => {
      setIsSaving(false);
      if (response.success) {
        setIsEditMode(false);
      }
    });
  }, [ros, connected, roiPoints, roiEnabled]);

  // Enter edit mode
  const enterEditMode = useCallback(() => {
    fetchCurrentROI();
    setIsEditMode(true);
  }, [fetchCurrentROI]);

  // Cancel edit mode
  const cancelEditMode = useCallback(() => {
    fetchCurrentROI(); // Reset to current values
    setIsEditMode(false);
  }, [fetchCurrentROI]);

  // Reset image error when connection changes
  useEffect(() => {
    if (connected) {
      setImageError(false);
      fetchCurrentROI();
    }
  }, [connected, fetchCurrentROI]);

  // Handle window dragging
  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      if (isMaximized) return;
      if ((e.target as HTMLElement).closest("button")) return;

      setIsDragging(true);
      setDragOffset({
        x: e.clientX - position.x,
        y: e.clientY - position.y,
      });
    },
    [position, isMaximized]
  );

  const handleMouseMove = useCallback(
    (e: MouseEvent) => {
      if (isDragging) {
        // Calculate new position
        let newX = e.clientX - dragOffset.x;
        let newY = e.clientY - dragOffset.y;

        // Bound to viewport
        const maxX = window.innerWidth - size.width;
        const maxY = window.innerHeight - size.height;

        newX = Math.max(0, Math.min(newX, maxX));
        newY = Math.max(0, Math.min(newY, maxY));

        setPosition({ x: newX, y: newY });
      } else if (isResizing && containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        // Limit width to not exceed viewport
        const maxWidth = window.innerWidth - position.x;
        const newWidth = Math.max(
          320,
          Math.min(e.clientX - rect.left, maxWidth)
        );
        // Calculate height to maintain aspect ratio (accounting for title bar + status bar = 48px)
        const chromeHeight = 48; // title bar (40px) + status bar (8px)
        const contentHeight = newWidth / imageAspectRatio;
        // Limit height to not exceed viewport
        const maxHeight = window.innerHeight - position.y;
        const newHeight = Math.max(
          240,
          Math.min(contentHeight + chromeHeight, maxHeight)
        );
        setSize({
          width: newWidth,
          height: newHeight,
        });
      } else if (draggingPointIndex !== null && imageContainerRef.current) {
        // Handle ROI point dragging
        const rect = imageContainerRef.current.getBoundingClientRect();
        const x = Math.max(
          0,
          Math.min(1, (e.clientX - rect.left) / rect.width)
        );
        const y = Math.max(
          0,
          Math.min(1, (e.clientY - rect.top) / rect.height)
        );

        setRoiPoints((prev) => {
          const newPoints = [...prev];
          newPoints[draggingPointIndex] = { x, y };
          return newPoints;
        });
      }
    },
    [
      isDragging,
      isResizing,
      dragOffset,
      draggingPointIndex,
      imageAspectRatio,
      size,
      position,
    ]
  );

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
    setIsResizing(false);
    setDraggingPointIndex(null);
  }, []);

  useEffect(() => {
    if (isDragging || isResizing || draggingPointIndex !== null) {
      window.addEventListener("mousemove", handleMouseMove);
      window.addEventListener("mouseup", handleMouseUp);
      return () => {
        window.removeEventListener("mousemove", handleMouseMove);
        window.removeEventListener("mouseup", handleMouseUp);
      };
    }
  }, [
    isDragging,
    isResizing,
    draggingPointIndex,
    handleMouseMove,
    handleMouseUp,
  ]);

  if (!open) return null;

  return (
    <div
      ref={containerRef}
      className={`
        fixed z-[100] flex flex-col overflow-hidden rounded-lg border
        bg-background shadow-2xl select-none
        ${isMaximized ? "inset-4" : ""}
        ${
          isDragging || isResizing || draggingPointIndex !== null
            ? "cursor-grabbing"
            : ""
        }
      `}
      style={{
        ...(isMaximized
          ? {}
          : {
              left: position.x,
              top: position.y,
              width: size.width,
              height: size.height,
            }),
        userSelect: "none",
      }}
    >
      {/* Title bar */}
      <div
        className={`
          flex h-10 shrink-0 cursor-move items-center justify-between border-b
          bg-muted px-3
        `}
        onMouseDown={handleMouseDown}
      >
        <span className="text-sm font-medium">
          Hand Detection Monitor
          {isEditMode && (
            <span className="ml-2 text-xs text-orange-500">(Edit Mode)</span>
          )}
        </span>
        <div className="flex items-center gap-1">
          {!isEditMode ? (
            <Button
              variant="ghost"
              size="icon"
              className="h-7 w-7"
              onClick={enterEditMode}
              title="Edit Safety Zone"
            >
              <Edit className="h-4 w-4" />
            </Button>
          ) : (
            <>
              <Button
                variant="ghost"
                size="icon"
                className="h-7 w-7 text-green-500 hover:text-green-600"
                onClick={applyROIChanges}
                disabled={isSaving}
                title="Apply Changes"
              >
                <Check className="h-4 w-4" />
              </Button>
              <Button
                variant="ghost"
                size="icon"
                className="h-7 w-7"
                onClick={cancelEditMode}
                title="Cancel"
              >
                <RotateCcw className="h-4 w-4" />
              </Button>
            </>
          )}
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7"
            onClick={() => setIsMaximized(!isMaximized)}
          >
            {isMaximized ? (
              <Minimize2 className="h-4 w-4" />
            ) : (
              <Maximize2 className="h-4 w-4" />
            )}
          </Button>
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7 hover:bg-destructive hover:text-destructive-foreground"
            onClick={onClose}
          >
            <X className="h-4 w-4" />
          </Button>
        </div>
      </div>

      {/* Content */}
      <div ref={imageContainerRef} className="relative min-h-0 flex-1 bg-black">
        {connected && !imageError ? (
          <>
            {/* eslint-disable-next-line @next/next/no-img-element */}
            <img
              ref={imageRef}
              src={imageUrl}
              alt="Hand detection view"
              className="h-full w-full object-contain pointer-events-none"
              draggable={false}
              onLoad={(e) => {
                const img = e.currentTarget;
                if (img.naturalWidth && img.naturalHeight) {
                  const ratio = img.naturalWidth / img.naturalHeight;
                  setImageAspectRatio(ratio);
                  // Adjust initial size to match aspect ratio
                  const chromeHeight = 48;
                  const contentHeight = size.width / ratio;
                  setSize((prev) => ({
                    width: prev.width,
                    height: contentHeight + chromeHeight,
                  }));
                }
              }}
              onError={() => setImageError(true)}
            />
            {/* ROI overlay in edit mode - semi-transparent safety zone filter */}
            {isEditMode && (
              <svg
                className="pointer-events-none absolute inset-0 h-full w-full"
                style={{ pointerEvents: "none" }}
                viewBox="0 0 100 100"
                preserveAspectRatio="none"
              >
                <defs>
                  {/* Mask to create the inverted fill effect */}
                  <mask id="safetyZoneMask">
                    {/* White background = visible */}
                    <rect x="0" y="0" width="100" height="100" fill="white" />
                    {/* Black polygon = cut out (transparent) */}
                    <polygon
                      points={roiPoints
                        .map((p) => `${p.x * 100},${p.y * 100}`)
                        .join(" ")}
                      fill="black"
                    />
                  </mask>
                </defs>
                {/* Dark overlay outside the safety zone */}
                <rect
                  x="0"
                  y="0"
                  width="100"
                  height="100"
                  fill="rgba(0, 0, 0, 0.5)"
                  mask="url(#safetyZoneMask)"
                />
                {/* Light tint inside the safety zone */}
                <polygon
                  points={roiPoints
                    .map((p) => `${p.x * 100},${p.y * 100}`)
                    .join(" ")}
                  fill="rgba(0, 255, 100, 0.2)"
                  stroke="#00ff00"
                  strokeWidth="0.5"
                  strokeDasharray="2,1"
                  style={{ pointerEvents: "none" }}
                />
              </svg>
            )}
            {/* Draggable points in edit mode */}
            {isEditMode &&
              roiPoints.map((point, index) => (
                <div
                  key={index}
                  className={`
                    absolute h-5 w-5 -translate-x-1/2 -translate-y-1/2 cursor-move
                    rounded-full border-2 border-white bg-green-500
                    transition-transform hover:scale-125
                  `}
                  style={{
                    left: `${point.x * 100}%`,
                    top: `${point.y * 100}%`,
                    pointerEvents: "auto",
                  }}
                  onMouseDown={(e) => {
                    e.stopPropagation();
                    setDraggingPointIndex(index);
                  }}
                >
                  <span
                    className={`
                      absolute -top-5 left-1/2 -translate-x-1/2 text-xs
                      font-bold text-white
                    `}
                  >
                    {index + 1}
                  </span>
                </div>
              ))}
          </>
        ) : (
          <div
            className={`
              flex h-full items-center justify-center p-4 text-center text-sm
              text-muted-foreground
            `}
          >
            {!connected
              ? "Connect to ROS to view hand detection feed"
              : imageError
              ? "Unable to load image. Ensure web_video_server is running and hand_safety_monitor node is active."
              : "Loading..."}
          </div>
        )}
      </div>

      {/* Status bar - always at bottom */}
      <div
        className={`
          flex h-8 shrink-0 items-center justify-between border-t bg-muted px-2
          text-xs text-muted-foreground mt-auto
        `}
      >
        <div className="flex items-center gap-4">
          <span>Topic: {currentTopic}</span>
          {isEditMode && (
            <label className="flex cursor-pointer items-center gap-1">
              <input
                type="checkbox"
                checked={roiEnabled}
                onChange={(e) => setRoiEnabled(e.target.checked)}
                className="h-3 w-3"
              />
              <span>ROI Enabled</span>
            </label>
          )}
        </div>
        <span
          className={`flex items-center gap-1 ${
            connected ? "text-green-500" : "text-red-500"
          }`}
        >
          <span
            className={`h-2 w-2 rounded-full ${
              connected ? "bg-green-500" : "bg-red-500"
            }`}
          />
          {connected ? "Connected" : "Disconnected"}
        </span>
      </div>

      {/* Resize handle */}
      {!isMaximized && (
        <div
          className="absolute right-0 bottom-0 h-4 w-4 cursor-se-resize"
          onMouseDown={(e) => {
            e.stopPropagation();
            setIsResizing(true);
          }}
        >
          <svg
            className="h-4 w-4 text-muted-foreground"
            viewBox="0 0 24 24"
            fill="currentColor"
          >
            <path d="M22 22H20V20H22V22ZM22 18H20V16H22V18ZM18 22H16V20H18V22ZM22 14H20V12H22V14ZM18 18H16V16H18V18ZM14 22H12V20H14V22Z" />
          </svg>
        </div>
      )}
    </div>
  );
};
