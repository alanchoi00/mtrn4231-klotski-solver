"use client";
import { BoardMsg, UICommandMsg, UIMode } from "@/lib/ros/types";
import { useCallback } from "react";
import ROSLIB, { type Ros } from "roslib";
import { toast } from "sonner";

export const useROSPublishers = (ros?: Ros, showToasts = true) => {
  const sendUICommand = useCallback(
    (mode: UIMode, replan = false) => {
      if (!ros) {
        if (showToasts) {
          toast.error("Cannot send command", {
            description: "ROS not connected",
          });
        }
        return;
      }

      try {
        const pub = new ROSLIB.Topic({
          ros,
          name: "/ui/cmd",
          messageType: "klotski_interfaces/msg/UICommand",
        });

        const msg: UICommandMsg = { mode, replan };
        pub.publish(new ROSLIB.Message(msg));

        if (showToasts) {
          toast.success(`Command sent: ${UIMode[mode]}`, {
            description: replan ? "Replanning requested" : undefined,
          });
        }
        console.log(`UI command sent: ${UIMode[mode]}, replan: ${replan}`);
      } catch (error) {
        console.error("Failed to send UI command:", error);
        if (showToasts) {
          toast.error("Failed to send UI command", {
            description:
              error instanceof Error ? error.message : "Unknown error",
          });
        }
      }
    },
    [ros, showToasts]
  );

  const sendGoalBoard = useCallback(
    (goal: BoardMsg) => {
      if (!ros) {
        if (showToasts) {
          toast.error("Cannot send goal", {
            description: "ROS not connected",
          });
        }
        return;
      }

      try {
        const pub = new ROSLIB.Topic({
          ros,
          name: "/ui/goal",
          messageType: "klotski_interfaces/msg/Board",
        });

        pub.publish(new ROSLIB.Message(goal));

        if (showToasts) {
          toast.success("Goal pattern set!", {
            description: `${goal.pieces.length} pieces on ${goal.spec.cols}×${goal.spec.rows} board`,
          });
        }

        console.log("Goal board sent:", {
          spec: goal.spec,
          pieceCount: goal.pieces.length,
        });
      } catch (error) {
        console.error("Failed to send goal board:", error);
        if (showToasts) {
          toast.error("Failed to send goal board", {
            description:
              error instanceof Error ? error.message : "Unknown error",
          });
        }
      }
    },
    [ros, showToasts]
  );

  return {
    sendUICommand,
    sendGoalBoard,
  };
};
