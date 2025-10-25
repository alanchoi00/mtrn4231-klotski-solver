"use client";
import type { RosStringMsg } from "@/lib/ros/types";
import { useCallback } from "react";
import ROSLIB, { type Ros } from "roslib";

const isValidStringMessage = (m: unknown): m is RosStringMsg => {
  return (
    typeof m === "object" &&
    m !== null &&
    "data" in m &&
    typeof (m as { data: unknown }).data === "string"
  );
};

export const useROSSubscribers = (ros?: Ros) => {
  const subscribeEvents = useCallback(
    (onMsg: (m: RosStringMsg) => void) => {
      if (!ros) {
        console.warn("Cannot subscribe to events: ROS not connected");
        return () => {};
      }

      try {
        const sub = new ROSLIB.Topic({
          ros,
          name: "/ui/events",
          messageType: "std_msgs/String",
        });

        const handleMessage = (m: unknown) => {
          if (isValidStringMessage(m)) {
            onMsg(m);
          } else {
            console.warn("Invalid message format received on /ui/events:", m);
          }
        };

        sub.subscribe(handleMessage);
        console.log("Subscribed to /ui/events");

        return () => {
          try {
            sub.unsubscribe(handleMessage as (message: unknown) => void);
            console.log("Unsubscribed from /ui/events");
          } catch (error) {
            console.warn("Error unsubscribing from /ui/events:", error);
          }
        };
      } catch (error) {
        console.error("Failed to subscribe to /ui/events:", error);
        return () => {};
      }
    },
    [ros]
  );

  return {
    subscribeEvents,
  };
};
