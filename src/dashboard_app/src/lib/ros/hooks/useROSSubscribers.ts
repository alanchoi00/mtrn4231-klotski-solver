"use client";
import { MESSAGE_TYPES, TOPICS } from "@/lib/constants";
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
          name: TOPICS.UI_EVENTS,
          messageType: MESSAGE_TYPES.STRING,
        });

        const handleMessage = (m: unknown) => {
          if (isValidStringMessage(m)) {
            onMsg(m);
          } else {
            console.warn(
              `Invalid message format received on ${TOPICS.UI_EVENTS}:`,
              m
            );
          }
        };

        sub.subscribe(handleMessage);
        console.log(`Subscribed to ${TOPICS.UI_EVENTS}`);

        return () => {
          try {
            sub.unsubscribe(handleMessage as (message: unknown) => void);
            console.log(`Unsubscribed from ${TOPICS.UI_EVENTS}`);
          } catch (error) {
            console.warn(
              `Error unsubscribing from ${TOPICS.UI_EVENTS}:`,
              error
            );
          }
        };
      } catch (error) {
        console.error(`Failed to subscribe to ${TOPICS.UI_EVENTS}:`, error);
        return () => {};
      }
    },
    [ros]
  );

  return {
    subscribeEvents,
  };
};
