"use client";
import type { BoardMsg, RosStringMsg, UIMode } from "@/lib/ros/types";
import { createContext, useContext } from "react";
import type { Ros } from "roslib";

export interface ROSContextType {
  ros?: Ros;
  connected: boolean;
  connecting: boolean;
  retryCount: number;
  sendUICommand: (mode: UIMode, replan?: boolean) => void;
  sendGoalBoard: (goal: BoardMsg) => void;
  subscribeEvents: (onMsg: (m: RosStringMsg) => void) => () => void;
  reconnect: () => void;
}

export const ROSContext = createContext<ROSContextType>({
  connected: false,
  connecting: false,
  retryCount: 0,
  sendUICommand: () => {},
  sendGoalBoard: () => {},
  subscribeEvents: () => () => {},
  reconnect: () => {},
});

export const useROS = () => {
  const context = useContext(ROSContext);
  if (!context) {
    throw new Error("useROS must be used within a ROSProvider");
  }
  return context;
};
