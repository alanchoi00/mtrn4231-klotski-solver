"use client";
import React, { createContext, useContext, useEffect, useMemo, useState, useCallback } from "react";
import ROSLIB, { Ros } from "roslib";
import type { UICommandMsg, RosStringMsg, UIMode, BoardMsg } from "@/lib/ros/types";

type ROSCtx = {
  ros?: Ros;
  connected: boolean;
  sendUICommand: (mode: UIMode, replan?: boolean) => void;
  sendGoalBoard: (goal: BoardMsg) => void;
  subscribeEvents: (onMsg: (m: RosStringMsg) => void) => () => void; // returns unsubscribe
};

const ROSContext = createContext<ROSCtx>({
  connected: false,
  sendUICommand: () => {},
  sendGoalBoard: () => {},
  subscribeEvents: () => () => {},
});

export const useROS = () => useContext(ROSContext);

export const ROSProvider: React.FC<{ children: React.ReactNode; url?: string }> = ({
  children,
  url = "ws://localhost:9090",
}) => {
  const [ros, setRos] = useState<Ros>();
  const [connected, setConnected] = useState<boolean>(false);

  useEffect(() => {
    const r = new ROSLIB.Ros({ url });
    const onConn = () => setConnected(true);
    const onClose = () => setConnected(false);
    const onErr = (e: unknown) => console.error("rosbridge error:", e);

    r.on("connection", onConn);
    r.on("close", onClose);
    r.on("error", onErr);
    setRos(r);

    return () => {
      r.off("connection", onConn);
      r.off("close", onClose);
      r.off("error", onErr);
      r.close();
    };
  }, [url]);

  const sendUICommand = useCallback((mode: UIMode, replan = false) => {
    if (!ros) return;
    const pub = new ROSLIB.Topic({
      ros,
      name: "/ui/cmd",
      messageType: "klotski_interfaces/msg/UICommand",
    });
    const msg: UICommandMsg = { mode, replan };
    pub.publish(new ROSLIB.Message(msg));
  }, [ros]);

  const sendGoalBoard = useCallback((goal: BoardMsg) => {
    console.log("sendGoalBoard called with:", goal);
    if (!ros) {
      console.error("No ROS connection available");
      return;
    }
    console.log("ROS connection exists, creating topic...");
    const pub = new ROSLIB.Topic({
      ros,
      name: "/ui/goal",
      messageType: "klotski_interfaces/msg/Board",
    });
    console.log("Publishing message to /ui/goal");
    pub.publish(new ROSLIB.Message(goal));
    console.log("Message published successfully");
  }, [ros]);

  const subscribeEvents = useCallback((onMsg: (m: RosStringMsg) => void) => {
    if (!ros) return () => {};
    const sub = new ROSLIB.Topic({
      ros,
      name: "/ui/events",
      messageType: "std_msgs/String",
    });
    const cb = (m: unknown) => {
      // Tight runtime check to avoid 'any'
      if (typeof m === "object" && m !== null && "data" in m && typeof (m as { data: unknown }).data === "string") {
        onMsg(m as RosStringMsg);
      }
    };
    sub.subscribe(cb);
    return () => sub.unsubscribe(cb as (message: unknown) => void);
  }, [ros]);

  const value = useMemo<ROSCtx>(() => ({ ros, connected, sendUICommand, sendGoalBoard, subscribeEvents }), [ros, connected]);

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
};