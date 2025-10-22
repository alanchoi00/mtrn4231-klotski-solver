"use client";

import { ROSContext, type ROSContextType } from "@/lib/ros/context";
import {
  useROSConnection,
  type UseROSConnectionOptions,
} from "@/lib/ros/useROSConnection";
import { useROSPublishers } from "@/lib/ros/useROSPublishers";
import { useROSSubscribers } from "@/lib/ros/useROSSubscribers";
import React, { useMemo } from "react";

export { useROS } from "@/lib/ros/context";

interface ROSProviderProps extends UseROSConnectionOptions {
  children: React.ReactNode;
  showToasts?: boolean;
}

export const ROSProvider: React.FC<ROSProviderProps> = ({
  children,
  showToasts = true,
  ...connectionOptions
}) => {
  // Connection management with auto-retry
  const { ros, connected, connecting, retryCount, reconnect } =
    useROSConnection({
      ...connectionOptions,
    });

  // Publisher hooks
  const { sendUICommand, sendGoalBoard } = useROSPublishers(ros, showToasts);

  // Subscriber hooks
  const { subscribeEvents } = useROSSubscribers(ros);

  // Context value
  const value = useMemo<ROSContextType>(
    () => ({
      ros,
      connected,
      connecting,
      retryCount,
      sendUICommand,
      sendGoalBoard,
      subscribeEvents,
      reconnect,
    }),
    [
      ros,
      connected,
      connecting,
      retryCount,
      sendUICommand,
      sendGoalBoard,
      subscribeEvents,
      reconnect,
    ]
  );

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
};
