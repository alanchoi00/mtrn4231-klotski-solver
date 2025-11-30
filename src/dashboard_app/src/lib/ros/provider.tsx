"use client";

import {
  ROSContext,
  useROSConnection,
  useROSPublishers,
  useROSSubscribers,
  type ROSContextType,
  type UseROSConnectionOptions,
} from "@/lib/ros";
import React, { useMemo } from "react";

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

  const { sendUICommand, sendGoalBoard } = useROSPublishers(ros, showToasts);

  const { subscribeEvents } = useROSSubscribers(ros);

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

export default ROSProvider;
