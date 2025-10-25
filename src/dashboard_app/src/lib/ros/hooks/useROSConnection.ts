"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB, { Ros } from "roslib";
import { toast } from "sonner";

export interface UseROSConnectionOptions {
  url: string;
  retryInterval?: number;
  maxRetries?: number;
  connectTimeout?: number;
  showToasts?: boolean;
}

export const useROSConnection = ({
  url,
  retryInterval = 2000,
  maxRetries = Infinity,
  connectTimeout = 4500,
  showToasts = true,
}: UseROSConnectionOptions) => {
  const [ros, setRos] = useState<Ros>();
  const [connected, setConnected] = useState(false);
  const [connecting, setConnecting] = useState(false);
  const [retryCount, setRetryCount] = useState(0);

  const rosRef = useRef<Ros | null>(null);
  const retryTimerRef = useRef<NodeJS.Timeout | null>(null);
  const connectTimerRef = useRef<NodeJS.Timeout | null>(null);

  // guards
  const retryScheduledRef = useRef(false);
  const attemptToastRef = useRef<number>(-1);
  const everConnectedRef = useRef(false);

  const clearRetryTimer = () => {
    if (retryTimerRef.current) {
      clearTimeout(retryTimerRef.current);
      retryTimerRef.current = null;
    }
    retryScheduledRef.current = false;
  };

  const clearConnectTimer = () => {
    if (connectTimerRef.current) {
      clearTimeout(connectTimerRef.current);
      connectTimerRef.current = null;
    }
  };

  const cleanup = useCallback(() => {
    clearRetryTimer();
    clearConnectTimer();
    const r = rosRef.current;
    if (r) {
      try {
        r.removeAllListeners();
        r.close();
      } catch (e) {
        console.warn("ROS close error:", e);
      }
      rosRef.current = null;
    }
  }, []);

  const scheduleRetry = useCallback(() => {
    if (retryScheduledRef.current) return;
    if (retryCount >= maxRetries) return;
    retryScheduledRef.current = true;
    retryTimerRef.current = setTimeout(() => {
      retryScheduledRef.current = false;
      setRetryCount((n) => n + 1);
    }, retryInterval);
  }, [retryCount, maxRetries, retryInterval]);

  const attemptConnection = useCallback(() => {
    if (connecting || connected) return;

    setConnecting(true);
    const attempt = retryCount + 1;

    if (showToasts && attemptToastRef.current !== attempt) {
      attemptToastRef.current = attempt;
    }

    try {
      const r = new ROSLIB.Ros({ url });
      rosRef.current = r;

      const onConnected = () => {
        clearConnectTimer();
        clearRetryTimer();
        everConnectedRef.current = true;
        setConnected(true);
        setConnecting(false);
        setRetryCount(0);
        setRos(r);
        if (showToasts) {
          toast.dismiss("ros-connection");
          toast.success("Connected to ROS bridge!");
        }
      };

      const failThisAttempt = () => {
        clearConnectTimer();
        setConnecting(false);
        scheduleRetry();
      };

      const onError = (e: unknown) => {
        console.error("ROS error:", e);
        failThisAttempt();
      };

      const onClosed = () => {
        setConnected(false);
        setRos(undefined);
        everConnectedRef.current = false; // Reset flag when connection is lost
        if (retryCount < maxRetries && maxRetries !== 0) {
          if (showToasts) {
            toast.error(`ROS connection lost (attempt ${attempt})`, {
              description: `Reconnecting to ${url}...`,
            });
          }
          scheduleRetry();
        }
      };

      clearConnectTimer();
      connectTimerRef.current = setTimeout(() => {
        console.warn("ROS connect timeout");
        try {
          r.close();
        } catch {}
        failThisAttempt();
      }, connectTimeout);

      r.on("connection", onConnected);
      r.on("error", onError);
      r.on("close", onClosed);
    } catch (e) {
      console.error("ROS init error:", e);
      setConnecting(false);
      if (showToasts) {
        toast.dismiss("ros-connection");
        toast.error("Failed to initialize ROS connection");
      }
      scheduleRetry();
    }
  }, [
    connecting,
    connected,
    retryCount,
    showToasts,
    url,
    connectTimeout,
    scheduleRetry,
    maxRetries,
  ]);

  const reconnect = useCallback(() => {
    cleanup();
    attemptToastRef.current = -1;
    everConnectedRef.current = false;
    setRetryCount(0);
    setConnected(false);
    setConnecting(false);
  }, [cleanup]);

  // try on mount & whenever retryCount increments
  useEffect(() => {
    if (retryCount === 0 || (!connecting && !connected && retryCount > 0)) {
      attemptConnection();
    }
  }, [retryCount, connecting, connected, attemptConnection]);

  useEffect(() => () => cleanup(), [cleanup]);

  return { ros, connected, connecting, retryCount, reconnect };
};
