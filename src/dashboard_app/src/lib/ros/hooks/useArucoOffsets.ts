"use client";

import {
  MESSAGE_TYPES,
  SERVICES,
  SERVICE_TYPES,
  TOPICS,
} from "@/lib/constants";
import type {
  ArucoOffsetsMsg,
  GetArucoOffsetsResponse,
  SetArucoOffsetsResponse,
} from "@/lib/ros/types";
import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB, { type Ros } from "roslib";

export interface UseArucoOffsetsOptions {
  ros?: Ros;
}

export const useArucoOffsets = ({ ros }: UseArucoOffsetsOptions) => {
  const [offsets, setOffsets] = useState<ArucoOffsetsMsg | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const subscriptionRef = useRef<ROSLIB.Topic | null>(null);

  // Get ArUco offsets via service call (for initial fetch)
  const getArucoOffsets =
    useCallback(async (): Promise<GetArucoOffsetsResponse> => {
      if (!ros) {
        return {
          offsets: { x: 0, y: 0, z: 0 },
          ok: false,
          message: "ROS not connected",
        };
      }

      return new Promise((resolve) => {
        const service = new ROSLIB.Service({
          ros,
          name: SERVICES.GET_ARUCO_OFFSETS,
          serviceType: SERVICE_TYPES.GET_ARUCO_OFFSETS,
        });

        const request = new ROSLIB.ServiceRequest({});

        service.callService(
          request,
          (response: GetArucoOffsetsResponse) => {
            if (response.ok) {
              setOffsets(response.offsets);
            }
            resolve(response);
          },
          (error: string) => {
            console.error("Failed to get ArUco offsets:", error);
            resolve({
              offsets: { x: 0, y: 0, z: 0 },
              ok: false,
              message: error,
            });
          }
        );
      });
    }, [ros]);

  // Subscribe to ArUco offsets topic
  useEffect(() => {
    if (!ros) {
      setIsSubscribed(false);
      return;
    }

    try {
      const topic = new ROSLIB.Topic({
        ros,
        name: TOPICS.ARUCO_OFFSETS,
        messageType: MESSAGE_TYPES.ARUCO_OFFSETS,
      });

      const handleMessage = (message: unknown) => {
        const msg = message as ArucoOffsetsMsg;
        setOffsets(msg);
      };

      topic.subscribe(handleMessage);
      subscriptionRef.current = topic;
      setIsSubscribed(true);
      console.log(`Subscribed to ${TOPICS.ARUCO_OFFSETS}`);

      // Fetch initial values via service since we may have missed the initial publish
      setIsLoading(true);
      getArucoOffsets().finally(() => setIsLoading(false));

      return () => {
        try {
          topic.unsubscribe(handleMessage as (message: unknown) => void);
          subscriptionRef.current = null;
          setIsSubscribed(false);
          console.log(`Unsubscribed from ${TOPICS.ARUCO_OFFSETS}`);
        } catch (error) {
          console.warn(
            `Error unsubscribing from ${TOPICS.ARUCO_OFFSETS}:`,
            error
          );
        }
      };
    } catch (error) {
      console.error(`Failed to subscribe to ${TOPICS.ARUCO_OFFSETS}:`, error);
      setIsSubscribed(false);
      return () => {};
    }
  }, [ros, getArucoOffsets]);

  // Set ArUco offsets service call
  const setArucoOffsets = useCallback(
    async (newOffsets: ArucoOffsetsMsg): Promise<SetArucoOffsetsResponse> => {
      if (!ros) {
        return { ok: false, message: "ROS not connected" };
      }

      return new Promise((resolve) => {
        const service = new ROSLIB.Service({
          ros,
          name: SERVICES.SET_ARUCO_OFFSETS,
          serviceType: SERVICE_TYPES.SET_ARUCO_OFFSETS,
        });

        const request = new ROSLIB.ServiceRequest({ offsets: newOffsets });

        service.callService(
          request,
          (response: SetArucoOffsetsResponse) => {
            resolve(response);
          },
          (error: string) => {
            console.error("Failed to set ArUco offsets:", error);
            resolve({ ok: false, message: error });
          }
        );
      });
    },
    [ros]
  );

  return {
    offsets,
    isSubscribed,
    isLoading,
    getArucoOffsets,
    setArucoOffsets,
  };
};
