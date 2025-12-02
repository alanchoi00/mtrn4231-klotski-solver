"use client";

import type {
  ExportHSVRangesYamlResponse,
  GetHSVRangesResponse,
  HSVRangesMsg,
  ResetHSVRangesResponse,
  SetHSVRangesResponse,
} from "@/lib/ros/types";
import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB, { type Ros } from "roslib";

export interface UseHSVConfigOptions {
  ros?: Ros;
  showToasts?: boolean;
}

export const useHSVConfig = ({ ros }: UseHSVConfigOptions) => {
  const [hsvRanges, setHsvRanges] = useState<HSVRangesMsg | null>(null);
  const [isSubscribed, setIsSubscribed] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const subscriptionRef = useRef<ROSLIB.Topic | null>(null);

  // Get HSV ranges via service call (for initial fetch)
  const getHSVRanges = useCallback(async (): Promise<GetHSVRangesResponse> => {
    if (!ros) {
      return {
        ranges: { ranges: [] },
        ok: false,
        message: "ROS not connected",
      };
    }

    return new Promise((resolve) => {
      const service = new ROSLIB.Service({
        ros,
        name: "/sense/get_hsv_ranges",
        serviceType: "klotski_interfaces/srv/GetHSVRanges",
      });

      const request = new ROSLIB.ServiceRequest({});

      service.callService(
        request,
        (response: GetHSVRangesResponse) => {
          if (response.ok) {
            setHsvRanges(response.ranges);
          }
          resolve(response);
        },
        (error: string) => {
          console.error("Failed to get HSV ranges:", error);
          resolve({ ranges: { ranges: [] }, ok: false, message: error });
        }
      );
    });
  }, [ros]);

  // Subscribe to HSV ranges topic
  useEffect(() => {
    if (!ros) {
      setIsSubscribed(false);
      return;
    }

    try {
      const topic = new ROSLIB.Topic({
        ros,
        name: "/sense/hsv_ranges",
        messageType: "klotski_interfaces/msg/HSVRanges",
      });

      const handleMessage = (message: unknown) => {
        const msg = message as HSVRangesMsg;
        setHsvRanges(msg);
      };

      topic.subscribe(handleMessage);
      subscriptionRef.current = topic;
      setIsSubscribed(true);
      console.log("Subscribed to /sense/hsv_ranges");

      // Fetch initial values via service since we may have missed the initial publish
      setIsLoading(true);
      getHSVRanges().finally(() => setIsLoading(false));

      return () => {
        try {
          topic.unsubscribe(handleMessage as (message: unknown) => void);
          subscriptionRef.current = null;
          setIsSubscribed(false);
          console.log("Unsubscribed from /sense/hsv_ranges");
        } catch (error) {
          console.warn("Error unsubscribing from /sense/hsv_ranges:", error);
        }
      };
    } catch (error) {
      console.error("Failed to subscribe to /sense/hsv_ranges:", error);
      setIsSubscribed(false);
      return () => {};
    }
  }, [ros, getHSVRanges]);

  // Set HSV ranges service call
  const setHSVRanges = useCallback(
    async (ranges: HSVRangesMsg): Promise<SetHSVRangesResponse> => {
      if (!ros) {
        return { ok: false, message: "ROS not connected" };
      }

      return new Promise((resolve) => {
        const service = new ROSLIB.Service({
          ros,
          name: "/sense/set_hsv_ranges",
          serviceType: "klotski_interfaces/srv/SetHSVRanges",
        });

        const request = new ROSLIB.ServiceRequest({ ranges });

        service.callService(
          request,
          (response: SetHSVRangesResponse) => {
            resolve(response);
          },
          (error: string) => {
            console.error("Failed to set HSV ranges:", error);
            resolve({ ok: false, message: error });
          }
        );
      });
    },
    [ros]
  );

  // Reset HSV ranges service call
  const resetHSVRanges =
    useCallback(async (): Promise<ResetHSVRangesResponse> => {
      if (!ros) {
        return {
          ranges: { ranges: [] },
          ok: false,
          message: "ROS not connected",
        };
      }

      return new Promise((resolve) => {
        const service = new ROSLIB.Service({
          ros,
          name: "/sense/reset_hsv_ranges",
          serviceType: "klotski_interfaces/srv/ResetHSVRanges",
        });

        const request = new ROSLIB.ServiceRequest({});

        service.callService(
          request,
          (response: ResetHSVRangesResponse) => {
            resolve(response);
          },
          (error: string) => {
            console.error("Failed to reset HSV ranges:", error);
            resolve({ ranges: { ranges: [] }, ok: false, message: error });
          }
        );
      });
    }, [ros]);

  // Export HSV ranges as YAML service call
  const exportHSVRangesYaml =
    useCallback(async (): Promise<ExportHSVRangesYamlResponse> => {
      if (!ros) {
        return { yaml_content: "", ok: false, message: "ROS not connected" };
      }

      return new Promise((resolve) => {
        const service = new ROSLIB.Service({
          ros,
          name: "/sense/export_hsv_ranges_yaml",
          serviceType: "klotski_interfaces/srv/ExportHSVRangesYaml",
        });

        const request = new ROSLIB.ServiceRequest({});

        service.callService(
          request,
          (response: ExportHSVRangesYamlResponse) => {
            resolve(response);
          },
          (error: string) => {
            console.error("Failed to export HSV ranges:", error);
            resolve({ yaml_content: "", ok: false, message: error });
          }
        );
      });
    }, [ros]);

  return {
    hsvRanges,
    isSubscribed,
    isLoading,
    getHSVRanges,
    setHSVRanges,
    resetHSVRanges,
    exportHSVRangesYaml,
  };
};
