"use client";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Separator } from "@/components/ui/separator";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import { UIMode, useROS } from "@/lib/ros";
import { Pause, Play, RotateCcw, SkipForward } from "lucide-react";
import React, { useEffect, useRef, useState } from "react";

export const ControlPanel: React.FC = () => {
  const { connected, sendUICommand, subscribeEvents } = useROS();
  const [events, setEvents] = useState<string[]>([]);
  const [mode, setMode] = useState<UIMode>(UIMode.MODE_IDLE);
  const scrollAreaRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const unsub = subscribeEvents((m) =>
      setEvents((prev) => [...prev, m.data])
    );
    return () => unsub();
  }, [subscribeEvents]);

  useEffect(() => {
    if (scrollAreaRef.current) {
      const scrollContainer = scrollAreaRef.current.querySelector(
        "[data-radix-scroll-area-viewport]"
      );
      if (scrollContainer) {
        scrollContainer.scrollTop = scrollContainer.scrollHeight;
      }
    }
  }, [events]);

  const pushCmd = (m: UIMode) => {
    setMode(m);
    sendUICommand(m);
  };

  return (
    <TooltipProvider>
      <Card className="w-full max-w-3xl shadow-xl">
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle className="text-2xl">Klotski Control Panel</CardTitle>
              <CardDescription>
                Control the orchestrator and watch system events.
              </CardDescription>
            </div>
            <Badge
              variant={connected ? "default" : "secondary"}
              className="text-sm"
            >
              {connected ? "🟢 Connected" : "🔴 Disconnected"}
            </Badge>
          </div>
        </CardHeader>

        <CardContent className="space-y-4">
          <div className="flex flex-wrap items-center gap-2">
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant={mode === UIMode.MODE_AUTO ? "default" : "outline"}
                  onClick={() => pushCmd(UIMode.MODE_AUTO)}
                >
                  <Play className="mr-2 h-4 w-4" /> Auto
                </Button>
              </TooltipTrigger>
              <TooltipContent>Run the full plan automatically</TooltipContent>
            </Tooltip>

            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant={mode === UIMode.MODE_STEP ? "default" : "outline"}
                  onClick={() => pushCmd(UIMode.MODE_STEP)}
                >
                  <SkipForward className="mr-2 h-4 w-4" /> Step
                </Button>
              </TooltipTrigger>
              <TooltipContent>Execute the next move only</TooltipContent>
            </Tooltip>

            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant={mode === UIMode.MODE_PAUSED ? "default" : "outline"}
                  onClick={() => pushCmd(UIMode.MODE_PAUSED)}
                >
                  <Pause className="mr-2 h-4 w-4" /> Pause
                </Button>
              </TooltipTrigger>
              <TooltipContent>Pause execution</TooltipContent>
            </Tooltip>

            <Separator orientation="vertical" className="mx-1 h-8" />

            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant="outline"
                  onClick={() => pushCmd(UIMode.MODE_RESET)}
                >
                  <RotateCcw className="mr-2 h-4 w-4" /> Reset
                </Button>
              </TooltipTrigger>
              <TooltipContent>
                Reset brain state (no hardware reset)
              </TooltipContent>
            </Tooltip>
          </div>

          <Separator />

          <div>
            <div className="mb-2 flex items-center justify-between">
              <h3 className="font-semibold">Events</h3>
              <Badge variant="secondary">{events.length}</Badge>
            </div>
            <ScrollArea
              ref={scrollAreaRef}
              className={`h-64 rounded-md border bg-muted/30 p-3`}
            >
              <ul className="space-y-1 text-sm">
                {events.map((e, i) => (
                  <li key={i} className="truncate">
                    <Badge
                      variant={events.length !== i + 1 ? "outline" : "default"}
                    >
                      {i + 1}
                    </Badge>{" "}
                    {e}
                  </li>
                ))}
              </ul>
            </ScrollArea>
          </div>
        </CardContent>
      </Card>
    </TooltipProvider>
  );
};
