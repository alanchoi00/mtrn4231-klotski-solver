import { ControlPanel } from "@/components/ControlPanel";
import { GoalEditor } from "@/components/GoalEditor";

export default function Home() {
  return (
    <main className="min-h-screen flex items-center justify-center bg-background gap-2 p-4">
      <GoalEditor />
      <ControlPanel />
    </main>
  );
}
