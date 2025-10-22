import { ControlPanel } from "@/components/ControlPanel";
import { GoalEditor } from "@/components/GoalEditor";

export default function Home() {
  return (
    <main
      className={`
        flex min-h-screen items-center justify-center gap-2 bg-background p-4
      `}
    >
      <GoalEditor />
      <ControlPanel />
    </main>
  );
}
