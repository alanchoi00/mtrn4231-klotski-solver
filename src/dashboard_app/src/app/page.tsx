import { ControlPanel } from "@/components/ControlPanel";
import { ToolsMenu } from "@/components/ToolsMenu";

export default function Home() {
  return (
    <main
      className={`
        flex min-h-screen items-center justify-center bg-background p-4
      `}
    >
      <ControlPanel />
      <ToolsMenu />
    </main>
  );
}
