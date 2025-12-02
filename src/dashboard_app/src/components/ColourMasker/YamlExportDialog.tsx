"use client";

import { Button } from "@/components/ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { Textarea } from "@/components/ui/textarea";
import { Check, Copy, Download } from "lucide-react";
import { useCallback, useState } from "react";
import { toast } from "sonner";

interface YamlExportDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  yamlContent: string;
}

export const YamlExportDialog: React.FC<YamlExportDialogProps> = ({
  open,
  onOpenChange,
  yamlContent,
}) => {
  const [copied, setCopied] = useState(false);

  const handleCopy = useCallback(async () => {
    if (!yamlContent) return;

    try {
      await navigator.clipboard.writeText(yamlContent);
      setCopied(true);
      toast.success("YAML copied to clipboard");
      setTimeout(() => setCopied(false), 2000);
    } catch {
      toast.error("Failed to copy to clipboard");
    }
  }, [yamlContent]);

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-2xl">
        <DialogHeader>
          <DialogTitle className="flex items-center gap-2">
            <Download className="h-5 w-5" />
            Export YAML Configuration
          </DialogTitle>
          <DialogDescription>
            Copy this YAML content to your HSV configuration file.
          </DialogDescription>
        </DialogHeader>
        <div className="space-y-4">
          <Textarea
            value={yamlContent}
            readOnly
            className="min-h-[300px] font-mono text-sm"
          />
          <div className="flex justify-end gap-2">
            <Button variant="outline" onClick={() => onOpenChange(false)}>
              Close
            </Button>
            <Button onClick={handleCopy}>
              {copied ? (
                <>
                  <Check className="mr-2 h-4 w-4" /> Copied
                </>
              ) : (
                <>
                  <Copy className="mr-2 h-4 w-4" /> Copy to Clipboard
                </>
              )}
            </Button>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
};
