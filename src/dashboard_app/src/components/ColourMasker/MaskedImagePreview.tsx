"use client";

interface MaskedImagePreviewProps {
  connected: boolean;
  imageError: boolean;
  maskedImageUrl: string;
  onImageError: () => void;
}

export const MaskedImagePreview: React.FC<MaskedImagePreviewProps> = ({
  connected,
  imageError,
  maskedImageUrl,
  onImageError,
}) => {
  return (
    <div className="shrink-0 space-y-2">
      <div
        className={`
          relative aspect-video overflow-hidden rounded-lg border bg-muted
        `}
      >
        {connected && !imageError ? (
          // Using img tag for streaming content from web_video_server
          // eslint-disable-next-line @next/next/no-img-element
          <img
            src={maskedImageUrl}
            alt="Masked camera view"
            className="h-full w-full object-contain"
            onError={onImageError}
          />
        ) : (
          <div
            className={`
              flex h-full items-center justify-center p-4 text-center text-sm
              text-muted-foreground
            `}
          >
            {!connected
              ? "Connect to ROS to view camera feed"
              : imageError
              ? "Unable to load masked image. Ensure web_video_server is running and camera is connected."
              : "Loading..."}
          </div>
        )}
      </div>
      <p className="text-xs text-muted-foreground">
        Topic: /sense/masked_image
      </p>
    </div>
  );
};
