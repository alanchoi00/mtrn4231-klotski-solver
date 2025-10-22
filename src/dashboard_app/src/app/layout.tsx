import { ROSProvider } from "@/components/ROSProvider";
import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";

import { Toaster } from "@/components/ui/sonner";
import "./globals.css";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "Klotski Solver Dashboard",
  description: "Dashboard for controlling the Klotski solver",
};

const ROSBRIDGE_URL: string =
  process.env.NEXT_PUBLIC_ROSBRIDGE_URL || "ws://localhost:9090";

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <ROSProvider url={ROSBRIDGE_URL} maxRetries={10} retryInterval={5000}>
        <body
          className={`
            ${geistSans.variable}
            ${geistMono.variable}
            antialiased
          `}
        >
          {children}
        </body>
        <Toaster />
      </ROSProvider>
    </html>
  );
}
