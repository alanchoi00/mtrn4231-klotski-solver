import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import { ROSProvider } from "@/components/ROSProvider";

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

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <ROSProvider>
        <body
          className={`${geistSans.variable} ${geistMono.variable} antialiased`}
        >
          {children}
        </body>
      </ROSProvider>
    </html>
  );
}
