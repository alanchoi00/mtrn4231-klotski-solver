import { clsx, type ClassValue } from "clsx";
import { twMerge } from "tailwind-merge";

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

/**
 * Normalizes a string to contain only digits
 * e.g., "a1b2c3" -> "123"
 */
export const normalizeDigits = (s: string): string =>
  (s || "").replace(/\D/g, "");

/**
 * Generates a random integer between min (inclusive) and max (exclusive)
 */
export const randomInt = (min: number, max: number): number =>
  Math.floor(Math.random() * (max - min)) + min;

/**
 * Fetches JSON data from a URL with error handling
 */
export const fetchJson = async <T>(url: string): Promise<T | null> => {
  try {
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }
    return await response.json();
  } catch (error) {
    console.error(`Failed to fetch JSON from ${url}:`, error);
    return null;
  }
};
