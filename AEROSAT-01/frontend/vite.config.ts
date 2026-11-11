import { defineConfig } from "vitest/config";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    host: "127.0.0.1",
  },
  test: {
    // DSH tests exercise the pure store and mapping functions; no browser or DOM is needed.
    environment: "node",
    include: ["src/__tests__/**/*.test.{ts,tsx}"],
  },
});
