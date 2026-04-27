import { defineConfig, loadEnv } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), "");
  const backendHttpTarget = env.VITE_BACKEND_URL || "http://localhost:8787";
  const backendWsTarget = backendHttpTarget.replace(/^http/i, "ws");

  return {
    plugins: [react()],
    server: {
      port: 5174,
      fs: {
        allow: [".."],
      },
      proxy: {
        "/api": {
          target: backendHttpTarget,
          changeOrigin: true,
        },
        "/telemetry": {
          target: backendWsTarget,
          ws: true,
        },
      },
    },
  };
});
