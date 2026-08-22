import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  build: {
    // Write artifacts to ASP.NET Core's wwwroot directly.
    // In development, we don't use Vite's dev server (different origin) because ASP.NET Core serves the SPA just like in production.
    outDir: "../backend/wwwroot",
    emptyOutDir: true,
  },
});
