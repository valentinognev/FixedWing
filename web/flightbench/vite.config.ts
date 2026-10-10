import tailwindcss from '@tailwindcss/vite'
import react from '@vitejs/plugin-react'
import { defineConfig } from 'vitest/config'

// ./run_flightbench.sh --dev exports FLIGHTBENCH_API_PORT as the API port.
// Unset, /api still proxies to 127.0.0.1:8765 (the Task 7 contract).
const apiPort = process.env.FLIGHTBENCH_API_PORT || '8765'

export default defineConfig({
  plugins: [react(), tailwindcss()],
  server: {
    port: 5173,
    proxy: {
      '/api': {
        target: `http://127.0.0.1:${apiPort}`,
        changeOrigin: true,
      },
    },
  },
  test: {
    environment: 'jsdom',
    setupFiles: ['./src/test/setup.ts'],
  },
})
