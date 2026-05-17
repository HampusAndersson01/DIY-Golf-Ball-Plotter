import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    host: '127.0.0.1',
    port: 5173,
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:5000',
        changeOrigin: true,
      },
      '^/(state|connect|apply-config|pen-up|pen-down|pen-test|servo-off|jog|go-home|zero-and-mark-calibrated|clear-calibrated|stepper-hold/apply|y-loop/start|y-loop/stop|run-gcode|pause|resume|stop|command|generate-image-gcode|analyze-image|analyze-image-colors|reset)': {
        target: 'http://127.0.0.1:5000',
        changeOrigin: true,
      },
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
})
