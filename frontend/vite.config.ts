import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const hmrClientPort = Number(env.VITE_HMR_CLIENT_PORT || '')
  const hmrProtocol = env.VITE_HMR_PROTOCOL || ''
  const hmrHost = env.VITE_HMR_HOST || ''

  return {
    plugins: [react()],
    server: {
      host: '127.0.0.1',
      port: 5173,
      strictPort: true,
      hmr:
        Number.isFinite(hmrClientPort) && hmrClientPort > 0
          ? {
              clientPort: hmrClientPort,
              protocol: hmrProtocol || undefined,
              host: hmrHost || undefined,
            }
          : undefined,
      proxy: {
        '/api': {
          target: 'http://127.0.0.1:5000',
          changeOrigin: true,
        },
        '^/(state|connect|apply-config|pen-up|pen-down|pen-test|servo-off|jog|go-home|zero-and-mark-calibrated|clear-calibrated|stepper-hold/apply|y-loop/start|y-loop/stop|run-gcode|pause|resume|stop|command|generate-image-gcode|generate-diagnostic-gcode|analyze-image|analyze-image-colors|reset)': {
          target: 'http://127.0.0.1:5000',
          changeOrigin: true,
        },
      },
    },
    build: {
      outDir: 'dist',
      emptyOutDir: true,
    },
  }
})
