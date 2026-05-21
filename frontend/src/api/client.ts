import type { ApiSuccess, AppConfig, GenerateResponse, ImageAnalysis, MachineState } from './types'

const ENDPOINTS = {
  bootstrap: '/api/bootstrap',
  state: '/state',
  connect: '/connect',
  applyConfig: '/apply-config',
  penUp: '/pen-up',
  penDown: '/pen-down',
  jog: '/jog',
  zeroAndMarkCalibrated: '/zero-and-mark-calibrated',
  clearCalibrated: '/clear-calibrated',
  applyStepperHoldPolicy: '/stepper-hold/apply',
  goHome: '/go-home',
  yLoopStart: '/y-loop/start',
  yLoopStop: '/y-loop/stop',
  runGcode: '/run-gcode',
  pause: '/pause',
  resume: '/resume',
  stop: '/stop',
  command: '/command',
  generateImageGcode: '/generate-image-gcode',
  generateDiagnosticGcode: '/generate-diagnostic-gcode',
  analyzeImageColors: '/analyze-image-colors',
} as const

async function parseJson<T>(response: Response): Promise<T> {
  const raw = await response.text()
  let payload: Record<string, unknown> | null = null
  if (raw.trim()) {
    try {
      payload = JSON.parse(raw) as Record<string, unknown>
    } catch {
      throw new Error(`Request failed with ${response.status}: ${raw.slice(0, 200)}`)
    }
  }
  if (!response.ok || payload?.ok === false) {
    const message =
      (typeof payload?.error === 'string' && payload.error) ||
      (raw.trim() ? `Request failed with ${response.status}: ${raw.slice(0, 200)}` : `Request failed with ${response.status}`)
    throw new Error(message)
  }
  if (!payload) {
    throw new Error(`Request returned no JSON payload (${response.status})`)
  }
  return payload as T
}

export const apiConfig = {
  endpoints: ENDPOINTS,
}

export async function fetchBootstrap() {
  const response = await fetch(ENDPOINTS.bootstrap)
  return parseJson<AppConfig & { ok: true }>(response)
}

export async function fetchState() {
  const response = await fetch(ENDPOINTS.state)
  return parseJson<MachineState>(response)
}

export async function postJson<T>(endpoint: string, body: unknown = {}) {
  const response = await fetch(endpoint, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  return parseJson<ApiSuccess<T>>(response)
}

export async function postForm<T>(endpoint: string, body: FormData) {
  const response = await fetch(endpoint, {
    method: 'POST',
    body,
  })
  return parseJson<ApiSuccess<T>>(response)
}

export async function analyzeImage(body: FormData) {
  const response = await postForm<{ analysis: ImageAnalysis }>(ENDPOINTS.analyzeImageColors, body)
  return response.analysis
}

export async function generateImageGcode(body: FormData) {
  return postForm<GenerateResponse>(ENDPOINTS.generateImageGcode, body)
}

export async function generateDiagnosticGcode(body: FormData) {
  return postForm<GenerateResponse>(ENDPOINTS.generateDiagnosticGcode, body)
}
