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
  goHome: '/go-home',
  runGcode: '/run-gcode',
  pause: '/pause',
  resume: '/resume',
  stop: '/stop',
  command: '/command',
  generateImageGcode: '/generate-image-gcode',
  analyzeImageColors: '/analyze-image-colors',
} as const

async function parseJson<T>(response: Response): Promise<T> {
  const payload = await response.json()
  if (!response.ok || payload?.ok === false) {
    const message = payload?.error || `Request failed with ${response.status}`
    throw new Error(message)
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
