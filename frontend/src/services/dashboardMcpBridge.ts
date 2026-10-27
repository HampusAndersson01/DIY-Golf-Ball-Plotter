export type DashboardMcpAction = 'connect' | 'generate_diagnostic_job' | 'calibrate' | 'run'
export type DashboardMcpRequestStatus = 'completed' | 'rejected' | 'expired' | 'failed'

type RequestBase = {
  request_id: string
  status: string
  created_at: number
  expires_at: number
  result: Record<string, unknown> | null
}

export type DashboardMcpRequest = RequestBase & (
  | { action: 'connect'; arguments: { com_port: string; usb_vendor_id: number; usb_product_id: number } }
  | { action: 'generate_diagnostic_job'; arguments: { pattern: '3x3_squares' | 'x_axis_rotation_ticks' } }
  | { action: 'calibrate'; arguments: { user_confirmation: string } }
  | { action: 'run'; arguments: { job_id: string; user_instruction: string } }
)

export type DashboardMcpSnapshot = {
  machine: {
    connected: boolean
    calibrated: boolean
    running: boolean
    paused: boolean
    status: string
    current_gcode_line: number
    y_loop_test: { enabled: boolean } | null
  }
  job: { job_id: string | null; line_count: number; summary: Record<string, unknown> | null }
  logs: string[]
  serial_diagnostics: Record<string, unknown>
}

export type ApprovalConfirmations = {
  centeredAtOrigin: boolean
  exactJobAndAreaClear: boolean
}

const MAX_RESPONSE_BYTES = 24 * 1024
const MAX_COMPLETION_BYTES = 16 * 1024
let csrfToken: string | null = null
let sessionInitialization: Promise<void> | null = null

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

async function readJson(response: Response): Promise<Record<string, unknown>> {
  const raw = await response.text()
  if (new TextEncoder().encode(raw).byteLength > MAX_RESPONSE_BYTES) throw new Error('Dashboard MCP response exceeded the size limit.')
  let parsed: unknown
  try {
    parsed = JSON.parse(raw)
  } catch {
    throw new Error('Dashboard MCP returned invalid JSON.')
  }
  if (!isRecord(parsed)) throw new Error('Dashboard MCP returned an invalid response.')
  if (!response.ok || parsed.ok !== true) throw new Error(String(parsed.error ?? `Dashboard MCP request failed (${response.status}).`))
  return parsed
}

async function relay(path: string, init: RequestInit = {}, needsCsrf = false): Promise<Record<string, unknown>> {
  const headers = new Headers(init.headers)
  if (init.body) headers.set('Content-Type', 'application/json')
  if (needsCsrf) {
    if (!csrfToken) throw new Error('Dashboard MCP session is not initialized.')
    headers.set('X-Dashboard-CSRF', csrfToken)
  }
  return readJson(await fetch(path, { ...init, headers, credentials: 'same-origin', cache: 'no-store' }))
}

export async function initializeDashboardMcpSession(): Promise<void> {
  if (sessionInitialization) return sessionInitialization

  const initialization = (async () => {
    const result = await relay('/api/mcp/dashboard/session', { method: 'POST', body: '{}' })
    if (typeof result.csrf_token !== 'string' || result.csrf_token.length < 32 || result.csrf_token.length > 256) {
      throw new Error('Dashboard MCP session returned an invalid CSRF token.')
    }
    csrfToken = result.csrf_token
  })()
  sessionInitialization = initialization
  try {
    await initialization
  } finally {
    if (sessionInitialization === initialization) sessionInitialization = null
  }
}

export async function publishDashboardSnapshot(snapshot: DashboardMcpSnapshot): Promise<void> {
  assertDashboardMcpSnapshot(snapshot)
  await relay('/api/mcp/dashboard/snapshot', { method: 'POST', body: JSON.stringify(snapshot) }, true)
}

export async function fetchPendingDashboardRequest(): Promise<DashboardMcpRequest | null> {
  const result = await relay('/api/mcp/dashboard/requests/next', { method: 'POST', body: '{}' }, true)
  if (result.request == null) return null
  return parseDashboardMcpRequest(result.request)
}

export async function startDashboardRequest(requestId: string): Promise<void> {
  if (!isValidRequestId(requestId)) throw new Error('Invalid dashboard MCP request ID.')
  await relay(`/api/mcp/dashboard/requests/${encodeURIComponent(requestId)}/start`, { method: 'POST', body: '{}' }, true)
}

export async function completeDashboardRequest(
  requestId: string,
  status: DashboardMcpRequestStatus,
  result: object = {},
): Promise<void> {
  if (!isValidRequestId(requestId)) throw new Error('Invalid dashboard MCP request ID.')
  if (!['completed', 'rejected', 'expired', 'failed'].includes(status)) throw new Error('Invalid dashboard MCP completion status.')
  const payload = JSON.stringify({ status, result })
  if (new TextEncoder().encode(payload).byteLength > MAX_COMPLETION_BYTES) throw new Error('Dashboard MCP completion exceeded the size limit.')
  await relay(`/api/mcp/dashboard/requests/${encodeURIComponent(requestId)}/complete`, { method: 'POST', body: payload }, true)
}

export function parseDashboardMcpRequest(value: unknown): DashboardMcpRequest {
  if (!isRecord(value) || !hasExactKeys(value, ['request_id', 'action', 'arguments', 'status', 'created_at', 'expires_at', 'result'])
    || !isValidRequestId(value.request_id) || typeof value.status !== 'string'
    || typeof value.created_at !== 'number' || !Number.isFinite(value.created_at)
    || typeof value.expires_at !== 'number' || !Number.isFinite(value.expires_at) || !isRecord(value.arguments)) {
    throw new Error('Dashboard MCP request has an invalid shape.')
  }
  if (value.result !== null && !isRecord(value.result)) throw new Error('Dashboard MCP request result has an invalid shape.')
  if (value.status !== 'awaiting_operator') throw new Error('Dashboard MCP request is not awaiting operator review.')
  const base = {
    request_id: value.request_id,
    status: value.status,
    created_at: value.created_at,
    // Flask uses monotonic time internally. Convert its remaining duration to
    // this browser's wall clock instead of comparing unrelated clock epochs.
    expires_at: Date.now() / 1000 + Math.max(0, value.expires_at - value.created_at),
    result: isRecord(value.result) ? value.result : null,
  }
  const args = value.arguments
  switch (value.action) {
    case 'connect':
      if (!hasExactKeys(args, ['com_port', 'usb_vendor_id', 'usb_product_id'])
        || typeof args.com_port !== 'string' || !/^COM[1-9][0-9]{0,3}$/.test(args.com_port)
        || !Number.isInteger(args.usb_vendor_id) || (args.usb_vendor_id as number) < 0 || (args.usb_vendor_id as number) > 0xffff
        || !Number.isInteger(args.usb_product_id) || (args.usb_product_id as number) < 0 || (args.usb_product_id as number) > 0xffff) {
        throw new Error('Unsupported connect request arguments.')
      }
      return { ...base, action: 'connect', arguments: {
        com_port: args.com_port,
        usb_vendor_id: args.usb_vendor_id as number,
        usb_product_id: args.usb_product_id as number,
      } }
    case 'generate_diagnostic_job':
      if (!hasExactKeys(args, ['pattern']) || (args.pattern !== '3x3_squares' && args.pattern !== 'x_axis_rotation_ticks')) throw new Error('Unsupported diagnostic pattern.')
      return { ...base, action: 'generate_diagnostic_job', arguments: { pattern: args.pattern } }
    case 'calibrate':
      if (!hasExactKeys(args, ['user_confirmation']) || !isBoundedText(args.user_confirmation)) throw new Error('Invalid calibration confirmation text.')
      return { ...base, action: 'calibrate', arguments: { user_confirmation: args.user_confirmation } }
    case 'run':
      if (!hasExactKeys(args, ['job_id', 'user_instruction']) || !isValidRequestId(args.job_id) || !isBoundedText(args.user_instruction)) throw new Error('Invalid run request arguments.')
      return { ...base, action: 'run', arguments: { job_id: args.job_id, user_instruction: args.user_instruction } }
    default:
      throw new Error('Unsupported dashboard MCP action.')
  }
}

export function canApproveMcpAction(
  request: DashboardMcpRequest,
  confirmations: ApprovalConfirmations,
  currentJobId: string | null,
): boolean {
  if (request.status !== 'awaiting_operator' || Date.now() / 1000 + 1 >= request.expires_at) return false
  if (request.action === 'calibrate') return confirmations.centeredAtOrigin
  if (request.action === 'run') return request.arguments.job_id === currentJobId && confirmations.exactJobAndAreaClear
  return true
}

function isValidRequestId(value: unknown): value is string {
  return typeof value === 'string' && /^[a-z0-9-]{8,64}$/i.test(value)
}

function isBoundedText(value: unknown): value is string {
  return typeof value === 'string' && value.trim().length > 0 && value.length <= 300
}

function hasExactKeys(value: Record<string, unknown>, expected: string[]): boolean {
  const keys = Object.keys(value)
  return keys.length === expected.length && expected.every((key) => Object.hasOwn(value, key))
}

function assertDashboardMcpSnapshot(value: unknown): asserts value is DashboardMcpSnapshot {
  if (!isRecord(value) || !hasExactKeys(value, ['machine', 'job', 'logs', 'serial_diagnostics'])
    || !isRecord(value.machine) || !isRecord(value.job)
    || !Array.isArray(value.logs) || !isRecord(value.serial_diagnostics)) {
    throw new Error('Dashboard MCP snapshot has an invalid shape.')
  }
  const machine = value.machine
  if (!hasExactKeys(machine, ['connected', 'calibrated', 'running', 'paused', 'status', 'current_gcode_line', 'y_loop_test'])
    || !['connected', 'calibrated', 'running', 'paused'].every((key) => typeof machine[key] === 'boolean')
    || typeof machine.status !== 'string' || machine.status.length > 80
    || !Number.isFinite(machine.current_gcode_line) || (machine.current_gcode_line as number) < 0
    || (machine.y_loop_test !== null && (!isRecord(machine.y_loop_test)
      || !hasExactKeys(machine.y_loop_test, ['enabled']) || typeof machine.y_loop_test.enabled !== 'boolean'))) {
    throw new Error('Dashboard MCP machine snapshot is invalid.')
  }
  if (!hasExactKeys(value.job, ['job_id', 'line_count', 'summary'])
    || (value.job.job_id !== null && !isValidRequestId(value.job.job_id))
    || !Number.isSafeInteger(value.job.line_count) || (value.job.line_count as number) < 0
    || (value.job.summary !== null && !isSafeSummary(value.job.summary))) {
    throw new Error('Dashboard MCP job snapshot is invalid.')
  }
  if (value.logs.length > 50 || value.logs.some((line) => typeof line !== 'string' || line.length > 500 || /^\s*>/.test(line))) {
    throw new Error('Dashboard MCP log snapshot is invalid.')
  }
  const diagnostics = value.serial_diagnostics
  const diagnosticKeys = ['last_complete_parsed_line', 'last_parsed_ack_line', 'last_parsed_status_line', 'last_ok_at', 'last_status_at', 'port_state', 'unexpected_ok_count', 'unexpected_error_count', 'recent_grbl_lines']
  if (!hasExactKeys(diagnostics, diagnosticKeys) || !Array.isArray(diagnostics.recent_grbl_lines)
    || diagnostics.recent_grbl_lines.length > 20
    || diagnostics.recent_grbl_lines.some((line) => typeof line !== 'string' || line.length > 160)
    || !isRecord(diagnostics.port_state) || !hasExactKeys(diagnostics.port_state, ['portOpen', 'readerActive', 'writerActive'])
    || Object.values(diagnostics.port_state).some((flag) => typeof flag !== 'boolean')) {
    throw new Error('Dashboard MCP serial diagnostics snapshot is invalid.')
  }
}

function isSafeSummary(value: unknown): value is Record<string, unknown> {
  if (!isRecord(value) || !hasExactKeys(value, [
    'image_size', 'gcode_line_count', 'estimated_runtime_seconds', 'wall_path_count', 'infill_path_count', 'detail_trace_path_count',
  ])) return false
  return typeof value.image_size === 'string' && value.image_size.length <= 80
    && ['gcode_line_count', 'estimated_runtime_seconds', 'wall_path_count', 'infill_path_count', 'detail_trace_path_count']
      .every((key) => typeof value[key] === 'number' && Number.isFinite(value[key]))
}
