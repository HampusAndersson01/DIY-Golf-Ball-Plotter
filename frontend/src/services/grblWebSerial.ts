import type { AppConfig, MachineState } from '../api/types'
import type { SettingsState } from '../store/appStore'

export const BAUD_RATE = 115200

export const COMMON_SERIAL_FILTERS: SerialPortFilter[] = [
  { usbVendorId: 0x2341 },
  { usbVendorId: 0x1a86 },
  { usbVendorId: 0x10c4 },
  { usbVendorId: 0x0403 },
]

type RunCallbacks = {
  onProgress?: (done: number, total: number, line: string) => void
  responseTimeoutMs?: number
  streamingMode?: 'buffered' | 'sync'
  ackDelayMs?: number
}

export type StrictAckStressScenario = 'simple_ack' | 'pen_toggle' | 'zero_motion' | 'status_free'

export type StrictAckStressResult = {
  scenario: StrictAckStressScenario
  repetitions: number
  commandsSent: number
  okCount: number
  errorCount: number
  partialChunkCount: number
  partialOCount: number
  timeouts: number
  maxAckLatencyMs: number
  averageAckLatencyMs: number
  statusPollingDisabled: boolean
}

type BrowserMachineState = MachineState & {
  current_servo_s: number
}

type PendingCommand = {
  transactionId: number | null
  lineNumber: number
  command: string
  normalizedCommand: string
  bytes: number
  sentAt: number | null
  acknowledgedAt: number | null
  response: string | null
  expectedStateChange: {
    kind: 'spindle' | 'motion' | 'none'
    spindleSpeed: number | null
    targetX: number | null
    targetY: number | null
  }
  desyncStatusMatches: number
  holdObserved: boolean
}

type GrblCommandResponse = {
  kind: 'ok' | 'error' | 'alarm' | 'startup' | 'message'
  line: string
  receivedAt: number
}

type ParsedGrblStatus = {
  raw: string
  state: string | null
  plannerBufferFree: number | null
  serialRxFree: number | null
  spindleSpeed: number | null
  x: number | null
  y: number | null
}

type PortDiagnostics = {
  portOpen: boolean
  readerActive: boolean
  writerActive: boolean
}

type StreamDebugEvent = {
  at: number
  event: string
  detail: string
}

type SentCommandEvent = {
  at: number
  lineNumber: number
  command: string
}

type TxTraceEntry = {
  at: number
  lineNumber: number | null
  exact: string
  byteLength: number
  escaped: string
}

type RxChunkTraceEntry = {
  at: number
  readLoopId: number
  byteLength: number
  bytesHex: string
  raw: string
  escaped: string
  decoderMode: 'stream'
  bufferBefore: string
  bufferAfter: string
  parsedLines: string[]
  remainingPartial: string
}

type RxLineTraceEntry = {
  at: number
  readLoopId: number
  line: string
  produced: 'ok' | 'error' | 'status' | 'other'
  activeTransactionId: number | null
  activeTransactionCommand: string | null
  unexpectedAck: boolean
}

type TransactionLifecycleEvent = {
  at: number
  event: string
  transactionId: number | null
  lineNumber: number | null
  command: string | null
  detail?: string
}

type StrictAckTransaction = {
  id: number
  lineNumber: number
  command: string
  normalizedCommand: string
  txText: string
  byteLength: number
  sentAt: number | null
  createdAt: number
  resolved: boolean
  timeoutId: ReturnType<typeof globalThis.setTimeout> | null
  resolve: (result: StrictAckTransactionResult) => void
  reject: (error: Error) => void
}

type StrictAckTransactionResult = {
  command: string
  response: string
  bytes: number
  receivedAt: number
  transactionId: number
}

const textEncoder = new TextEncoder()
const GRBL_RX_BUFFER_SIZE = 128
const DEFAULT_STREAM_RESPONSE_TIMEOUT_MS = 20_000
const ACK_SILENCE_STATUS_PROBE_MS = 1_500
const STATUS_QUERY_TIMEOUT_MS = 250
const RECENT_TRACE_LIMIT = 100
const DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS = 4_000
const STREAM_DEBUG_EVENT_LIMIT = 40
const POST_TIMEOUT_OBSERVATION_MS = 2_000

function sleep(ms: number) {
  return new Promise((resolve) => globalThis.setTimeout(resolve, ms))
}

function normalizeErrorMessage(error: unknown): string {
  if (error instanceof Error) {
    if (error.name === 'NotFoundError') {
      return 'Serial port selection was cancelled.'
    }
    if (error.name === 'InvalidStateError') {
      return 'The selected serial port is already open.'
    }
    if (error.name === 'NetworkError') {
      return 'Serial port permission was revoked or the device was disconnected.'
    }
    return error.message || error.name
  }
  return String(error)
}

function escapeSerialPayload(payload: string) {
  return payload.replace(/\r/g, '\\r').replace(/\n/g, '\\n')
}

function bytesToHex(value: Uint8Array) {
  return Array.from(value, (byte) => byte.toString(16).toUpperCase().padStart(2, '0')).join(' ')
}

function nowSeconds() {
  return Date.now() / 1000
}

export function parseGrblSerialChunk(buffer: string, chunk: string) {
  const combined = buffer + chunk
  const parts = combined.split(/\r?\n/)
  return {
    lines: parts.slice(0, -1).map((line) => line.trim()).filter(Boolean),
    remainder: parts.at(-1) ?? '',
  }
}

function sanitizeGcodeLineForStreaming(line: string) {
  const trimmed = line.trim()
  if (!trimmed) {
    return null
  }

  let result = ''
  let parenthesisDepth = 0
  for (const char of trimmed) {
    if (char === ';' && parenthesisDepth === 0) {
      break
    }
    if (char === '(') {
      parenthesisDepth += 1
      continue
    }
    if (char === ')' && parenthesisDepth > 0) {
      parenthesisDepth -= 1
      continue
    }
    if (parenthesisDepth === 0) {
      result += char
    }
  }

  const sanitized = result.trim()
  return sanitized || null
}

function buildDisconnectedMachineState(): BrowserMachineState {
  return {
    connected: false,
    calibrated: false,
    machine_position_trusted: false,
    emergency_stopped: false,
    running: false,
    paused: false,
    status: 'Not connected',
    progress_done: 0,
    progress_total: 0,
    run_started_at: null,
    run_finished_at: null,
    job_started_at: null,
    job_finished_at: null,
    pause_started_at: null,
    paused_duration_seconds: 0,
    job_elapsed_seconds: 0,
    current_gcode_line: 0,
    current_path_id: null,
    current_preview_point_index: 0,
    current_servo_s: 575,
    current_position_x: 0,
    current_position_y: 0,
    motor_hold_enabled: false,
    last_timeout_debug: null,
    defaults: {
      pen_up_s: 575,
      pen_down_s: 700,
      pen_up_dwell_ms: 30,
      pen_down_dwell_ms: 60,
      servo_ramp_enabled: true,
      servo_ramp_step: 20,
      servo_ramp_delay_ms: 10,
    },
    y_loop_test: {
      enabled: false,
      center_y: 0,
      distance: 10,
      feedrate: 1200,
      dwell_sec: 0.25,
      phase: 'idle',
      cycles_completed: 0,
    },
    streaming: {
      mode: 'sync',
      current_line: 0,
      current_path_id: null,
      current_path_kind: null,
      pending_buffer_chars: 0,
      pending_commands: 0,
      last_response_age_sec: 0,
      last_grbl_status: null,
      ok_count: 0,
      error_count: 0,
      sent_count: 0,
      acked_count: 0,
      total_lines: 0,
      streaming_active: false,
    },
    last_summary: null,
  }
}

function parseGrblStatus(line: string): ParsedGrblStatus | null {
  if (!line.startsWith('<') || !line.endsWith('>')) {
    return null
  }

  const raw = line.slice(1, -1)
  const parts = raw.split('|')
  const coords = parts
    .slice(1)
    .map((part) => part.split(':', 2) as [string, string])
    .find(([key]) => key === 'MPos' || key === 'WPos')

  const [x, y] = coords?.[1]?.split(',').slice(0, 2).map((value) => Number(value)) ?? [null, null]
  const bfField = parts
    .slice(1)
    .map((part) => part.split(':', 2) as [string, string])
    .find(([key]) => key === 'Bf')?.[1]
  const [plannerBufferFree, serialRxFree] = bfField?.split(',').map((value) => Number(value)) ?? [null, null]
  const fsField = parts
    .slice(1)
    .map((part) => part.split(':', 2) as [string, string])
    .find(([key]) => key === 'FS')?.[1]
  const [, spindleSpeed] = fsField?.split(',').map((value) => Number(value)) ?? [null, null]

  return {
    raw: line,
    state: parts[0] ?? null,
    plannerBufferFree: Number.isFinite(plannerBufferFree) ? plannerBufferFree : null,
    serialRxFree: Number.isFinite(serialRxFree) ? serialRxFree : null,
    spindleSpeed: Number.isFinite(spindleSpeed) ? spindleSpeed : null,
    x: Number.isFinite(x) ? x : null,
    y: Number.isFinite(y) ? y : null,
  }
}

function mergeMachineState(current: BrowserMachineState, patch: Partial<BrowserMachineState>): BrowserMachineState {
  return {
    ...current,
    ...patch,
    y_loop_test: patch.y_loop_test ?? current.y_loop_test,
    streaming: patch.streaming ?? current.streaming,
    last_summary: patch.last_summary ?? current.last_summary,
  }
}

function buildPenCommands(endS: number, dwellMs: number) {
  const commands = [`M3 S${endS}`]
  if (dwellMs > 0) {
    commands.push(`G4 P${(dwellMs / 1000).toFixed(3)}`)
  }
  return commands
}

function formatAxisMove(axis: 'X' | 'Y', degrees: number, feed: number) {
  return `G1 ${axis}${degrees.toFixed(6)} F${feed.toFixed(3)}`
}

function isSerializedCommand(command: string) {
  const upper = command.trim().toUpperCase()
  return upper.startsWith('G4') || upper.startsWith('M3') || upper.startsWith('M4') || upper.startsWith('M5')
}

function normalizeCommand(command: string) {
  return command.trim().replace(/\s+/g, ' ').toUpperCase()
}

function parseSpindleCommand(command: string) {
  const match = normalizeCommand(command).match(/^M[345]\s+S(-?\d+(?:\.\d+)?)$/)
  if (!match) {
    return null
  }
  const speed = Number(match[1])
  return Number.isFinite(speed) ? speed : null
}

function buildExpectedStateChange(command: string): PendingCommand['expectedStateChange'] {
  const spindleSpeed = parseSpindleCommand(command)
  if (spindleSpeed != null) {
    return {
      kind: 'spindle',
      spindleSpeed,
      targetX: null,
      targetY: null,
    }
  }
  const normalized = normalizeCommand(command)
  const xMatch = normalized.match(/(?:^|\s)X(-?\d+(?:\.\d+)?)/)
  const yMatch = normalized.match(/(?:^|\s)Y(-?\d+(?:\.\d+)?)/)
  const targetX = xMatch ? Number(xMatch[1]) : null
  const targetY = yMatch ? Number(yMatch[1]) : null
  if (targetX != null || targetY != null) {
    return {
      kind: 'motion',
      spindleSpeed: null,
      targetX: Number.isFinite(targetX) ? targetX : null,
      targetY: Number.isFinite(targetY) ? targetY : null,
    }
  }
  return {
    kind: 'none',
    spindleSpeed: null,
    targetX: null,
    targetY: null,
  }
}

function formatPendingQueue(pendingCommands: PendingCommand[]) {
  return pendingCommands.map((pending) => `L${pending.lineNumber}:${pending.command}`)
}

export class GrblWebSerialService {
  private activePort: SerialPort | null = null
  private isConnecting = false
  private readBuffer = ''
  private stopRequested = false
  private machine: BrowserMachineState = buildDisconnectedMachineState()
  private yLoopAbortController: AbortController | null = null
  private portReader: ReadableStreamDefaultReader<Uint8Array> | null = null
  private portWriter: WritableStreamDefaultWriter<Uint8Array> | null = null
  private readLoopPromise: Promise<void> | null = null
  private readLoopStopped = false
  private nextReadLoopId = 1
  private activeReadLoopId: number | null = null
  private strictAckStreaming = false
  private nextTransactionId = 1
  private activeTransaction: StrictAckTransaction | null = null
  private commandResponseQueue: GrblCommandResponse[] = []
  private commandWaiters: Array<{
    resolve: (response: GrblCommandResponse) => void
    reject: (error: Error) => void
  }> = []
  private statusWaiters: Array<{
    resolve: (line: string) => void
    reject: (error: Error) => void
  }> = []
  private streamPendingCommands: PendingCommand[] = []
  private recentGrblLines: string[] = []
  private recentSentCommands: SentCommandEvent[] = []
  private txTrace: TxTraceEntry[] = []
  private rxChunkTrace: RxChunkTraceEntry[] = []
  private rxLineTrace: RxLineTraceEntry[] = []
  private transactionLifecycleEvents: TransactionLifecycleEvent[] = []
  private unexpectedOkCount = 0
  private unexpectedErrorCount = 0
  private streamDebugEvents: StreamDebugEvent[] = []
  private lastResponseAt: number | null = null
  private lastOkAt: number | null = null
  private lastStatusAt: number | null = null
  private lastRawSerialChunk = ''
  private lastRawSerialChunkAt: number | null = null
  private lastCompleteParsedLine: string | null = null
  private lastCompleteParsedLineAt: number | null = null
  private lastParsedAckLine: string | null = null
  private lastParsedAckLineAt: number | null = null
  private lastParsedStatusLine: string | null = null
  private lastParsedStatusLineAt: number | null = null
  private partialLineBufferUpdatedAt: number | null = null
  private postTimeoutObservationUntil = 0

  getMachineState() {
    const snapshot = structuredClone(this.machine)
    const start = snapshot.run_started_at ?? snapshot.job_started_at
    if (start != null) {
      const pausedDuration = snapshot.paused_duration_seconds ?? 0
      const pauseStarted = snapshot.paused && snapshot.pause_started_at != null ? snapshot.pause_started_at : null
      const now = Date.now() / 1000
      const pausedExtra = pauseStarted != null ? Math.max(0, now - pauseStarted) : 0
      snapshot.job_elapsed_seconds = Math.max(0, now - start - pausedDuration - pausedExtra)
    }
    if (snapshot.streaming) {
      snapshot.streaming.last_response_age_sec = this.lastResponseAt == null ? 0 : Math.max(0, Date.now() / 1000 - this.lastResponseAt)
    }
    return snapshot
  }

  getPort() {
    return this.activePort
  }

  getStreamDiagnostics() {
    return {
      machine: this.getMachineState(),
      recent_grbl_lines: [...this.recentGrblLines],
      recent_sent_commands: [...this.recentSentCommands],
      tx_trace: [...this.txTrace],
      rx_chunks: [...this.rxChunkTrace],
      rx_lines: [...this.rxLineTrace],
      transaction_lifecycle_events: [...this.transactionLifecycleEvents],
      active_transaction: this.activeTransactionSnapshot(),
      unexpected_ok_count: this.unexpectedOkCount,
      unexpected_error_count: this.unexpectedErrorCount,
      stream_debug_events: [...this.streamDebugEvents],
      last_raw_serial_chunk: this.lastRawSerialChunk || null,
      last_raw_serial_chunk_at: this.lastRawSerialChunkAt,
      current_partial_line_buffer: this.readBuffer || null,
      partial_line_buffer_updated_at: this.partialLineBufferUpdatedAt,
      last_complete_parsed_line: this.lastCompleteParsedLine,
      last_complete_parsed_line_at: this.lastCompleteParsedLineAt,
      last_parsed_ack_line: this.lastParsedAckLine,
      last_parsed_ack_line_at: this.lastParsedAckLineAt,
      last_parsed_status_line: this.lastParsedStatusLine,
      last_parsed_status_line_at: this.lastParsedStatusLineAt,
      last_ok_at: this.lastOkAt,
      last_status_at: this.lastStatusAt,
      port_state: this.buildPortDiagnostics(),
      read_loop_id: this.activeReadLoopId,
      status_polling_state: {
        disabled_for_strict_ack: this.strictAckStreaming,
        waiters: this.statusWaiters.length,
      },
    }
  }

  async getPreviouslyApprovedPorts() {
    if (!navigator.serial) return []
    return navigator.serial.getPorts()
  }

  private updateMachine(patch: Partial<BrowserMachineState>) {
    this.machine = mergeMachineState(this.machine, patch)
    return this.getMachineState()
  }

  private pushStreamDebugEvent(event: string, detail: string) {
    this.streamDebugEvents.push({
      at: Date.now() / 1000,
      event,
      detail,
    })
    if (this.streamDebugEvents.length > STREAM_DEBUG_EVENT_LIMIT) {
      this.streamDebugEvents.shift()
    }
  }

  private appendPostTimeoutObservation(key: 'rx_chunks' | 'rx_lines', entry: RxChunkTraceEntry | RxLineTraceEntry) {
    if (Date.now() > this.postTimeoutObservationUntil) {
      return
    }
    const debug = this.machine.last_timeout_debug
    if (!debug || typeof debug !== 'object') {
      return
    }
    const observation = {
      ...(debug.post_timeout_observation && typeof debug.post_timeout_observation === 'object'
        ? debug.post_timeout_observation as Record<string, unknown>
        : {}),
    }
    const entries = Array.isArray(observation[key]) ? observation[key] as Array<RxChunkTraceEntry | RxLineTraceEntry> : []
    observation[key] = [...entries, entry].slice(-50)
    observation.last_observed_at = nowSeconds()
    this.updateMachine({
      last_timeout_debug: {
        ...debug,
        post_timeout_observation: observation,
      },
    })
  }

  private recordSentCommand(lineNumber: number, command: string) {
    this.recentSentCommands.push({
      at: Date.now() / 1000,
      lineNumber,
      command,
    })
    if (this.recentSentCommands.length > RECENT_TRACE_LIMIT) {
      this.recentSentCommands.shift()
    }
  }

  private recordTxTrace(payload: string, lineNumber: number | null, byteLength: number) {
    this.txTrace.push({
      at: Date.now() / 1000,
      lineNumber,
      exact: payload,
      byteLength,
      escaped: escapeSerialPayload(payload),
    })
    if (this.txTrace.length > RECENT_TRACE_LIMIT) {
      this.txTrace.shift()
    }
  }

  private recordRxChunkTrace(
    value: Uint8Array,
    raw: string,
    readLoopId: number,
    bufferBefore: string,
    bufferAfter: string,
    parsedLines: string[],
  ) {
    const entry: RxChunkTraceEntry = {
      at: Date.now() / 1000,
      readLoopId,
      byteLength: value.byteLength,
      bytesHex: bytesToHex(value),
      raw,
      escaped: escapeSerialPayload(raw),
      decoderMode: 'stream',
      bufferBefore,
      bufferAfter,
      parsedLines,
      remainingPartial: bufferAfter,
    }
    this.rxChunkTrace.push(entry)
    if (this.rxChunkTrace.length > RECENT_TRACE_LIMIT) {
      this.rxChunkTrace.shift()
    }
    this.appendPostTimeoutObservation('rx_chunks', entry)
  }

  private recordRxLineTrace(line: string, readLoopId: number, produced: RxLineTraceEntry['produced']) {
    const activeTransaction = this.activeTransaction
    const entry: RxLineTraceEntry = {
      at: Date.now() / 1000,
      readLoopId,
      line,
      produced,
      activeTransactionId: activeTransaction?.id ?? null,
      activeTransactionCommand: activeTransaction?.command ?? null,
      unexpectedAck: (produced === 'ok' || produced === 'error') && !activeTransaction,
    }
    this.rxLineTrace.push(entry)
    if (this.rxLineTrace.length > RECENT_TRACE_LIMIT) {
      this.rxLineTrace.shift()
    }
    this.appendPostTimeoutObservation('rx_lines', entry)
  }

  private recordTransactionLifecycle(
    event: string,
    transaction: Pick<StrictAckTransaction, 'id' | 'lineNumber' | 'command'> | null,
    detail?: string,
  ) {
    this.transactionLifecycleEvents.push({
      at: Date.now() / 1000,
      event,
      transactionId: transaction?.id ?? null,
      lineNumber: transaction?.lineNumber ?? null,
      command: transaction?.command ?? null,
      detail,
    })
    if (this.transactionLifecycleEvents.length > 200) {
      this.transactionLifecycleEvents.shift()
    }
    this.pushStreamDebugEvent(event, `${transaction ? `T${transaction.id} L${transaction.lineNumber} ${transaction.command}` : 'no active transaction'}${detail ? ` ${detail}` : ''}`)
  }

  private activeTransactionSnapshot() {
    const tx = this.activeTransaction
    if (!tx) {
      return null
    }
    return {
      id: tx.id,
      lineNumber: tx.lineNumber,
      command: tx.command,
      sentAt: tx.sentAt,
      createdAt: tx.createdAt,
      resolved: tx.resolved,
      byteLength: tx.byteLength,
    }
  }

  private getStreamingState() {
    return this.machine.streaming ?? buildDisconnectedMachineState().streaming!
  }

  private ensureSerialSupport() {
    if (!navigator.serial) {
      throw new Error('Web Serial is not supported. Use Chrome or Edge on desktop.')
    }
  }

  private ensureConnectedPort() {
    if (!this.activePort || !this.portWriter) {
      throw new Error('Connect plotter first.')
    }
    return this.activePort
  }

  private recordGrblLine(line: string) {
    this.recentGrblLines.push(line)
    if (this.recentGrblLines.length > RECENT_TRACE_LIMIT) {
      this.recentGrblLines.shift()
    }
    const receivedAt = nowSeconds()
    this.lastResponseAt = receivedAt
    this.lastCompleteParsedLine = line
    this.lastCompleteParsedLineAt = receivedAt
    if (line === 'ok' || line.toUpperCase().startsWith('ERROR:') || line.toUpperCase().startsWith('ALARM:')) {
      this.lastParsedAckLine = line
      this.lastParsedAckLineAt = receivedAt
    }
    if (line.startsWith('<')) {
      this.lastParsedStatusLine = line
      this.lastParsedStatusLineAt = receivedAt
    }
  }

  private buildPortDiagnostics(): PortDiagnostics {
    return {
      portOpen: Boolean(this.activePort),
      readerActive: Boolean(this.portReader),
      writerActive: Boolean(this.portWriter),
    }
  }

  private rejectWaiters(waiters: Array<{ reject: (error: Error) => void }>, error: Error) {
    const stale = waiters.splice(0, waiters.length)
    for (const waiter of stale) {
      waiter.reject(error)
    }
  }

  private enqueueCommandResponse(response: GrblCommandResponse) {
    const waiter = this.commandWaiters.shift()
    if (waiter) {
      waiter.resolve(response)
      return
    }
    this.commandResponseQueue.push(response)
  }

  private handleIncomingStatusLine(line: string) {
    const parsedStatus = parseGrblStatus(line)
    this.lastStatusAt = Date.now() / 1000
    this.pushStreamDebugEvent('status', line)
    const state = parsedStatus?.state ?? line
    const isHoldState = typeof parsedStatus?.state === 'string' && parsedStatus.state.startsWith('Hold')
    const isRunState = parsedStatus?.state === 'Run'
    const pauseStartedAt = isHoldState && !this.machine.paused
      ? Date.now() / 1000
      : this.machine.pause_started_at
    this.updateMachine({
      status: state,
      paused: isHoldState ? true : isRunState ? false : this.machine.paused,
      job_state: this.machine.running
        ? isHoldState
          ? 'paused'
          : isRunState
            ? 'running'
            : this.machine.job_state
        : this.machine.job_state,
      pause_started_at: pauseStartedAt,
      current_position_x: parsedStatus?.x ?? this.machine.current_position_x,
      current_position_y: parsedStatus?.y ?? this.machine.current_position_y,
      streaming: {
        ...this.getStreamingState(),
        last_grbl_status: line,
        last_response_age_sec: 0,
      },
    })

    const waiter = this.statusWaiters.shift()
    if (waiter) {
      waiter.resolve(line)
    }
  }

  private classifyCommandResponse(line: string): GrblCommandResponse {
    const upper = line.toUpperCase()
    if (line === 'ok') {
      return { kind: 'ok', line, receivedAt: Date.now() / 1000 }
    }
    if (upper.startsWith('ERROR:')) {
      return { kind: 'error', line, receivedAt: Date.now() / 1000 }
    }
    if (upper.startsWith('ALARM:')) {
      return { kind: 'alarm', line, receivedAt: Date.now() / 1000 }
    }
    if (/grbl/i.test(line)) {
      return { kind: 'startup', line, receivedAt: Date.now() / 1000 }
    }
    return { kind: 'message', line, receivedAt: Date.now() / 1000 }
  }

  private clearActiveTransaction(transaction: StrictAckTransaction, event: string) {
    if (this.activeTransaction?.id !== transaction.id) {
      this.recordTransactionLifecycle('TX_CLEAR_MISMATCH', transaction, `active=${this.activeTransaction?.id ?? 'none'}`)
      return false
    }
    if (transaction.timeoutId) {
      globalThis.clearTimeout(transaction.timeoutId)
      transaction.timeoutId = null
    }
    this.activeTransaction = null
    this.recordTransactionLifecycle(event, transaction)
    return true
  }

  private clearPendingForTransaction(transaction: StrictAckTransaction, response: string) {
    const pendingIndex = this.streamPendingCommands.findIndex((pending) => pending.transactionId === transaction.id)
    if (pendingIndex < 0) {
      this.recordTransactionLifecycle('TX_PENDING_MISSING', transaction)
      return
    }
    const pending = this.streamPendingCommands[pendingIndex]
    pending.acknowledgedAt = Date.now() / 1000
    pending.response = response
    this.streamPendingCommands.splice(pendingIndex, 1)
    this.updatePendingQueueState()
  }

  private completeActiveTransactionWithOk(line: string, readLoopId: number) {
    const transaction = this.activeTransaction
    if (!transaction) {
      this.unexpectedOkCount += 1
      this.recordTransactionLifecycle('UNEXPECTED_OK', null, `readLoopId=${readLoopId}`)
      if (!this.strictAckStreaming || this.commandWaiters.length > 0) {
        this.enqueueCommandResponse(this.classifyCommandResponse(line))
      }
      return true
    }

    transaction.resolved = true
    this.recordTransactionLifecycle('RX_OK_FOR_TX', transaction, `readLoopId=${readLoopId}`)
    this.clearPendingForTransaction(transaction, line)
    this.clearActiveTransaction(transaction, 'TX_CLEAR')
    this.recordTransactionLifecycle('TX_RESOLVE', transaction, 'ok')
    transaction.resolve({
      command: transaction.command,
      response: line,
      bytes: transaction.byteLength,
      receivedAt: Date.now() / 1000,
      transactionId: transaction.id,
    })
    return true
  }

  private completeActiveTransactionWithError(line: string, readLoopId: number) {
    const transaction = this.activeTransaction
    if (!transaction) {
      this.unexpectedErrorCount += 1
      this.recordTransactionLifecycle('UNEXPECTED_ERROR', null, `${line} readLoopId=${readLoopId}`)
      if (!this.strictAckStreaming || this.commandWaiters.length > 0) {
        this.enqueueCommandResponse(this.classifyCommandResponse(line))
      }
      return true
    }

    transaction.resolved = true
    this.recordTransactionLifecycle('RX_ERROR_FOR_TX', transaction, `${line} readLoopId=${readLoopId}`)
    this.clearPendingForTransaction(transaction, line)
    this.clearActiveTransaction(transaction, 'TX_CLEAR')
    this.recordTransactionLifecycle('TX_REJECT', transaction, line)
    transaction.reject(new Error(`GRBL error for line ${transaction.lineNumber}: ${transaction.command}: ${line}`))
    return true
  }

  private handleIncomingLine(line: string, readLoopId = this.activeReadLoopId ?? 0) {
    this.recordGrblLine(line)
    this.recordTransactionLifecycle('RX_LINE', this.activeTransaction, `${line} readLoopId=${readLoopId}`)
    if (line.startsWith('<')) {
      this.recordRxLineTrace(line, readLoopId, 'status')
      this.handleIncomingStatusLine(line)
      return
    }
    if (line === 'ok') {
      this.lastOkAt = Date.now() / 1000
      this.recordRxLineTrace(line, readLoopId, 'ok')
      this.completeActiveTransactionWithOk(line, readLoopId)
      return
    } else if (line.toUpperCase().startsWith('ERROR:') || line.toUpperCase().startsWith('ALARM:')) {
      this.recordRxLineTrace(line, readLoopId, 'error')
      this.completeActiveTransactionWithError(line, readLoopId)
      return
    } else {
      this.recordRxLineTrace(line, readLoopId, 'other')
    }
    this.pushStreamDebugEvent('response', line)
    this.enqueueCommandResponse(this.classifyCommandResponse(line))
  }

  private async startReadLoop() {
    if (!this.portReader) return
    if (this.activeReadLoopId != null || (this.readLoopPromise && !this.readLoopStopped)) {
      throw new Error(`Serial read loop invariant violated: second read loop requested while readLoopId=${this.activeReadLoopId ?? 'unknown'} is active.`)
    }
    const reader = this.portReader
    const readLoopId = this.nextReadLoopId
    this.nextReadLoopId += 1
    this.activeReadLoopId = readLoopId
    this.readLoopStopped = false
    const decoder = new TextDecoder()
    this.readLoopPromise = (async () => {
      try {
        while (!this.readLoopStopped) {
          const { value, done } = await reader.read()
          if (done) break
          if (!value) continue
          const bufferBefore = this.readBuffer
          const decoded = decoder.decode(value, { stream: true })
          this.lastRawSerialChunk = decoded
          this.lastRawSerialChunkAt = nowSeconds()
          this.pushStreamDebugEvent('RX_CHUNK', `R${readLoopId} bytes=[${bytesToHex(value)}] text="${escapeSerialPayload(decoded)}" bufferBefore="${escapeSerialPayload(bufferBefore)}"`)
          const extracted = parseGrblSerialChunk(bufferBefore, decoded)
          this.readBuffer = extracted.remainder
          this.partialLineBufferUpdatedAt = this.readBuffer ? nowSeconds() : null
          this.recordRxChunkTrace(value, decoded, readLoopId, bufferBefore, this.readBuffer, extracted.lines)
          this.recordTransactionLifecycle(
            'RX_CHUNK',
            this.activeTransaction,
            `readLoopId=${readLoopId} bytes=[${bytesToHex(value)}] text="${escapeSerialPayload(decoded)}" bufferBefore="${escapeSerialPayload(bufferBefore)}" bufferAfter="${escapeSerialPayload(this.readBuffer)}" parsed=${JSON.stringify(extracted.lines)}`,
          )
          for (const line of extracted.lines) {
            this.handleIncomingLine(line, readLoopId)
          }
        }
      } catch (error) {
        if (!this.readLoopStopped) {
          const message = error instanceof Error ? error.message : String(error)
          const failure = new Error(`Serial read loop stopped unexpectedly: ${message}`)
          this.rejectWaiters(this.commandWaiters, failure)
          this.rejectWaiters(this.statusWaiters, failure)
        }
      } finally {
        if (this.activeReadLoopId === readLoopId) {
          this.activeReadLoopId = null
        }
      }
    })()
  }

  private async openSerialSession(port: SerialPort) {
    this.portReader = port.readable?.getReader() ?? null
    this.portWriter = port.writable?.getWriter() ?? null
    if (!this.portReader || !this.portWriter) {
      throw new Error('Selected serial port is not readable/writable.')
    }
    await this.startReadLoop()
  }

  private async stopReadLoop() {
    this.readLoopStopped = true
    try {
      await this.portReader?.cancel()
    } catch {
      // Best-effort cancellation during cleanup.
    }
    try {
      await this.readLoopPromise
    } catch {
      // Cleanup path; read loop failures are already surfaced elsewhere.
    }
  }

  private resetSerialState() {
    this.readBuffer = ''
    this.commandResponseQueue = []
    if (this.activeTransaction && !this.activeTransaction.resolved) {
      const tx = this.activeTransaction
      tx.resolved = true
      if (tx.timeoutId) {
        globalThis.clearTimeout(tx.timeoutId)
      }
      tx.reject(new Error('Serial session reset.'))
    }
    this.activeTransaction = null
    this.streamPendingCommands = []
    this.recentGrblLines = []
    this.recentSentCommands = []
    this.txTrace = []
    this.rxChunkTrace = []
    this.rxLineTrace = []
    this.transactionLifecycleEvents = []
    this.unexpectedOkCount = 0
    this.unexpectedErrorCount = 0
    this.streamDebugEvents = []
    this.lastResponseAt = null
    this.lastOkAt = null
    this.lastStatusAt = null
    this.lastRawSerialChunk = ''
    this.lastRawSerialChunkAt = null
    this.lastCompleteParsedLine = null
    this.lastCompleteParsedLineAt = null
    this.lastParsedAckLine = null
    this.lastParsedAckLineAt = null
    this.lastParsedStatusLine = null
    this.lastParsedStatusLineAt = null
    this.partialLineBufferUpdatedAt = null
    this.postTimeoutObservationUntil = 0
    this.rejectWaiters(this.commandWaiters, new Error('Serial session reset.'))
    this.rejectWaiters(this.statusWaiters, new Error('Serial session reset.'))
  }

  private async closeSerialPort() {
    const port = this.activePort
    await this.stopReadLoop()
    try {
      if (this.portWriter && 'close' in this.portWriter && typeof this.portWriter.close === 'function') {
        await this.portWriter.close()
      }
    } catch {
      // Best-effort cleanup.
    }
    try {
      this.portReader?.releaseLock()
    } catch {
      // Best-effort cleanup.
    }
    try {
      this.portWriter?.releaseLock()
    } catch {
      // Best-effort cleanup.
    }
    this.portReader = null
    this.portWriter = null
    this.readLoopPromise = null
    this.readLoopStopped = false
    this.activeReadLoopId = null
    if (port) {
      try {
        await port.close()
      } catch {
        // Ignore close failures during cleanup.
      }
    }
    this.activePort = null
    this.resetSerialState()
  }

  private async writeRaw(payload: string, lineNumber: number | null = null) {
    this.ensureConnectedPort()
    if (!this.portWriter) {
      throw new Error('Selected serial port is not writable.')
    }
    const encoded = textEncoder.encode(payload)
    await this.portWriter.write(encoded)
    this.recordTxTrace(payload, lineNumber, encoded.byteLength)
  }

  private normalizeStrictCommandLine(command: string) {
    const trimmed = command.trim()
    if (!trimmed) {
      throw new Error('Refusing to send an empty GRBL command in Strict Ack mode.')
    }
    if (trimmed.includes('\n') || trimmed.includes('\r')) {
      throw new Error(`Refusing to send command with embedded line break: ${JSON.stringify(command)}`)
    }
    const payload = `${trimmed}\n`
    const encoded = textEncoder.encode(payload)
    if (encoded.byteLength !== payload.length) {
      throw new Error(`Encoded byte count mismatch for "${trimmed}": encoded=${encoded.byteLength} expected=${payload.length}`)
    }
    return { command: trimmed, payload, encoded, byteLength: encoded.byteLength }
  }

  private async waitForCommandResponse(timeoutMs: number) {
    const queued = this.commandResponseQueue.shift()
    if (queued) {
      return queued
    }

    return new Promise<GrblCommandResponse>((resolve, reject) => {
      const timer = globalThis.setTimeout(() => {
        const index = this.commandWaiters.findIndex((waiter) => waiter.resolve === resolve)
        if (index >= 0) {
          this.commandWaiters.splice(index, 1)
        }
        reject(new Error('Timed out waiting for GRBL command acknowledgement'))
      }, timeoutMs)

      this.commandWaiters.push({
        resolve: (response) => {
          globalThis.clearTimeout(timer)
          resolve(response)
        },
        reject: (error) => {
          globalThis.clearTimeout(timer)
          reject(error)
        },
      })
    })
  }

  private async waitForStatusLine(timeoutMs: number) {
    return new Promise<string>((resolve, reject) => {
      const timer = globalThis.setTimeout(() => {
        const index = this.statusWaiters.findIndex((waiter) => waiter.resolve === resolve)
        if (index >= 0) {
          this.statusWaiters.splice(index, 1)
        }
        reject(new Error('Timed out waiting for GRBL status response'))
      }, timeoutMs)

      this.statusWaiters.push({
        resolve: (line) => {
          globalThis.clearTimeout(timer)
          resolve(line)
        },
        reject: (error) => {
          globalThis.clearTimeout(timer)
          reject(error)
        },
      })
    })
  }

  private updatePendingQueueState() {
    this.updateMachine({
      streaming: {
        ...this.getStreamingState(),
        pending_commands: this.streamPendingCommands.length,
        pending_buffer_chars: this.streamPendingCommands.reduce((total, pending) => total + pending.bytes, 0),
      },
    })
  }

  private resolveCommandTimeoutMs(command: string, timeoutMs: number) {
    void command
    return timeoutMs
  }

  private classifyTimeout(desyncDetails: Record<string, unknown> | null = null) {
    if (desyncDetails?.failure_class && typeof desyncDetails.failure_class === 'string') {
      return desyncDetails.failure_class
    }
    const port = this.buildPortDiagnostics()
    const activeTransactionId = this.activeTransaction?.id ?? null
    const activeTransactionObservedAt = this.activeTransaction?.sentAt ?? this.activeTransaction?.createdAt ?? null
    const lastOkEntry = [...this.rxLineTrace].reverse().find((entry) => entry.produced === 'ok')
    const completeLineBelongsToActiveTransaction = activeTransactionObservedAt == null || (
      this.lastCompleteParsedLineAt != null && this.lastCompleteParsedLineAt >= activeTransactionObservedAt
    )
    const partialLineBelongsToActiveTransaction = activeTransactionObservedAt == null || (
      this.partialLineBufferUpdatedAt != null && this.partialLineBufferUpdatedAt >= activeTransactionObservedAt
    )
    const rawChunkBelongsToActiveTransaction = activeTransactionObservedAt == null || (
      this.lastRawSerialChunkAt != null && this.lastRawSerialChunkAt >= activeTransactionObservedAt
    )
    if (!port.portOpen || !port.readerActive || !port.writerActive) {
      return 'TIMEOUT_PORT_CLOSED'
    }
    const lastLine = this.lastCompleteParsedLine ?? ''
    if (/^Grbl\b/i.test(lastLine)) {
      return 'TIMEOUT_CONTROLLER_RESET'
    }
    if (this.readBuffer && partialLineBelongsToActiveTransaction) {
      return 'TIMEOUT_PARTIAL_LINE'
    }
    if (
      completeLineBelongsToActiveTransaction
      && this.lastCompleteParsedLine === 'ok'
      && this.lastParsedAckLine === 'ok'
      && activeTransactionId != null
      && lastOkEntry?.activeTransactionId === activeTransactionId
    ) {
      return 'TIMEOUT_AFTER_COMPLETE_OK_BUT_NOT_RESOLVED'
    }
    if (this.lastCompleteParsedLine?.startsWith('<') && completeLineBelongsToActiveTransaction) {
      return 'TIMEOUT_AFTER_COMPLETE_STATUS'
    }
    if (this.lastCompleteParsedLine && completeLineBelongsToActiveTransaction) {
      return 'TIMEOUT_INVALID_COMPLETE_LINE'
    }
    if (!this.lastRawSerialChunk || !rawChunkBelongsToActiveTransaction) {
      return 'TIMEOUT_NO_RX'
    }
    if (this.lastRawSerialChunk && !this.lastCompleteParsedLine) {
      return 'TIMEOUT_SERIAL_GARBAGE'
    }
    if (this.readLoopPromise && !this.readLoopStopped) {
      return 'TIMEOUT_READER_STALLED'
    }
    return 'TIMEOUT_NO_RX'
  }

  private buildTimeoutError(
    pendingCommands: PendingCommand[],
    currentLineNumber: number,
    responseTimeoutMs: number,
    statusLine: string | null,
    desyncDetails: Record<string, unknown> | null = null,
  ) {
    const timedOutCommand = pendingCommands[0] ?? null
    const lastSentCommand = pendingCommands.at(-1) ?? null
    const now = nowSeconds()
    const failureClass = this.classifyTimeout(desyncDetails)
    const timeoutDebug = {
      timeout_ms: responseTimeoutMs,
      last_sent_command: lastSentCommand?.command ?? null,
      last_sent_line: lastSentCommand?.lineNumber ?? null,
      timed_out_command: timedOutCommand?.command ?? null,
      timed_out_line: timedOutCommand?.lineNumber ?? null,
      timed_out_command_age_ms: timedOutCommand?.sentAt != null ? Math.max(0, Math.round((Date.now() / 1000 - timedOutCommand.sentAt) * 1000)) : null,
      current_line: currentLineNumber,
      pending_queue_length: pendingCommands.length,
      bytes_in_flight: pendingCommands.reduce((total, pending) => total + pending.bytes, 0),
      pending_queue: pendingCommands.map((pending) => ({
        transactionId: pending.transactionId,
        lineNumber: pending.lineNumber,
        command: pending.command,
        normalizedCommand: pending.normalizedCommand,
        bytes: pending.bytes,
        sentAt: pending.sentAt,
        ackedAt: pending.acknowledgedAt,
        response: pending.response,
        expectedStateChange: pending.expectedStateChange,
      })),
      active_transaction: this.activeTransactionSnapshot(),
      active_transaction_id: this.activeTransaction?.id ?? null,
      active_transaction_existed_at_last_ok: [...this.rxLineTrace].reverse().find((entry) => entry.produced === 'ok')?.activeTransactionId != null,
      last_ok_was_unexpected: [...this.rxLineTrace].reverse().find((entry) => entry.produced === 'ok')?.unexpectedAck ?? null,
      last_raw_fragment: this.lastRawSerialChunk || null,
      last_raw_fragment_at: this.lastRawSerialChunkAt,
      current_partial_line_buffer: this.readBuffer || null,
      partial_line_buffer_updated_at: this.partialLineBufferUpdatedAt,
      last_complete_parsed_line: this.lastCompleteParsedLine,
      last_complete_parsed_line_at: this.lastCompleteParsedLineAt,
      last_parsed_ack_line: this.lastParsedAckLine,
      last_parsed_ack_line_at: this.lastParsedAckLineAt,
      last_parsed_status_line: this.lastParsedStatusLine,
      last_parsed_status_line_at: this.lastParsedStatusLineAt,
      timing_ms: {
        since_write_started: this.activeTransaction?.createdAt != null ? Math.max(0, Math.round((now - this.activeTransaction.createdAt) * 1000)) : null,
        since_write_resolved: this.activeTransaction?.sentAt != null ? Math.max(0, Math.round((now - this.activeTransaction.sentAt) * 1000)) : null,
        since_first_partial_rx_byte: this.readBuffer && this.partialLineBufferUpdatedAt != null ? Math.max(0, Math.round((now - this.partialLineBufferUpdatedAt) * 1000)) : null,
        since_last_raw_rx_byte: this.lastRawSerialChunkAt != null ? Math.max(0, Math.round((now - this.lastRawSerialChunkAt) * 1000)) : null,
        since_last_complete_line: this.lastCompleteParsedLineAt != null ? Math.max(0, Math.round((now - this.lastCompleteParsedLineAt) * 1000)) : null,
      },
      last_100_tx_entries: [...this.txTrace],
      last_100_rx_chunks: [...this.rxChunkTrace],
      last_100_parsed_rx_lines: [...this.rxLineTrace],
      last_200_transaction_lifecycle_events: [...this.transactionLifecycleEvents],
      last_100_received_grbl_lines: [...this.recentGrblLines],
      last_100_sent_commands: [...this.recentSentCommands],
      last_grbl_response: this.lastCompleteParsedLine,
      last_raw_serial_chunk: this.lastRawSerialChunk || null,
      last_ok_at: this.lastOkAt,
      last_status_at: this.lastStatusAt,
      stream_debug_events: [...this.streamDebugEvents],
      status_query_response: statusLine,
      port_state: this.buildPortDiagnostics(),
      serial_read_loop_active: Boolean(this.readLoopPromise && !this.readLoopStopped),
      read_loop_id: this.activeReadLoopId,
      writer_active: Boolean(this.portWriter),
      status_polling_state: {
        disabled_for_strict_ack: this.strictAckStreaming,
        waiters: this.statusWaiters.length,
      },
      streaming_mode: this.getStreamingState().mode,
      failure_class: failureClass,
      desync_details: desyncDetails,
      post_timeout_observation: {
        observe_ms: POST_TIMEOUT_OBSERVATION_MS,
        rx_chunks: [],
        rx_lines: [],
      },
    }
    this.updateMachine({ last_timeout_debug: timeoutDebug })
    this.postTimeoutObservationUntil = Date.now() + POST_TIMEOUT_OBSERVATION_MS

    const portDiagnostics = this.buildPortDiagnostics()
    const lastCompleteLine = this.lastCompleteParsedLine ?? 'none'
    const lastRawFragment = this.lastRawSerialChunk ? escapeSerialPayload(this.lastRawSerialChunk) : 'none'
    const partialBuffer = this.readBuffer ? escapeSerialPayload(this.readBuffer) : 'none'
    const pendingQueueText = formatPendingQueue(pendingCommands).join(', ') || 'empty'
    const statusText = statusLine ?? 'none'
    return new Error(
      `${timeoutDebug.failure_class}: ` +
      `GRBL communication timeout at line ${timedOutCommand?.lineNumber ?? currentLineNumber} after "${timedOutCommand?.command ?? 'unknown command'}". ` +
      `Last sent="${lastSentCommand?.command ?? 'none'}". Pending=${pendingCommands.length} [${pendingQueueText}]. ` +
      `Last complete GRBL line="${lastCompleteLine}". Last raw fragment="${lastRawFragment}". Partial RX buffer="${partialBuffer}". Status query="${statusText}". ` +
      `Port open=${portDiagnostics.portOpen} reader=${portDiagnostics.readerActive} writer=${portDiagnostics.writerActive}.` +
      (desyncDetails ? ' Command state appears applied but the matching ok was not observed.' : ''),
    )
  }

  private async sendLineAndWait(command: string, timeoutMs = DEFAULT_COMMAND_RESPONSE_TIMEOUT_MS) {
    const result = await this.sendLineAndWaitForAck(command, 0, timeoutMs)
    return {
      command: result.command,
      response: result.response,
      lines: [result.response],
    }
  }

  private async sendLineAndWaitForAck(line: string, lineNumber: number, timeoutMs = DEFAULT_STREAM_RESPONSE_TIMEOUT_MS) {
    if (this.activeTransaction) {
      throw new Error(`Strict Ack violation: command already active (T${this.activeTransaction.id} L${this.activeTransaction.lineNumber} ${this.activeTransaction.command})`)
    }
    if (this.streamPendingCommands.length > 0) {
      throw new Error('Strict Ack mode invariant violated: attempted to queue a command while another command is pending.')
    }
    this.ensureConnectedPort()
    if (!this.portWriter) {
      throw new Error('Selected serial port is not writable.')
    }

    const normalized = this.normalizeStrictCommandLine(line)
    const resolvedTimeoutMs = this.resolveCommandTimeoutMs(normalized.command, timeoutMs)

    return new Promise<StrictAckTransactionResult>((resolve, reject) => {
      const transaction: StrictAckTransaction = {
        id: this.nextTransactionId,
        lineNumber,
        command: normalized.command,
        normalizedCommand: normalizeCommand(normalized.command),
        txText: normalized.payload,
        byteLength: normalized.byteLength,
        sentAt: null,
        createdAt: Date.now() / 1000,
        resolved: false,
        timeoutId: null,
        resolve,
        reject,
      }
      this.nextTransactionId += 1

      const pending: PendingCommand = {
        transactionId: transaction.id,
        lineNumber,
        command: normalized.command,
        normalizedCommand: transaction.normalizedCommand,
        bytes: normalized.byteLength,
        sentAt: null,
        acknowledgedAt: null,
        response: null,
        expectedStateChange: buildExpectedStateChange(normalized.command),
        desyncStatusMatches: 0,
        holdObserved: false,
      }

      this.activeTransaction = transaction
      this.streamPendingCommands.push(pending)
      this.recordSentCommand(lineNumber, normalized.command)
      this.recordTransactionLifecycle('TX_CREATE', transaction)
      this.updatePendingQueueState()

      const armTimeout = () => {
        transaction.timeoutId = globalThis.setTimeout(() => {
          void (async () => {
            const pendingCommand = this.streamPendingCommands.find((entry) => entry.transactionId === transaction.id)
            if (this.machine.paused && !this.stopRequested && this.activeTransaction?.id === transaction.id && !transaction.resolved) {
              this.recordTransactionLifecycle('TX_TIMEOUT_DEFERRED', transaction, 'machine paused')
              armTimeout()
              return
            }
            if (this.activeTransaction?.id !== transaction.id || transaction.resolved) {
              return
            }
            const recovery = await this.tryRecoverTimedOutTransaction(transaction, pendingCommand, resolvedTimeoutMs)
            if (recovery?.recovered) {
              return
            }
            if (this.activeTransaction?.id !== transaction.id || transaction.resolved) {
              return
            }
            this.recordTransactionLifecycle('TX_TIMEOUT', transaction)
            transaction.resolved = true
            const error = recovery?.error ?? this.buildTimeoutError(this.streamPendingCommands, Math.max(0, lineNumber - 1), resolvedTimeoutMs, null)
            this.clearActiveTransaction(transaction, 'TX_CLEAR')
            reject(error)
          })().catch((error) => {
            if (this.activeTransaction?.id !== transaction.id || transaction.resolved) {
              return
            }
            this.recordTransactionLifecycle('TX_TIMEOUT_RECOVERY_ERROR', transaction, normalizeErrorMessage(error))
            transaction.resolved = true
            const timeoutError = this.buildTimeoutError(this.streamPendingCommands, Math.max(0, lineNumber - 1), resolvedTimeoutMs, null)
            this.clearActiveTransaction(transaction, 'TX_CLEAR')
            reject(timeoutError)
          })
        }, resolvedTimeoutMs)
      }
      armTimeout()

      void (async () => {
        try {
          this.recordTransactionLifecycle('TX_WRITE_START', transaction, escapeSerialPayload(normalized.payload))
          await this.portWriter!.write(normalized.encoded)
          transaction.sentAt = Date.now() / 1000
          const pendingCommand = this.streamPendingCommands.find((entry) => entry.transactionId === transaction.id)
          if (pendingCommand) {
            pendingCommand.sentAt = transaction.sentAt
          }
          this.recordTxTrace(normalized.payload, lineNumber, normalized.byteLength)
          this.recordTransactionLifecycle('TX_WRITE_DONE', transaction, `${normalized.byteLength} bytes`)
        } catch (error) {
          const message = normalizeErrorMessage(error)
          transaction.resolved = true
          this.clearPendingForTransaction(transaction, `write failed: ${message}`)
          this.clearActiveTransaction(transaction, 'TX_CLEAR')
          this.recordTransactionLifecycle('TX_REJECT', transaction, message)
          reject(new Error(message, { cause: error }))
        }
      })()
    })
  }

  private async queryStatus(timeoutMs = 1500) {
    if (this.strictAckStreaming) {
      throw new Error('Status polling is disabled during Strict Ack streaming.')
    }
    return this.queryStatusBypassStrictAck(timeoutMs)
  }

  private async queryStatusBypassStrictAck(timeoutMs = 1500) {
    const waitForStatus = this.waitForStatusLine(timeoutMs)
    await this.writeRaw('?')
    try {
      return await waitForStatus
    } catch {
      return null
    }
  }

  private commandMatchesObservedStatus(pending: PendingCommand | undefined, parsedStatus: ParsedGrblStatus | null) {
    if (!pending || !parsedStatus) {
      return { matched: false, reason: 'missing_pending_or_status' }
    }
    if (pending.expectedStateChange.kind === 'spindle') {
      return {
        matched: parsedStatus.spindleSpeed === pending.expectedStateChange.spindleSpeed,
        reason: 'spindle_speed_compare',
      }
    }
    if (pending.expectedStateChange.kind === 'motion') {
      const tolerance = 0.05
      const xMatches = pending.expectedStateChange.targetX == null || (
        parsedStatus.x != null && Math.abs(parsedStatus.x - pending.expectedStateChange.targetX) <= tolerance
      )
      const yMatches = pending.expectedStateChange.targetY == null || (
        parsedStatus.y != null && Math.abs(parsedStatus.y - pending.expectedStateChange.targetY) <= tolerance
      )
      return {
        matched: xMatches && yMatches,
        reason: 'axis_target_compare',
      }
    }
    return { matched: false, reason: 'no_expected_state_change' }
  }

  private async tryRecoverTimedOutTransaction(
    transaction: StrictAckTransaction,
    pending: PendingCommand | undefined,
    responseTimeoutMs: number,
  ) {
    const failureClass = this.classifyTimeout()
    if (failureClass !== 'TIMEOUT_INVALID_COMPLETE_LINE') {
      return null
    }
    const statusLine = await this.queryStatusBypassStrictAck(STATUS_QUERY_TIMEOUT_MS)
    const parsedStatus = statusLine ? parseGrblStatus(statusLine) : null
    const statusMatch = this.commandMatchesObservedStatus(pending, parsedStatus)
    const desyncDetails = {
      failure_class: statusMatch.matched ? 'RECOVERED_FROM_STATUS_MATCH' : failureClass,
      status_line: statusLine,
      parsed_status: parsedStatus,
      status_match_reason: statusMatch.reason,
      expected_state_change: pending?.expectedStateChange ?? null,
    }
    if (!statusMatch.matched) {
      return {
        recovered: false,
        statusLine,
        error: this.buildTimeoutError(this.streamPendingCommands, Math.max(0, transaction.lineNumber - 1), responseTimeoutMs, statusLine, desyncDetails),
      }
    }

    this.recordTransactionLifecycle('TX_RECOVER_STATUS_MATCH', transaction, statusLine ?? 'no status line')
    transaction.resolved = true
    this.clearPendingForTransaction(transaction, 'ok (recovered from status)')
    this.clearActiveTransaction(transaction, 'TX_CLEAR')
    this.recordTransactionLifecycle('TX_RESOLVE', transaction, 'ok (recovered from status)')
    this.updateMachine({ last_timeout_debug: null })
    transaction.resolve({
      command: transaction.command,
      response: 'ok',
      bytes: transaction.byteLength,
      receivedAt: Date.now() / 1000,
      transactionId: transaction.id,
    })
    return { recovered: true, statusLine, error: null }
  }

  private async performHandshake() {
    await this.writeRaw('\r\n\r\n')
    await sleep(250)

    const startupLines: string[] = []
    const handshakeDeadline = Date.now() + 1500
    while (Date.now() < handshakeDeadline) {
      const response = this.commandResponseQueue.shift()
      if (!response) {
        await sleep(10)
        continue
      }
      startupLines.push(response.line)
      if (/grbl/i.test(response.line)) {
        this.updateMachine({ status: 'Connected to GRBL' })
        await this.queryStatus().catch(() => null)
        return startupLines
      }
    }

    const statusProbe = await this.queryStatus(1500)
    if (statusProbe) {
      this.updateMachine({ status: 'Connected to GRBL' })
      return [...startupLines, statusProbe]
    }

    throw new Error('GRBL does not respond. Select the Arduino/GRBL controller and try again.')
  }

  async connect() {
    if (this.isConnecting) {
      throw new Error('Connection already in progress')
    }

    this.ensureSerialSupport()
    this.isConnecting = true
    try {
      if (this.activePort || this.portReader || this.portWriter) {
        await this.closeSerialPort()
      }

      const port = await navigator.serial!.requestPort({
        filters: COMMON_SERIAL_FILTERS,
      })

      await port.open({
        baudRate: BAUD_RATE,
        dataBits: 8,
        stopBits: 1,
        parity: 'none',
        flowControl: 'none',
      })

      this.activePort = port
      await this.openSerialSession(port)
      await this.performHandshake()

      this.stopRequested = false
      this.updateMachine({
        connected: true,
        emergency_stopped: false,
        status: 'Connected to GRBL',
      })

      return port
    } catch (error) {
      await this.closeSerialPort()
      this.updateMachine(buildDisconnectedMachineState())
      throw new Error(normalizeErrorMessage(error), { cause: error })
    } finally {
      this.isConnecting = false
    }
  }

  async disconnect() {
    this.stopRequested = true
    this.yLoopAbortController?.abort()
    await this.closeSerialPort()
    this.updateMachine(buildDisconnectedMachineState())
  }

  async sendCommands(commands: string[], successStatus?: string) {
    const responses: string[] = []
    for (const command of commands) {
      const result = await this.sendLineAndWait(command)
      responses.push(...result.lines)
    }
    await this.queryStatus().catch(() => null)
    if (successStatus) {
      this.updateMachine({ status: successStatus })
    }
    return {
      command: commands.join(' ; '),
      response: responses.join('\n') || 'ok',
      lines: responses,
    }
  }

  async runStrictAckStressTest(
    scenario: StrictAckStressScenario,
    repetitions: number,
    options: { responseTimeoutMs?: number } = {},
  ): Promise<StrictAckStressResult> {
    if (!Number.isInteger(repetitions) || repetitions <= 0) {
      throw new Error('Stress test repetitions must be a positive integer.')
    }

    const responseTimeoutMs = options.responseTimeoutMs ?? DEFAULT_STREAM_RESPONSE_TIMEOUT_MS
    const commandsForIteration = (iteration: number) => {
      switch (scenario) {
        case 'simple_ack':
        case 'status_free':
          return ['G4 P0.001']
        case 'pen_toggle':
          return [iteration % 2 === 0 ? 'M3 S700' : 'M3 S575']
        case 'zero_motion':
          return ['G91', 'G1 X0 F1000', 'G90']
        default:
          return ['G4 P0.001']
      }
    }

    const startRxChunkCount = this.rxChunkTrace.length
    let commandsSent = 0
    let okCount = 0
    let errorCount = 0
    let timeouts = 0
    const latencies: number[] = []

    this.strictAckStreaming = true
    this.updateMachine({
      status: `Strict Ack stress test: ${scenario}`,
      streaming: {
        ...this.getStreamingState(),
        mode: 'sync',
        sent_count: 0,
        acked_count: 0,
        ok_count: 0,
        error_count: 0,
        streaming_active: true,
      },
    })

    try {
      for (let iteration = 0; iteration < repetitions; iteration += 1) {
        for (const command of commandsForIteration(iteration)) {
          const startedAt = nowSeconds()
          try {
            await this.sendLineAndWaitForAck(command, commandsSent + 1, responseTimeoutMs)
            commandsSent += 1
            okCount += 1
            latencies.push(Math.max(0, Math.round((nowSeconds() - startedAt) * 1000)))
          } catch (error) {
            commandsSent += 1
            if (String(error).includes('timeout')) {
              timeouts += 1
            } else {
              errorCount += 1
            }
            throw error
          }
        }
      }
    } finally {
      this.strictAckStreaming = false
      this.updateMachine({
        streaming: {
          ...this.getStreamingState(),
          sent_count: commandsSent,
          acked_count: okCount,
          ok_count: okCount,
          error_count: errorCount,
          streaming_active: false,
        },
      })
    }

    const stressChunks = this.rxChunkTrace.slice(startRxChunkCount)
    const partialChunks = stressChunks.filter((chunk) => chunk.remainingPartial)
    const partialOCount = stressChunks.filter((chunk) => chunk.remainingPartial === 'o' || chunk.raw === 'o').length
    const totalLatency = latencies.reduce((total, latency) => total + latency, 0)

    return {
      scenario,
      repetitions,
      commandsSent,
      okCount,
      errorCount,
      partialChunkCount: partialChunks.length,
      partialOCount,
      timeouts,
      maxAckLatencyMs: latencies.length ? Math.max(...latencies) : 0,
      averageAckLatencyMs: latencies.length ? Math.round(totalLatency / latencies.length) : 0,
      statusPollingDisabled: true,
    }
  }

  async applyConfig(appConfig: AppConfig, settings: SettingsState) {
    const xSteps = (200 * 16) / 360
    const ySteps = (200 * 16) / 360
    return this.sendCommands(
      [
        '$X',
        '$30=1000',
        '$31=0',
        '$32=0',
        '$22=0',
        '$20=0',
        '$21=0',
        `$100=${xSteps.toFixed(6)}`,
        `$110=${settings.xMaxFeed.toFixed(3)}`,
        `$120=${settings.xAcceleration.toFixed(3)}`,
        '$130=100000',
        `$101=${ySteps.toFixed(6)}`,
        `$111=${settings.yMaxFeed.toFixed(3)}`,
        `$121=${settings.yAcceleration.toFixed(3)}`,
        '$131=90',
        '$102=80.000',
        '$112=500.000',
        '$122=50.000',
        '$132=10',
        'G21',
        'G90',
      ],
      `GRBL settings applied at ${appConfig.ballDiameterMm.toFixed(2)} mm ball profile`,
    )
  }

  async penUp(settings: SettingsState) {
    const commands = ['$X', ...buildPenCommands(settings.penUpS, settings.penUpDwellMs)]
    const result = await this.sendCommands(commands, 'Pen up')
    this.updateMachine({ current_servo_s: settings.penUpS, status: 'Pen up' })
    return result
  }

  async penDown(settings: SettingsState) {
    const commands = ['$X', ...buildPenCommands(settings.penDownS, settings.penDownDwellMs)]
    const result = await this.sendCommands(commands, 'Pen down')
    this.updateMachine({ current_servo_s: settings.penDownS, status: 'Pen down' })
    return result
  }

  async goHome(settings: SettingsState) {
    const commands = [
      '$X',
      ...buildPenCommands(settings.penUpS, settings.penUpDwellMs),
      'G21',
      'G90',
      'G0 X0.0000 Y0.0000',
    ]
    const result = await this.sendCommands(commands, 'Returned to X0 Y0 with pen up')
    this.updateMachine({
      current_position_x: 0,
      current_position_y: 0,
      machine_position_trusted: true,
    })
    return result
  }

  async jog(axis: 'X' | 'Y', degrees: number, feed: number) {
    const result = await this.sendCommands(
      ['$X', 'G21', 'G91', formatAxisMove(axis, degrees, feed), 'G4 P0.010', 'G90'],
      `Jogged ${axis}${degrees.toFixed(3)}`,
    )
    this.updateMachine({
      current_position_x: axis === 'X' ? (this.machine.current_position_x ?? 0) + degrees : this.machine.current_position_x,
      current_position_y: axis === 'Y' ? (this.machine.current_position_y ?? 0) + degrees : this.machine.current_position_y,
      machine_position_trusted: Boolean(this.machine.machine_position_trusted || this.machine.calibrated),
    })
    return result
  }

  async zeroAndMarkCalibrated() {
    const result = await this.sendCommands(['$X', 'G21', 'G92 X0 Y0', 'G90', '$1=255'], 'Origin set and calibrated')
    this.updateMachine({
      calibrated: true,
      machine_position_trusted: true,
      emergency_stopped: false,
      motor_hold_enabled: true,
      current_servo_s: 575,
      current_position_x: 0,
      current_position_y: 0,
    })
    return result
  }

  async clearCalibrated() {
    await this.stopYLoop()
    const result = await this.sendCommands(['$1=0'], 'Calibration cleared')
    this.updateMachine({
      calibrated: false,
      machine_position_trusted: false,
      motor_hold_enabled: false,
    })
    return result
  }

  async applyStepperHoldPolicy() {
    const holdValue = this.machine.calibrated ? 255 : 0
    const result = await this.sendCommands([`$1=${holdValue}`], `Stepper hold policy applied: $1=${holdValue}`)
    this.updateMachine({ motor_hold_enabled: holdValue === 255 })
    return result
  }

  async runGcode(lines: string[], callbacks: RunCallbacks = {}) {
    const streamableLines = lines
      .map((line) => sanitizeGcodeLineForStreaming(line))
      .filter((line): line is string => Boolean(line))
    if (!streamableLines.length) {
      throw new Error('Generate a job before starting the plotter.')
    }

    const responseTimeoutMs = callbacks.responseTimeoutMs ?? DEFAULT_STREAM_RESPONSE_TIMEOUT_MS
    const streamingMode = callbacks.streamingMode ?? 'sync'
    const pendingCommands = this.streamPendingCommands
    pendingCommands.length = 0
    let sentCount = 0
    let ackedCount = 0
    let pendingBufferChars = 0
    let recentSerializedBarrier = false

    this.stopRequested = false
    this.updateMachine({
      running: true,
      paused: false,
      job_state: 'running',
      status: 'Streaming G-code',
      progress_done: 0,
      progress_total: streamableLines.length,
      run_started_at: Date.now() / 1000,
      job_started_at: Date.now() / 1000,
      run_finished_at: null,
      job_finished_at: null,
      pause_started_at: null,
      paused_duration_seconds: 0,
      job_elapsed_seconds: 0,
      current_gcode_line: 0,
      last_timeout_debug: null,
      streaming: {
        ...this.getStreamingState(),
        mode: streamingMode,
        current_line: 0,
        sent_count: 0,
        acked_count: 0,
        ok_count: 0,
        error_count: 0,
        pending_buffer_chars: 0,
        pending_commands: 0,
        total_lines: streamableLines.length,
        streaming_active: true,
      },
    })

    try {
      await this.sendLineAndWait('$X')
      if (streamingMode === 'sync') {
        this.strictAckStreaming = true
        try {
          for (let index = 0; index < streamableLines.length; index += 1) {
            while (this.machine.paused && !this.stopRequested) {
              await sleep(50)
            }
            if (this.stopRequested) {
              throw new Error('Stop requested')
            }

            const command = streamableLines[index]
            const result = await this.sendLineAndWaitForAck(command, index + 1, responseTimeoutMs)
            sentCount = index + 1
            ackedCount = index + 1
            pendingBufferChars = 0
            this.updateMachine({
              progress_done: ackedCount,
              current_gcode_line: index + 1,
              streaming: {
                ...this.getStreamingState(),
                mode: streamingMode,
                current_line: index + 1,
                pending_buffer_chars: 0,
                pending_commands: 0,
                sent_count: sentCount,
                acked_count: ackedCount,
                ok_count: ackedCount,
                total_lines: streamableLines.length,
                streaming_active: true,
                last_response_age_sec: 0,
              },
            })
            callbacks.onProgress?.(ackedCount, streamableLines.length, result.command)
            const ackDelayMs = callbacks.ackDelayMs ?? 0
            if (ackDelayMs > 0) {
              await sleep(ackDelayMs)
            }
          }
        } finally {
          this.strictAckStreaming = false
        }

        this.updateMachine({
          running: false,
          paused: false,
          job_state: 'completed',
          status: 'Job complete',
          run_finished_at: Date.now() / 1000,
          job_finished_at: Date.now() / 1000,
          current_gcode_line: streamableLines.length,
          progress_done: streamableLines.length,
          streaming: {
            ...this.getStreamingState(),
            current_line: streamableLines.length,
            pending_buffer_chars: 0,
            pending_commands: 0,
            streaming_active: false,
          },
        })
        pendingCommands.length = 0
        this.updatePendingQueueState()
        return {
          command: 'RUN GCODE',
          response: `Streamed ${streamableLines.length} G-code lines.`,
          lines: [],
        }
      }

      while (sentCount < streamableLines.length || pendingCommands.length > 0) {
        while (sentCount < streamableLines.length) {
          while (this.machine.paused && !this.stopRequested) {
            await sleep(50)
          }
          if (this.stopRequested) {
            throw new Error('Stop requested')
          }

          const command = streamableLines[sentCount]
          const serialized = isSerializedCommand(command)
          const bytes = textEncoder.encode(`${command}\n`).length
          if (pendingCommands.length > 0 && pendingBufferChars + bytes > GRBL_RX_BUFFER_SIZE) {
            break
          }
          if (recentSerializedBarrier && pendingCommands.length > 0) {
            break
          }
          if (serialized && pendingCommands.length > 0) {
            break
          }

          const pending: PendingCommand = {
            transactionId: null,
            lineNumber: sentCount + 1,
            command,
            normalizedCommand: normalizeCommand(command),
            bytes,
            sentAt: Date.now() / 1000,
            acknowledgedAt: null,
            response: null,
            expectedStateChange: buildExpectedStateChange(command),
            desyncStatusMatches: 0,
            holdObserved: false,
          }
          await this.writeRaw(`${command}\n`)
          pendingCommands.push(pending)
          this.recordSentCommand(pending.lineNumber, pending.command)
          this.pushStreamDebugEvent('send', `L${pending.lineNumber} ${pending.normalizedCommand}`)
          pendingBufferChars += bytes
          sentCount += 1
          recentSerializedBarrier = serialized

          this.updateMachine({
            current_gcode_line: ackedCount,
            streaming: {
              ...this.getStreamingState(),
              mode: streamingMode,
              current_line: ackedCount,
              pending_buffer_chars: pendingBufferChars,
              pending_commands: pendingCommands.length,
              sent_count: sentCount,
              acked_count: ackedCount,
              total_lines: streamableLines.length,
              streaming_active: true,
            },
          })
          this.updatePendingQueueState()

          if (serialized) {
            break
          }
        }

        if (!pendingCommands.length) {
          continue
        }

        let response: GrblCommandResponse | null = null
        let responseDeadline = Date.now() + responseTimeoutMs
        while (!response) {
          if (this.machine.paused && !this.stopRequested) {
            responseDeadline = Date.now() + responseTimeoutMs
            await sleep(100)
            continue
          }
          const remainingMs = responseDeadline - Date.now()
          if (remainingMs <= 0) {
            const statusLine = await this.queryStatus(STATUS_QUERY_TIMEOUT_MS)
            const holdStatus = statusLine ? parseGrblStatus(statusLine) : null
            if (statusLine && holdStatus?.state?.startsWith('Hold')) {
              for (const pending of pendingCommands) {
                pending.holdObserved = true
              }
              this.updateMachine({
                paused: true,
                job_state: this.machine.running ? 'paused' : this.machine.job_state,
                pause_started_at: this.machine.pause_started_at ?? Date.now() / 1000,
                status: holdStatus.state,
              })
              responseDeadline = Date.now() + responseTimeoutMs
              continue
            }
            throw this.buildTimeoutError(
              pendingCommands,
              ackedCount,
              responseTimeoutMs,
              statusLine,
            )
          }

          try {
            response = await this.waitForCommandResponse(Math.min(ACK_SILENCE_STATUS_PROBE_MS, remainingMs))
          } catch {
            const statusLine = await this.queryStatus(STATUS_QUERY_TIMEOUT_MS)
            const holdStatus = statusLine ? parseGrblStatus(statusLine) : null
            if (statusLine && holdStatus?.state?.startsWith('Hold')) {
              for (const pending of pendingCommands) {
                pending.holdObserved = true
              }
              this.updateMachine({
                paused: true,
                job_state: this.machine.running ? 'paused' : this.machine.job_state,
                pause_started_at: this.machine.pause_started_at ?? Date.now() / 1000,
                status: holdStatus.state,
              })
              responseDeadline = Date.now() + responseTimeoutMs
              continue
            }
          }
        }

        if (!response) {
          continue
        }

        if (response.kind === 'startup' || response.kind === 'message') {
          continue
        }

        const oldestPending = pendingCommands.shift()
        if (!oldestPending) {
          continue
        }

        oldestPending.acknowledgedAt = response.receivedAt
        oldestPending.response = response.line
        this.pushStreamDebugEvent(response.kind, `L${oldestPending.lineNumber} ${oldestPending.command} -> ${response.line}`)
        pendingBufferChars = Math.max(0, pendingBufferChars - oldestPending.bytes)
        recentSerializedBarrier = pendingCommands.some((pending) => isSerializedCommand(pending.command))
        this.updatePendingQueueState()

        if (response.kind === 'ok') {
          ackedCount += 1
          this.updateMachine({
            progress_done: ackedCount,
            current_gcode_line: oldestPending.lineNumber,
            streaming: {
              ...this.getStreamingState(),
              mode: streamingMode,
              current_line: oldestPending.lineNumber,
              pending_buffer_chars: pendingBufferChars,
              pending_commands: pendingCommands.length,
              sent_count: sentCount,
              acked_count: ackedCount,
              ok_count: ackedCount,
              total_lines: streamableLines.length,
              streaming_active: true,
              last_response_age_sec: 0,
            },
          })
          callbacks.onProgress?.(ackedCount, streamableLines.length, oldestPending.command)
          continue
        }

        const errorCount = (this.getStreamingState().error_count ?? 0) + 1
        this.updateMachine({
          streaming: {
            ...this.getStreamingState(),
            error_count: errorCount,
            pending_buffer_chars: pendingBufferChars,
            pending_commands: pendingCommands.length,
          },
        })
        throw new Error(`GRBL ${response.kind} on line ${oldestPending.lineNumber} while executing "${oldestPending.command}": ${response.line}`)
      }

      await this.queryStatus().catch(() => null)
      this.updateMachine({
        running: false,
        paused: false,
        job_state: 'completed',
        status: 'Job complete',
        run_finished_at: Date.now() / 1000,
        job_finished_at: Date.now() / 1000,
        current_gcode_line: streamableLines.length,
        progress_done: streamableLines.length,
        streaming: {
          ...this.getStreamingState(),
          current_line: streamableLines.length,
          pending_buffer_chars: 0,
          pending_commands: 0,
          streaming_active: false,
        },
      })
      pendingCommands.length = 0
      this.updatePendingQueueState()
      return {
        command: 'RUN GCODE',
        response: `Streamed ${streamableLines.length} G-code lines.`,
        lines: [],
      }
    } catch (error) {
      const userStopRequested = this.stopRequested
      this.stopRequested = true
      const failedStatus = normalizeErrorMessage(error)
      if (this.machine.last_timeout_debug) {
        this.pushStreamDebugEvent('POST_TIMEOUT_LISTEN_START', `${POST_TIMEOUT_OBSERVATION_MS}ms`)
        await sleep(POST_TIMEOUT_OBSERVATION_MS)
        this.pushStreamDebugEvent('POST_TIMEOUT_LISTEN_DONE', `${POST_TIMEOUT_OBSERVATION_MS}ms`)
      }
      const timeoutDebug = this.machine.last_timeout_debug ?? null
      const preserveConnection = Boolean(
        timeoutDebug
        && ['TIMEOUT_INVALID_COMPLETE_LINE', 'TIMEOUT_PARTIAL_LINE', 'TIMEOUT_AFTER_COMPLETE_STATUS', 'TIMEOUT_AFTER_COMPLETE_OK_BUT_NOT_RESOLVED'].includes(
          String((timeoutDebug as Record<string, unknown>).failure_class ?? ''),
        )
        && this.activePort
        && this.portReader
        && this.portWriter
        && !userStopRequested,
      )
      if (!preserveConnection) {
        await this.closeSerialPort()
      } else {
        pendingCommands.length = 0
        this.streamPendingCommands.length = 0
        pendingBufferChars = 0
        this.updatePendingQueueState()
      }
      this.updateMachine({
        ...(preserveConnection ? this.getMachineState() : buildDisconnectedMachineState()),
        connected: preserveConnection,
        running: false,
        paused: preserveConnection,
        job_state: userStopRequested ? 'stopped' : 'failed',
        status: preserveConnection ? `${failedStatus} Connection kept open for recovery.` : failedStatus,
        run_finished_at: Date.now() / 1000,
        job_finished_at: Date.now() / 1000,
        progress_done: ackedCount,
        progress_total: streamableLines.length,
        current_gcode_line: ackedCount,
        last_timeout_debug: timeoutDebug,
        streaming: {
          ...this.getStreamingState(),
          pending_buffer_chars: pendingBufferChars,
          pending_commands: pendingCommands.length,
          sent_count: sentCount,
          acked_count: ackedCount,
          total_lines: streamableLines.length,
          streaming_active: false,
        },
      })
      throw error
    }
  }

  async pause() {
    if (!this.machine.running) {
      throw new Error('No active job is running.')
    }
    await this.writeRaw('!')
    this.updateMachine({
      paused: true,
      job_state: 'paused',
      pause_started_at: Date.now() / 1000,
      status: 'Pause requested',
    })
    return {
      command: 'PAUSE',
      response: 'Feed hold requested.',
      lines: [],
    }
  }

  async resume() {
    if (!this.machine.running) {
      throw new Error('No active job is running.')
    }
    const pausedStartedAt = this.machine.pause_started_at
    const pausedExtra = pausedStartedAt == null ? 0 : Math.max(0, Date.now() / 1000 - pausedStartedAt)
    await this.writeRaw('~')
    this.updateMachine({
      paused: false,
      job_state: 'running',
      pause_started_at: null,
      paused_duration_seconds: (this.machine.paused_duration_seconds ?? 0) + pausedExtra,
      status: 'Resumed',
    })
    return {
      command: 'RESUME',
      response: 'Cycle start requested.',
      lines: [],
    }
  }

  async stop() {
    if (!this.activePort) {
      throw new Error('Connect plotter first.')
    }
    this.stopRequested = true
    this.yLoopAbortController?.abort()
    await this.writeRaw('\x18')
    await sleep(250)
    await this.closeSerialPort()
    this.updateMachine({
      ...buildDisconnectedMachineState(),
      emergency_stopped: true,
      status: 'Soft reset sent - calibration cleared',
    })
    return {
      command: 'CTRL-X RESET',
      response: 'Soft reset sent.',
      lines: [],
    }
  }

  async startYLoop(settings: SettingsState) {
    if (!this.machine.connected) {
      throw new Error('Connect the plotter before starting the Y loop test.')
    }
    if (!this.machine.calibrated) {
      throw new Error('Calibrate the plotter before starting the Y loop test.')
    }
    if (this.machine.running) {
      throw new Error('Stop the active print before starting the Y loop test.')
    }
    if (this.machine.y_loop_test?.enabled) {
      throw new Error('Y loop test is already running.')
    }

    const controller = new AbortController()
    this.yLoopAbortController = controller
    this.updateMachine({
      y_loop_test: {
        enabled: true,
        center_y: 0,
        distance: settings.yLoopDistance,
        feedrate: settings.yLoopFeedrate,
        dwell_sec: settings.yLoopDwellSec,
        phase: 'running',
        cycles_completed: 0,
      },
      status: 'Y loop test running',
    })

    void (async () => {
      let cyclesCompleted = 0
      const halfDistance = settings.yLoopDistance / 2
      try {
        while (!controller.signal.aborted) {
          await this.sendCommands(['$X', 'G21', 'G91', `G1 Y${halfDistance.toFixed(6)} F${settings.yLoopFeedrate.toFixed(3)}`, 'G90'])
          await sleep(settings.yLoopDwellSec * 1000)
          if (controller.signal.aborted) break
          await this.sendCommands(['$X', 'G21', 'G91', `G1 Y${(-halfDistance).toFixed(6)} F${settings.yLoopFeedrate.toFixed(3)}`, 'G90'])
          await sleep(settings.yLoopDwellSec * 1000)
          cyclesCompleted += 1
          this.updateMachine({
            y_loop_test: {
              enabled: true,
              center_y: 0,
              distance: settings.yLoopDistance,
              feedrate: settings.yLoopFeedrate,
              dwell_sec: settings.yLoopDwellSec,
              phase: 'running',
              cycles_completed: cyclesCompleted,
            },
          })
        }
      } catch (error) {
        this.updateMachine({ status: normalizeErrorMessage(error) })
      } finally {
        this.updateMachine({
          y_loop_test: {
            enabled: false,
            center_y: 0,
            distance: settings.yLoopDistance,
            feedrate: settings.yLoopFeedrate,
            dwell_sec: settings.yLoopDwellSec,
            phase: 'idle',
            cycles_completed: cyclesCompleted,
          },
          status: 'Y loop test stopped',
        })
      }
    })()

    return {
      command: 'START Y LOOP TEST',
      response: 'Y loop test started.',
      lines: [],
    }
  }

  async stopYLoop() {
    if (!this.machine.y_loop_test?.enabled) {
      return {
        command: 'STOP Y LOOP TEST',
        response: 'Y loop test already stopped.',
        lines: [],
      }
    }
    this.yLoopAbortController?.abort()
    this.yLoopAbortController = null
    this.updateMachine({
      y_loop_test: {
        ...(this.machine.y_loop_test ?? buildDisconnectedMachineState().y_loop_test!),
        enabled: false,
        phase: 'idle',
      },
      status: 'Y loop test stopped',
    })
    return {
      command: 'STOP Y LOOP TEST',
      response: 'Y loop test stopped.',
      lines: [],
    }
  }
}

export function createBrowserMachineState() {
  return buildDisconnectedMachineState()
}
