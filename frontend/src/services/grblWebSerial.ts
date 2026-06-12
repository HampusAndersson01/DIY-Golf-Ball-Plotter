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
}

type BrowserMachineState = MachineState & {
  current_servo_s: number
}

type PendingCommand = {
  lineNumber: number
  command: string
  bytes: number
  sentAt: number
  acknowledgedAt: number | null
  response: string | null
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
  x: number | null
  y: number | null
}

type PortDiagnostics = {
  portOpen: boolean
  readerActive: boolean
  writerActive: boolean
}

const textEncoder = new TextEncoder()
const textDecoder = new TextDecoder()
const GRBL_RX_BUFFER_SIZE = 128
const GRBL_PLANNER_BUFFER_SIZE = 15
const DEFAULT_STREAM_RESPONSE_TIMEOUT_MS = 20_000
const STATUS_QUERY_TIMEOUT_MS = 250
const RECENT_RESPONSE_LIMIT = 20

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

function extractLines(buffer: string) {
  const parts = buffer.split(/\r?\n/)
  return {
    lines: parts.slice(0, -1).map((line) => line.trim()).filter(Boolean),
    remainder: parts.at(-1) ?? '',
  }
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

  return {
    raw: line,
    state: parts[0] ?? null,
    plannerBufferFree: Number.isFinite(plannerBufferFree) ? plannerBufferFree : null,
    serialRxFree: Number.isFinite(serialRxFree) ? serialRxFree : null,
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
  private commandResponseQueue: GrblCommandResponse[] = []
  private commandWaiters: Array<{
    resolve: (response: GrblCommandResponse) => void
    reject: (error: Error) => void
  }> = []
  private statusWaiters: Array<{
    resolve: (line: string) => void
    reject: (error: Error) => void
  }> = []
  private recentGrblLines: string[] = []
  private lastResponseAt: number | null = null

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

  async getPreviouslyApprovedPorts() {
    if (!navigator.serial) return []
    return navigator.serial.getPorts()
  }

  private updateMachine(patch: Partial<BrowserMachineState>) {
    this.machine = mergeMachineState(this.machine, patch)
    return this.getMachineState()
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
    if (this.recentGrblLines.length > RECENT_RESPONSE_LIMIT) {
      this.recentGrblLines.shift()
    }
    this.lastResponseAt = Date.now() / 1000
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
    this.updateMachine({
      status: parsedStatus?.state ?? line,
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

  private handleIncomingLine(line: string) {
    this.recordGrblLine(line)
    if (line.startsWith('<')) {
      this.handleIncomingStatusLine(line)
      return
    }
    this.enqueueCommandResponse(this.classifyCommandResponse(line))
  }

  private async startReadLoop() {
    if (!this.portReader) return
    const reader = this.portReader
    this.readLoopStopped = false
    this.readLoopPromise = (async () => {
      try {
        while (!this.readLoopStopped) {
          const { value, done } = await reader.read()
          if (done) break
          if (!value) continue
          this.readBuffer += textDecoder.decode(value, { stream: true })
          const extracted = extractLines(this.readBuffer)
          this.readBuffer = extracted.remainder
          for (const line of extracted.lines) {
            this.handleIncomingLine(line)
          }
        }
      } catch (error) {
        if (!this.readLoopStopped) {
          const message = error instanceof Error ? error.message : String(error)
          const failure = new Error(`Serial read loop stopped unexpectedly: ${message}`)
          this.rejectWaiters(this.commandWaiters, failure)
          this.rejectWaiters(this.statusWaiters, failure)
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
    this.recentGrblLines = []
    this.lastResponseAt = null
    this.rejectWaiters(this.commandWaiters, new Error('Serial session reset.'))
    this.rejectWaiters(this.statusWaiters, new Error('Serial session reset.'))
  }

  private async closeSerialPort() {
    const port = this.activePort
    await this.stopReadLoop()
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

  private async writeRaw(payload: string) {
    this.ensureConnectedPort()
    if (!this.portWriter) {
      throw new Error('Selected serial port is not writable.')
    }
    await this.portWriter.write(textEncoder.encode(payload))
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

  private buildTimeoutError(
    pendingCommands: PendingCommand[],
    currentLineNumber: number,
    responseTimeoutMs: number,
    statusLine: string | null,
  ) {
    const timedOutCommand = pendingCommands[0] ?? null
    const lastSentCommand = pendingCommands.at(-1) ?? null
    const timeoutDebug = {
      timeout_ms: responseTimeoutMs,
      last_sent_command: lastSentCommand?.command ?? null,
      last_sent_line: lastSentCommand?.lineNumber ?? null,
      timed_out_command: timedOutCommand?.command ?? null,
      timed_out_line: timedOutCommand?.lineNumber ?? null,
      current_line: currentLineNumber,
      pending_queue_length: pendingCommands.length,
      pending_queue: pendingCommands.map((pending) => ({
        lineNumber: pending.lineNumber,
        command: pending.command,
        bytes: pending.bytes,
        sentAt: pending.sentAt,
      })),
      last_20_received_grbl_lines: [...this.recentGrblLines],
      last_grbl_response: this.recentGrblLines.at(-1) ?? null,
      status_query_response: statusLine,
      port_state: this.buildPortDiagnostics(),
    }
    this.updateMachine({ last_timeout_debug: timeoutDebug })

    const portDiagnostics = this.buildPortDiagnostics()
    const lastResponse = this.recentGrblLines.at(-1) ?? 'none'
    const pendingQueueText = formatPendingQueue(pendingCommands).join(', ') || 'empty'
    const statusText = statusLine ?? 'none'
    return new Error(
      `GRBL communication timeout at line ${timedOutCommand?.lineNumber ?? currentLineNumber} after "${timedOutCommand?.command ?? 'unknown command'}". ` +
      `Last sent="${lastSentCommand?.command ?? 'none'}". Pending=${pendingCommands.length} [${pendingQueueText}]. ` +
      `Last GRBL response="${lastResponse}". Status query="${statusText}". ` +
      `Port open=${portDiagnostics.portOpen} reader=${portDiagnostics.readerActive} writer=${portDiagnostics.writerActive}.`,
    )
  }

  private async sendLineAndWait(command: string, timeoutMs = 4000) {
    await this.writeRaw(command.endsWith('\n') ? command : `${command}\n`)

    while (true) {
      const response = await this.waitForCommandResponse(timeoutMs)
      if (response.kind === 'startup' || response.kind === 'message') {
        continue
      }
      if (response.kind === 'error' || response.kind === 'alarm') {
        throw new Error(response.line)
      }
      return {
        command,
        response: response.line,
        lines: [response.line],
      }
    }
  }

  private async queryStatus(timeoutMs = 1500) {
    await this.writeRaw('?')
    try {
      return await this.waitForStatusLine(timeoutMs)
    } catch {
      return null
    }
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
    const streamableLines = lines.map((line) => line.trim()).filter((line) => line && !line.startsWith(';') && !line.startsWith('('))
    if (!streamableLines.length) {
      throw new Error('Generate a job before starting the plotter.')
    }

    const responseTimeoutMs = callbacks.responseTimeoutMs ?? DEFAULT_STREAM_RESPONSE_TIMEOUT_MS
    const streamingMode = callbacks.streamingMode ?? 'buffered'
    const pendingCommands: PendingCommand[] = []
    let sentCount = 0
    let ackedCount = 0
    let pendingBufferChars = 0
    let recentSerializedBarrier = false

    this.stopRequested = false
    this.updateMachine({
      running: true,
      paused: false,
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
          if (streamingMode === 'sync' && pendingCommands.length > 0) {
            break
          }
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
            lineNumber: sentCount + 1,
            command,
            bytes,
            sentAt: Date.now() / 1000,
            acknowledgedAt: null,
            response: null,
          }
          await this.writeRaw(`${command}\n`)
          pendingCommands.push(pending)
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

          if (streamingMode === 'sync' || serialized) {
            break
          }
        }

        if (!pendingCommands.length) {
          continue
        }

        let response: GrblCommandResponse
        try {
          response = await this.waitForCommandResponse(responseTimeoutMs)
        } catch {
          const statusLine = await this.queryStatus(STATUS_QUERY_TIMEOUT_MS)
          const parsedStatus = statusLine ? parseGrblStatus(statusLine) : null
          const idleWithEmptyBuffers = Boolean(
            parsedStatus?.state === 'Idle'
              && parsedStatus.plannerBufferFree != null
              && parsedStatus.serialRxFree != null
              && parsedStatus.plannerBufferFree >= GRBL_PLANNER_BUFFER_SIZE
              && parsedStatus.serialRxFree >= GRBL_RX_BUFFER_SIZE,
          )
          if (idleWithEmptyBuffers) {
            for (const pending of pendingCommands) {
              pending.acknowledgedAt = Date.now() / 1000
              pending.response = 'ok (recovered from idle status)'
              ackedCount += 1
              callbacks.onProgress?.(ackedCount, streamableLines.length, pending.command)
            }
            pendingCommands.length = 0
            pendingBufferChars = 0
            recentSerializedBarrier = false
            this.updateMachine({
              progress_done: ackedCount,
              current_gcode_line: ackedCount,
              streaming: {
                ...this.getStreamingState(),
                current_line: ackedCount,
                pending_buffer_chars: 0,
                pending_commands: 0,
                sent_count: sentCount,
                acked_count: ackedCount,
                ok_count: ackedCount,
                total_lines: streamableLines.length,
                streaming_active: true,
                last_grbl_status: statusLine,
                last_response_age_sec: 0,
              },
            })
            continue
          }
          throw this.buildTimeoutError(pendingCommands, ackedCount, responseTimeoutMs, statusLine)
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
        pendingBufferChars = Math.max(0, pendingBufferChars - oldestPending.bytes)
        recentSerializedBarrier = pendingCommands.some((pending) => isSerializedCommand(pending.command))

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
      return {
        command: 'RUN GCODE',
        response: `Streamed ${streamableLines.length} G-code lines.`,
        lines: [],
      }
    } catch (error) {
      this.stopRequested = true
      this.updateMachine({
        running: false,
        paused: false,
        status: this.stopRequested ? normalizeErrorMessage(error) : normalizeErrorMessage(error),
        run_finished_at: Date.now() / 1000,
        job_finished_at: Date.now() / 1000,
        streaming: {
          ...this.getStreamingState(),
          pending_buffer_chars: 0,
          pending_commands: 0,
          streaming_active: false,
        },
      })
      await this.closeSerialPort()
      this.updateMachine({
        ...this.machine,
        connected: false,
        running: false,
        paused: false,
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
