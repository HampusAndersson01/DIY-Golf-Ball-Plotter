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
}

type BrowserMachineState = MachineState & {
  current_servo_s: number
}

const textEncoder = new TextEncoder()

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
    pause_started_at: null,
    paused_duration_seconds: 0,
    current_gcode_line: 0,
    current_path_id: null,
    current_preview_point_index: 0,
    current_servo_s: 575,
    current_position_x: 0,
    current_position_y: 0,
    motor_hold_enabled: false,
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

export class GrblWebSerialService {
  private activePort: SerialPort | null = null
  private isConnecting = false
  private readBuffer = ''
  private stopRequested = false
  private machine: BrowserMachineState = buildDisconnectedMachineState()
  private yLoopAbortController: AbortController | null = null

  getMachineState() {
    return structuredClone(this.machine)
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
    if (!this.activePort) {
      throw new Error('Connect plotter first.')
    }
    return this.activePort
  }

  private async writeRaw(payload: string) {
    const port = this.ensureConnectedPort()
    const writer = port.writable?.getWriter()
    if (!writer) {
      throw new Error('Selected serial port is not writable.')
    }
    try {
      await writer.write(textEncoder.encode(payload))
    } finally {
      writer.releaseLock()
    }
  }

  private async readUntil(predicate: (line: string, lines: string[]) => boolean, timeoutMs: number) {
    const port = this.ensureConnectedPort()
    const reader = port.readable?.getReader()
    if (!reader) {
      throw new Error('Selected serial port is not readable.')
    }

    const lines: string[] = []
    let matched = false
    let timedOut = false
    const timer = globalThis.setTimeout(() => {
      timedOut = true
      void reader.cancel().catch(() => {})
    }, timeoutMs)

    try {
      while (true) {
        const { value, done } = await reader.read()
        if (done) break
        if (!value) continue
        this.readBuffer += new TextDecoder().decode(value, { stream: true })
        const extracted = extractLines(this.readBuffer)
        this.readBuffer = extracted.remainder
        for (const line of extracted.lines) {
          lines.push(line)
          if (predicate(line, lines)) {
            matched = true
            return { matched, timedOut, lines }
          }
        }
      }
    } finally {
      globalThis.clearTimeout(timer)
      reader.releaseLock()
    }

    return { matched, timedOut, lines }
  }

  private async readFor(timeoutMs: number) {
    return this.readUntil(() => false, timeoutMs)
  }

  private async sendLineAndWait(command: string, timeoutMs = 4000) {
    await this.writeRaw(command.endsWith('\n') ? command : `${command}\n`)
    const result = await this.readUntil(
      (line) => line === 'ok' || line.startsWith('error:') || line.startsWith('ALARM:'),
      timeoutMs,
    )

    const terminal = result.lines.find((line) => line === 'ok' || line.startsWith('error:') || line.startsWith('ALARM:')) ?? ''
    if (!terminal) {
      throw new Error(`GRBL did not respond to "${command}".`)
    }
    if (terminal.startsWith('error:') || terminal.startsWith('ALARM:')) {
      throw new Error(terminal)
    }
    return {
      command,
      response: result.lines.join('\n') || 'ok',
      lines: result.lines,
    }
  }

  private async queryStatus() {
    await this.writeRaw('?')
    const result = await this.readUntil((line) => line.startsWith('<'), 1500)
    const statusLine = result.lines.find((line) => line.startsWith('<')) ?? null
    if (!statusLine) {
      return null
    }

    const statusMatch = statusLine.match(/^<([^|>]+)/)
    const positionMatch = statusLine.match(/MPos:([-0-9.]+),([-0-9.]+)/)

    this.updateMachine({
      status: statusMatch?.[1] ?? statusLine,
      current_position_x: positionMatch ? Number(positionMatch[1]) : this.machine.current_position_x,
      current_position_y: positionMatch ? Number(positionMatch[2]) : this.machine.current_position_y,
      streaming: {
        ...this.getStreamingState(),
        last_grbl_status: statusLine,
        last_response_age_sec: 0,
      },
    })

    return statusLine
  }

  private async performHandshake() {
    await this.writeRaw('\r\n\r\n')
    await sleep(250)
    const startup = await this.readFor(1500)
    const startupLines = startup.lines
    if (startupLines.some((line) => /grbl/i.test(line))) {
      this.updateMachine({ status: 'Connected to GRBL' })
      await this.queryStatus().catch(() => null)
      return startupLines
    }

    await this.writeRaw('?\n')
    const statusProbe = await this.readUntil((line) => line.startsWith('<'), 1500)
    if (statusProbe.lines.some((line) => line.startsWith('<'))) {
      this.updateMachine({ status: 'Connected to GRBL' })
      return [...startupLines, ...statusProbe.lines]
    }

    throw new Error('GRBL does not respond. Select the Arduino/GRBL controller and try again.')
  }

  private async closePort(port: SerialPort | null) {
    if (!port) return
    try {
      await port.close()
    } catch {
      // Ignore close failures during cleanup.
    }
  }

  async connect() {
    if (this.isConnecting) {
      throw new Error('Connection already in progress')
    }
    if (this.activePort) {
      throw new Error('The selected serial port is already open.')
    }

    this.ensureSerialSupport()
    this.isConnecting = true
    let port: SerialPort | null = null

    try {
      port = await navigator.serial!.requestPort({
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
      await this.performHandshake()

      this.stopRequested = false
      this.updateMachine({
        connected: true,
        emergency_stopped: false,
        status: 'Connected to GRBL',
      })

      return port
    } catch (error) {
      this.activePort = null
      this.readBuffer = ''
      await this.closePort(port)
      this.updateMachine(buildDisconnectedMachineState())
      throw new Error(normalizeErrorMessage(error), { cause: error })
    } finally {
      this.isConnecting = false
    }
  }

  async disconnect() {
    this.stopRequested = true
    this.yLoopAbortController?.abort()
    await this.closePort(this.activePort)
    this.activePort = null
    this.readBuffer = ''
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

    this.stopRequested = false
    this.updateMachine({
      running: true,
      paused: false,
      status: 'Streaming G-code',
      progress_done: 0,
      progress_total: streamableLines.length,
      run_started_at: Date.now() / 1000,
      pause_started_at: null,
      streaming: {
        ...this.getStreamingState(),
        mode: 'sync',
        current_line: 0,
        sent_count: 0,
        acked_count: 0,
        total_lines: streamableLines.length,
        streaming_active: true,
      },
    })

    try {
      await this.sendLineAndWait('$X')
      for (let index = 0; index < streamableLines.length; index += 1) {
        while (this.machine.paused && !this.stopRequested) {
          await sleep(50)
        }
        if (this.stopRequested) {
          throw new Error('Stop requested')
        }

        const line = streamableLines[index]
        await this.sendLineAndWait(line, 20000)
        this.updateMachine({
          progress_done: index + 1,
          current_gcode_line: index + 1,
          streaming: {
            ...this.getStreamingState(),
            current_line: index + 1,
            sent_count: index + 1,
            acked_count: index + 1,
            total_lines: streamableLines.length,
            streaming_active: true,
          },
        })
        callbacks.onProgress?.(index + 1, streamableLines.length, line)
      }

      await this.queryStatus().catch(() => null)
      this.updateMachine({
        running: false,
        paused: false,
        status: 'Job complete',
        run_finished_at: Date.now() / 1000,
        streaming: {
          ...this.getStreamingState(),
          streaming_active: false,
        },
      })
      return {
        command: 'RUN GCODE',
        response: `Streamed ${streamableLines.length} G-code lines.`,
        lines: [],
      }
    } catch (error) {
      this.updateMachine({
        running: false,
        paused: false,
        status: this.stopRequested ? 'Stop requested' : normalizeErrorMessage(error),
        streaming: {
          ...this.getStreamingState(),
          streaming_active: false,
        },
      })
      if (!this.stopRequested) {
        throw error
      }
      return {
        command: 'RUN GCODE',
        response: 'Stop requested',
        lines: [],
      }
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
    await this.writeRaw('~')
    this.updateMachine({
      paused: false,
      pause_started_at: null,
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
    await this.readFor(500).catch(() => null)
    this.updateMachine({
      running: false,
      paused: false,
      calibrated: false,
      machine_position_trusted: false,
      emergency_stopped: true,
      status: 'Soft reset sent - calibration cleared',
      y_loop_test: {
        ...(this.machine.y_loop_test ?? buildDisconnectedMachineState().y_loop_test!),
        enabled: false,
        phase: 'idle',
      },
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
