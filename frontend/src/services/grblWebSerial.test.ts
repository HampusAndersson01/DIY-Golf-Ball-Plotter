import { beforeEach, describe, expect, it, vi } from 'vitest'

import { BAUD_RATE, GrblWebSerialService, parseGrblSerialChunk } from './grblWebSerial'

function createMockPort() {
  const open = vi.fn().mockResolvedValue(undefined)
  const close = vi.fn().mockResolvedValue(undefined)
  const chunks: Uint8Array[] = []
  const writes: string[] = []
  let onWrite: ((payload: string) => void) | null = null
  let readIndex = 0
  let pendingRead: ((result: { done: false; value: Uint8Array } | { done: true; value: undefined }) => void) | null = null

  const enqueueChunk = (chunk: string) => {
    const encoded = new TextEncoder().encode(chunk)
    if (pendingRead) {
      const resolve = pendingRead
      pendingRead = null
      readIndex += 1
      chunks.push(encoded)
      resolve({ done: false, value: encoded })
      return
    }
    chunks.push(encoded)
  }

  const port: SerialPort = {
    readable: {
      getReader() {
        let cancelled = false
        return {
          async read() {
            if (cancelled) {
              return { done: true, value: undefined }
            }
            const value = chunks[readIndex]
            if (value) {
              readIndex += 1
              return { done: false, value }
            }
            return new Promise((resolve) => {
              pendingRead = resolve
            })
          },
          async cancel() {
            cancelled = true
            if (pendingRead) {
              const resolve = pendingRead
              pendingRead = null
              resolve({ done: true, value: undefined })
            }
          },
          releaseLock() {},
        } as ReadableStreamDefaultReader<Uint8Array>
      },
    } as ReadableStream<Uint8Array>,
    writable: {
      getWriter() {
        return {
          async write(value) {
            const payload = new TextDecoder().decode(value)
            writes.push(payload)
            onWrite?.(payload)
            return undefined
          },
          releaseLock() {},
        } as WritableStreamDefaultWriter<Uint8Array>
      },
    } as WritableStream<Uint8Array>,
    open,
    close,
    getInfo() {
      return { usbVendorId: 0x2341 }
    },
  }

  return {
    port,
    open,
    close,
    pushLine(line: string) {
      enqueueChunk(`${line}\n`)
    },
    pushChunk(chunk: string) {
      enqueueChunk(chunk)
    },
    setOnWrite(handler: (payload: string) => void) {
      onWrite = handler
    },
    writes,
  }
}

function waitForMicrotasks() {
  return new Promise((resolve) => setTimeout(resolve, 0))
}

describe('GrblWebSerialService', () => {
  beforeEach(() => {
    vi.restoreAllMocks()
    if (!('navigator' in globalThis)) {
      Object.defineProperty(globalThis, 'navigator', {
        configurable: true,
        value: {},
      })
    }
  })

  it.each([
    ['ok\\n', ['', 'ok\n'], ['ok']],
    ['ok\\r\\n', ['', 'ok\r\n'], ['ok']],
    ['status then ok', ['', '<Idle|MPos:0.000,0.000,0.000>\nok\n'], ['<Idle|MPos:0.000,0.000,0.000>', 'ok']],
    ['ok then status', ['', 'ok\n<Idle|MPos:0.000,0.000,0.000>\n'], ['ok', '<Idle|MPos:0.000,0.000,0.000>']],
    ['partial ok', ['o', 'k\n'], ['ok']],
    ['multiple ok and status', ['', 'ok\nok\n<Idle|MPos:0.000,0.000,0.000>\nok\n'], ['ok', 'ok', '<Idle|MPos:0.000,0.000,0.000>', 'ok']],
  ])('parses RX chunk case: %s', (_name, chunks, expectedLines) => {
    let buffer = ''
    const parsed: string[] = []
    for (const chunk of chunks) {
      const result = parseGrblSerialChunk(buffer, chunk)
      parsed.push(...result.lines)
      buffer = result.remainder
    }
    expect(parsed).toEqual(expectedLines)
    expect(parsed.filter((line) => line === 'ok')).toHaveLength(expectedLines.filter((line) => line === 'ok').length)
  })

  it('calls requestPort exactly once for each connect click', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.pushLine('<Idle|MPos:0.000,0.000,0.000>')

    const requestPort = vi.fn().mockResolvedValue(mock.port)
    const getPorts = vi.fn().mockResolvedValue([])

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort, getPorts },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    expect(requestPort).toHaveBeenCalledTimes(1)
    expect(mock.open).toHaveBeenCalledWith(expect.objectContaining({ baudRate: BAUD_RATE }))
  })

  it('blocks double connect attempts while a picker is already in progress', async () => {
    let rejectPort: ((reason?: unknown) => void) | undefined
    const deferred = new Promise<SerialPort>((_, reject) => {
      rejectPort = reject
    })
    const requestPort = vi.fn().mockReturnValue(deferred)

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort, getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    const firstConnect = service.connect()

    await expect(service.connect()).rejects.toThrow('Connection already in progress')

    rejectPort?.(new DOMException('cancelled', 'NotFoundError'))
    await expect(firstConnect).rejects.toThrow('Serial port selection was cancelled.')
    expect(requestPort).toHaveBeenCalledTimes(1)
  })

  it('resets the connecting guard after a cancelled picker', async () => {
    const requestPort = vi.fn().mockRejectedValue(new DOMException('cancelled', 'NotFoundError'))

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort, getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()

    await expect(service.connect()).rejects.toThrow('Serial port selection was cancelled.')
    await expect(service.connect()).rejects.toThrow('Serial port selection was cancelled.')
    expect(requestPort).toHaveBeenCalledTimes(2)
  })

  it('stores the selected SerialPort object after a successful connection', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.pushLine('<Idle|MPos:0.000,0.000,0.000>')

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    const port = await service.connect()

    expect(port).toBe(mock.port)
    expect(service.getPort()).toBe(mock.port)
    expect(service.getMachineState().connected).toBe(true)
  })

  it('closes and cleans up the port after a failed handshake', async () => {
    const mock = createMockPort()
    mock.pushLine('usb device ready')

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()

    await expect(service.connect()).rejects.toThrow('GRBL does not respond.')
    expect(mock.close).toHaveBeenCalledTimes(1)
    expect(service.getPort()).toBeNull()
    expect(service.getMachineState().connected).toBe(false)
  })

  it('does not recover a buffered motion command from idle status alone when the ok is missing', async () => {
    const mock = createMockPort()
    const statusReplies = [
      '<Idle|MPos:0.000,0.000,0.000|Bf:15,128>',
      '<Idle|WPos:-15.1000,-14.7000,0.000|Bf:15,128>',
      '<Idle|WPos:-15.1000,-14.7000,0.000|Bf:15,128>',
    ]

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === '?') {
        const status = statusReplies.shift()
        if (status) {
          mock.pushLine(status)
        }
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(
      service.runGcode(['G1 X-15.2271 Y-14.8662'], { responseTimeoutMs: 5, streamingMode: 'buffered' }),
    ).rejects.toThrow('GRBL communication timeout at line 1 after "G1 X-15.2271 Y-14.8662".')
  })

  it('reports a communication timeout when a buffered run gets no ack and no status', async () => {
    const mock = createMockPort()
    let statusReplies = 1

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === '?' && statusReplies > 0) {
        statusReplies -= 1
        mock.pushLine('<Idle|MPos:0.000,0.000,0.000|Bf:15,128>')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(
      service.runGcode(['G1 X-15.2271 Y-14.8662'], { responseTimeoutMs: 5, streamingMode: 'buffered' }),
    ).rejects.toThrow('GRBL communication timeout at line 1 after "G1 X-15.2271 Y-14.8662".')
  })

  it('resolves a strict transaction when ok arrives during the same write turn', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X' || command === 'M3 S700') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await service.runGcode(['M3 S700'], { responseTimeoutMs: 20, streamingMode: 'sync' })
    await waitForMicrotasks()

    const diagnostics = service.getStreamDiagnostics()
    expect(service.getMachineState().progress_done).toBe(1)
    expect(service.getMachineState().streaming?.pending_commands).toBe(0)
    expect(diagnostics.active_transaction).toBeNull()
    expect(diagnostics.transaction_lifecycle_events).toEqual(
      expect.arrayContaining([
        expect.objectContaining({ event: 'TX_CREATE', command: 'M3 S700' }),
        expect.objectContaining({ event: 'RX_OK_FOR_TX', command: 'M3 S700' }),
        expect.objectContaining({ event: 'TX_RESOLVE', command: 'M3 S700' }),
      ]),
    )
  })

  it('does not time out after parser resolves the active M3 transaction from ok', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === 'M3 S700') {
        setTimeout(() => mock.pushLine('ok'), 0)
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await service.runGcode(['M3 S700'], { responseTimeoutMs: 5, streamingMode: 'sync' })
    await new Promise((resolve) => setTimeout(resolve, 15))

    expect(service.getMachineState().status).toBe('Job complete')
    expect(service.getMachineState().last_timeout_debug).toBeNull()
    expect(service.getStreamDiagnostics().active_transaction).toBeNull()
  })

  it('logs an unexpected ok without corrupting the next strict transaction', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X' || command.startsWith('G1 ')) {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()
    mock.pushLine('ok')
    await waitForMicrotasks()

    await service.runGcode(['G1 X1.0000 Y0.0000 F1200.000'], { responseTimeoutMs: 20, streamingMode: 'sync' })

    const diagnostics = service.getStreamDiagnostics()
    expect(diagnostics.unexpected_ok_count).toBe(1)
    expect(diagnostics.transaction_lifecycle_events).toEqual(
      expect.arrayContaining([
        expect.objectContaining({ event: 'UNEXPECTED_OK' }),
        expect.objectContaining({ event: 'RX_OK_FOR_TX', command: 'G1 X1.0000 Y0.0000 F1200.000' }),
      ]),
    )
    expect(service.getMachineState().progress_done).toBe(1)
  })

  it('resolves an active transaction from a partial ok chunk', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command.startsWith('G1 ')) {
        mock.pushChunk('o')
        mock.pushChunk('k\n')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await service.runGcode(['G1 X2.0000 Y0.0000 F1200.000'], { responseTimeoutMs: 20, streamingMode: 'sync' })

    expect(service.getMachineState().progress_done).toBe(1)
    expect(service.getStreamDiagnostics().rx_lines.filter((entry) => entry.line === 'ok')).toHaveLength(2)
  })

  it('classifies a timeout after a dangling o byte as a partial line timeout', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command.startsWith('G1 ')) {
        mock.pushChunk('o')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(service.runGcode(['G1 X2.0000 Y0.0000 F1200.000'], { responseTimeoutMs: 5, streamingMode: 'sync' })).rejects.toThrow(
      'TIMEOUT_PARTIAL_LINE',
    )

    const timeoutDebug = service.getMachineState().last_timeout_debug
    expect(timeoutDebug?.failure_class).toBe('TIMEOUT_PARTIAL_LINE')
    expect(timeoutDebug?.last_raw_fragment).toBe('o')
    expect(timeoutDebug?.current_partial_line_buffer).toBe('o')
    expect(timeoutDebug?.last_complete_parsed_line).toBe('ok')
  })

  it('classifies o followed by CRLF as an invalid complete line, not a missed ok race', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === 'M3 S700') {
        mock.pushLine('ok')
      } else if (command === 'G4 P0.060') {
        mock.pushChunk('o')
        mock.pushChunk('\r\n')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(service.runGcode(['M3 S700', 'G4 P0.060'], { responseTimeoutMs: 5, streamingMode: 'sync' })).rejects.toThrow(
      'TIMEOUT_INVALID_COMPLETE_LINE',
    )

    const timeoutDebug = service.getMachineState().last_timeout_debug
    expect(timeoutDebug?.failure_class).toBe('TIMEOUT_INVALID_COMPLETE_LINE')
    expect(timeoutDebug?.timed_out_command).toBe('G4 P0.060')
    expect(timeoutDebug?.last_complete_parsed_line).toBe('o')
    expect(timeoutDebug?.last_parsed_ack_line).toBe('ok')
    expect(timeoutDebug?.current_partial_line_buffer).toBeNull()
  })

  it('recovers a malformed strict motion acknowledgement when status proves the move completed', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X' || command === 'G90') {
        mock.pushLine('ok')
      } else if (command === 'X1.0000 Y2.0000') {
        mock.pushChunk('o')
        mock.pushChunk('\r\n')
      } else if (command === '?') {
        mock.pushLine('<Idle|WPos:1.000,2.000,0.000|Bf:15,128|FS:0,700>')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await service.runGcode(['G90', 'X1.0000 Y2.0000'], { responseTimeoutMs: 5, streamingMode: 'sync' })

    expect(service.getMachineState().status).toBe('Job complete')
    expect(service.getMachineState().progress_done).toBe(2)
    expect(service.getMachineState().connected).toBe(true)
    expect(service.getMachineState().last_timeout_debug).toBeNull()
    expect(service.getStreamDiagnostics().transaction_lifecycle_events).toEqual(
      expect.arrayContaining([
        expect.objectContaining({ event: 'TX_RECOVER_STATUS_MATCH', command: 'X1.0000 Y2.0000' }),
      ]),
    )
  })

  it('keeps the port open after an invalid complete line timeout so the operator can recover', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === 'X1.0000 Y2.0000') {
        mock.pushChunk('o')
        mock.pushChunk('\r\n')
      } else if (command === '?') {
        mock.pushLine('<Idle|WPos:0.900,2.000,0.000|Bf:15,128|FS:0,700>')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(service.runGcode(['X1.0000 Y2.0000'], { responseTimeoutMs: 5, streamingMode: 'sync' })).rejects.toThrow(
      'TIMEOUT_INVALID_COMPLETE_LINE',
    )

    expect(service.getMachineState().connected).toBe(true)
    expect(service.getPort()).toBe(mock.port)
    expect(service.getMachineState().paused).toBe(true)
    expect(service.getMachineState().status).toContain('Connection kept open for recovery')
    expect(mock.close).not.toHaveBeenCalled()
  })

  it('updates status and resolves active transaction from a multi-line status plus ok chunk', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command.startsWith('G1 ')) {
        mock.pushChunk('<Idle|WPos:1.000,0.000,0.000|Bf:15,128>\nok\n')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await service.runGcode(['G1 X1.0000 Y0.0000 F1200.000'], { responseTimeoutMs: 20, streamingMode: 'sync' })

    expect(service.getMachineState().progress_done).toBe(1)
    expect(service.getMachineState().streaming?.last_grbl_status).toBe('<Idle|WPos:1.000,0.000,0.000|Bf:15,128>')
  })

  it('does not synthesize an acknowledgement for M3 when status confirms the spindle state', async () => {
    const mock = createMockPort()

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()
    const runWritesStart = mock.writes.length

    setTimeout(() => {
      mock.pushLine('<Idle|WPos:16.313,-11.700,-2.000|Bf:15,128|FS:0,700|Ov:100,100,100|A:S>')
    }, 0)

    await expect(service.runGcode(['M3 S700'], { responseTimeoutMs: 5, streamingMode: 'sync' })).rejects.toThrow(
      'GRBL communication timeout at line 1 after "M3 S700".',
    )
    expect(mock.writes.slice(runWritesStart)).not.toContain('?')
    expect(service.getMachineState().connected).toBe(true)
    expect(service.getMachineState().job_state).toBe('failed')
    expect(service.getMachineState().status).toContain('Connection kept open for recovery')
  })

  it('does not synthesize an acknowledgement for motion commands when idle status does not match the target position', async () => {
    const mock = createMockPort()
    const statusReplies = [
      '<Idle|MPos:0.000,0.000,0.000|Bf:15,128>',
      '<Idle|WPos:0.930,0.000,0.000|Bf:15,128>',
      '<Idle|WPos:0.930,0.000,0.000|Bf:15,128>',
    ]

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === '?') {
        const status = statusReplies.shift()
        if (status) {
          mock.pushLine(status)
        }
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(
      service.runGcode(['G1 X1.0000 Y0.0000 F1200.000'], { responseTimeoutMs: 5, streamingMode: 'buffered' }),
    ).rejects.toThrow('GRBL communication timeout at line 1 after "G1 X1.0000 Y0.0000 F1200.000".')
  })

  it('fails a strict modal motion line when status appears applied but ok is missing', async () => {
    const mock = createMockPort()

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()
    const runWritesStart = mock.writes.length

    setTimeout(() => {
      mock.pushLine('<Idle|WPos:-21.375,6.975,-2.000|Bf:15,128|FS:0,700|Ov:100,100,100|A:S>')
    }, 0)

    await expect(service.runGcode(['X-21.3699 Y6.9464'], { responseTimeoutMs: 5, streamingMode: 'sync' })).rejects.toThrow(
      'GRBL communication timeout at line 1 after "X-21.3699 Y6.9464".',
    )
    expect(mock.writes.slice(runWritesStart)).not.toContain('?')
    expect(service.getMachineState().last_timeout_debug?.last_100_parsed_rx_lines).toEqual(
      expect.arrayContaining([
        expect.objectContaining({
          line: '<Idle|WPos:-21.375,6.975,-2.000|Bf:15,128|FS:0,700|Ov:100,100,100|A:S>',
          produced: 'status',
        }),
      ]),
    )
  })

  it('fails a strict dwell when idle status arrives but ok is missing', async () => {
    const mock = createMockPort()

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()
    const runWritesStart = mock.writes.length

    setTimeout(() => {
      mock.pushLine('<Idle|WPos:-10.575,-15.862,-2.000|Bf:15,128|FS:0,575>')
    }, 0)

    await expect(service.runGcode(['G4 P0.030'], { responseTimeoutMs: 100, streamingMode: 'sync' })).rejects.toThrow(
      'GRBL communication timeout at line 1 after "G4 P0.030".',
    )
    expect(mock.writes.slice(runWritesStart)).not.toContain('?')
  })

  it('does not synthesize an acknowledgement when multiple commands are pending and one is M3', async () => {
    const mock = createMockPort()
    const statusReplies = [
      '<Idle|MPos:0.000,0.000,0.000|Bf:15,128>',
      '<Idle|WPos:0.000,0.000,0.000|Bf:15,128|FS:0,700|Ov:100,100,100|A:S>',
      '<Idle|WPos:0.000,0.000,0.000|Bf:15,128|FS:0,700|Ov:100,100,100|A:S>',
    ]

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command === '?') {
        const status = statusReplies.shift()
        if (status) {
          mock.pushLine(status)
        }
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(
      service.runGcode(['G1 X1.0000 Y0.0000 F1200.000', 'M3 S700'], { responseTimeoutMs: 5, streamingMode: 'buffered' }),
    ).rejects.toThrow('GRBL communication timeout')
  })

  it('preserves extra ok lines that arrive in the same serial chunk during buffered streaming', async () => {
    const mock = createMockPort()

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
        return
      }
      if (command.startsWith('G1 ')) {
        return
      }
      if (command === '?') {
        mock.pushLine('<Idle|WPos:0.000,0.000,0.000|Bf:15,128>')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    setTimeout(() => {
      mock.pushLine('ok\nok\nok')
    }, 0)

    await service.runGcode(
      [
        'G1 X-0.6110 Y1.4116 F3000.000',
        'G1 X-0.6120 Y1.4126 F3000.000',
        'G1 X-0.6130 Y1.4136 F3000.000',
      ],
      { responseTimeoutMs: 20, streamingMode: 'buffered' },
    )

    expect(service.getMachineState().progress_done).toBe(3)
    expect(service.getMachineState().status).toBe('Job complete')
  })

  it('does not synthesize calibration command acknowledgements from status', async () => {
    const mock = createMockPort()
    let skippedAckForG92 = false

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === 'G92 X0 Y0' && !skippedAckForG92) {
        skippedAckForG92 = true
      } else if (command !== '?') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(service.zeroAndMarkCalibrated()).rejects.toThrow('GRBL communication timeout')

    expect(service.getMachineState().calibrated).toBe(false)
    expect(mock.writes.filter((payload) => payload === '?')).toHaveLength(2)
  }, 10_000)

  it('releases the serial port after a failed run so the next connect can reopen it', async () => {
    const failing = createMockPort()
    let failingStatusReplies = 1
    failing.pushLine('Grbl 1.1h')
    failing.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        failing.pushLine('ok')
      } else if (command === '?' && failingStatusReplies > 0) {
        failingStatusReplies -= 1
        failing.pushLine('<Idle|MPos:0.000,0.000,0.000|Bf:15,128>')
      }
    })

    const succeeding = createMockPort()
    succeeding.pushLine('Grbl 1.1h')
    succeeding.pushLine('<Idle|MPos:0.000,0.000,0.000>')

    const requestPort = vi.fn()
      .mockResolvedValueOnce(failing.port)
      .mockResolvedValueOnce(succeeding.port)

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort, getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(
      service.runGcode(['G1 X-15.2271 Y-14.8662'], { responseTimeoutMs: 5, streamingMode: 'buffered' }),
    ).rejects.toThrow('GRBL communication timeout')

    await service.connect()

    expect(failing.close).toHaveBeenCalled()
    expect(succeeding.open).toHaveBeenCalledWith(expect.objectContaining({ baudRate: BAUD_RATE }))
    expect(service.getMachineState().connected).toBe(true)
  }, 10_000)

  it('uses Strict Ack mode by default and keeps at most one command in flight on a long simulated run', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X' || command.startsWith('G1 ')) {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    const job = Array.from({ length: 12_000 }, (_, index) => `G1 X${(index / 100).toFixed(4)} Y-1.0000 F1200.000`)
    const runWritesStart = mock.writes.length
    await service.runGcode(job)
    const runWrites = mock.writes.slice(runWritesStart)

    expect(service.getMachineState().streaming?.mode).toBe('sync')
    expect(service.getMachineState().progress_done).toBe(12_000)
    expect(runWrites).not.toContain('?')
    expect(runWrites).toHaveLength(12_001)
    for (const payload of runWrites) {
      expect(payload.endsWith('\n')).toBe(true)
      expect([...payload.matchAll(/\n/g)]).toHaveLength(1)
    }
  }, 120_000)

  it('strips inline and whole-line comments before live streaming while preserving executable commands', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X' || command === 'G1 X1.0000 Y2.0000 F1200.000' || command === 'M3 S700') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    const runWritesStart = mock.writes.length
    await service.runGcode(
      [
        '(debug header)',
        'G1 X1.0000 Y2.0000 F1200.000 ; travel move comment',
        'M3 S700 (pen down annotation)',
        '; trailing footer comment',
      ],
      { responseTimeoutMs: 20, streamingMode: 'sync' },
    )

    expect(mock.writes.slice(runWritesStart)).toEqual(['$X\n', 'G1 X1.0000 Y2.0000 F1200.000\n', 'M3 S700\n'])
    expect(service.getMachineState().progress_total).toBe(2)
    expect(service.getMachineState().progress_done).toBe(2)
  })

  it('keeps the machine connected after a timeout when GRBL still returns status lines', async () => {
    const mock = createMockPort()

    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    await expect(
      service.runGcode(['G1 X-15.2271 Y-14.8662'], { responseTimeoutMs: 5, streamingMode: 'sync' }),
    ).rejects.toThrow('GRBL communication timeout')

    expect(service.getMachineState().connected).toBe(true)
    expect(service.getPort()).toBe(mock.port)
    expect(mock.close).not.toHaveBeenCalled()
    expect(service.getMachineState().status).toContain('Connection kept open for recovery')
  })

  it('treats controller hold as a paused run and can recover after resume without failing the stream', async () => {
    const mock = createMockPort()
    mock.pushLine('Grbl 1.1h')
    mock.setOnWrite((payload) => {
      const command = payload.trim()
      if (command === '$X') {
        mock.pushLine('ok')
      } else if (command.startsWith('G1 ')) {
        mock.pushLine('<Hold:0|WPos:0.950,0.000,0.000|Bf:15,128|FS:0,700>')
      } else if (command === '~') {
        mock.pushLine('ok')
      }
    })

    Object.defineProperty(globalThis.navigator, 'serial', {
      configurable: true,
      value: { requestPort: vi.fn().mockResolvedValue(mock.port), getPorts: vi.fn().mockResolvedValue([]) },
    })

    const service = new GrblWebSerialService()
    await service.connect()

    const runPromise = service.runGcode(['G1 X1.0000 Y0.0000 F1200.000'], { responseTimeoutMs: 200, streamingMode: 'sync' })
    for (let attempt = 0; attempt < 20 && !service.getMachineState().paused; attempt += 1) {
      await new Promise((resolve) => setTimeout(resolve, 10))
    }

    expect(service.getMachineState().paused).toBe(true)
    expect(service.getMachineState().job_state).toBe('paused')

    await service.resume()
    await runPromise

    expect(service.getMachineState().status).toBe('Job complete')
    expect(service.getMachineState().progress_done).toBe(1)
    expect(service.getMachineState().job_state).toBe('completed')
  })
})
