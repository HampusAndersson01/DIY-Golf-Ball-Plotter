import { beforeEach, describe, expect, it, vi } from 'vitest'

import { BAUD_RATE, GrblWebSerialService } from './grblWebSerial'

function createMockPort() {
  const open = vi.fn().mockResolvedValue(undefined)
  const close = vi.fn().mockResolvedValue(undefined)
  const chunks: Uint8Array[] = []
  const writes: string[] = []
  let onWrite: ((payload: string) => void) | null = null
  let readIndex = 0

  const port: SerialPort = {
    readable: {
      getReader() {
        let cancelled = false
        return {
          async read() {
            while (!cancelled) {
              const value = chunks[readIndex]
              if (value) {
                readIndex += 1
                return { done: false, value }
              }
              await new Promise((resolve) => setTimeout(resolve, 0))
            }
            return { done: true, value: undefined }
          },
          async cancel() {
            cancelled = true
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
      chunks.push(new TextEncoder().encode(`${line}\n`))
    },
    setOnWrite(handler: (payload: string) => void) {
      onWrite = handler
    },
    writes,
  }
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

  it('recovers a buffered run when GRBL is idle and buffers are empty after a missing ok', async () => {
    const mock = createMockPort()
    const statusReplies = [
      '<Idle|MPos:0.000,0.000,0.000|Bf:15,128>',
      '<Idle|WPos:-15.2271,-14.8662,0.000|Bf:15,128>',
      '<Idle|WPos:-15.2271,-14.8662,0.000|Bf:15,128>',
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
    await service.runGcode(['G1 X-15.2271 Y-14.8662'], { responseTimeoutMs: 5, streamingMode: 'buffered' })

    expect(service.getMachineState().progress_done).toBe(1)
    expect(service.getMachineState().status).toBe('Job complete')
    expect(mock.writes.some((payload) => payload.includes('G1 X-15.2271 Y-14.8662'))).toBe(true)
    expect(mock.writes.filter((payload) => payload === '?').length).toBeGreaterThanOrEqual(2)
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
})
