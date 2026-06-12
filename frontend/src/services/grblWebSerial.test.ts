import { beforeEach, describe, expect, it, vi } from 'vitest'

import { BAUD_RATE, GrblWebSerialService } from './grblWebSerial'

function createMockPort() {
  const open = vi.fn().mockResolvedValue(undefined)
  const close = vi.fn().mockResolvedValue(undefined)
  const chunks: Uint8Array[] = []

  const port: SerialPort = {
    readable: {
      getReader() {
        let cancelled = false
        let index = 0
        return {
          async read() {
            if (cancelled) {
              return { done: true, value: undefined }
            }
            const value = chunks[index]
            index += 1
            if (!value) {
              await new Promise((resolve) => setTimeout(resolve, 0))
              return { done: true, value: undefined }
            }
            return { done: false, value }
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
          async write() {
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
})
