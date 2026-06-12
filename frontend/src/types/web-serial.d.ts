type SerialParityType = 'none' | 'even' | 'odd'
type SerialFlowControlType = 'none' | 'hardware'

interface SerialPortFilter {
  usbVendorId?: number
  usbProductId?: number
}

interface SerialPortRequestOptions {
  filters?: SerialPortFilter[]
}

interface SerialOptions {
  baudRate: number
  dataBits?: 7 | 8
  stopBits?: 1 | 2
  parity?: SerialParityType
  bufferSize?: number
  flowControl?: SerialFlowControlType
}

interface SerialPortInfo {
  usbVendorId?: number
  usbProductId?: number
}

interface SerialPort {
  readonly readable: ReadableStream<Uint8Array> | null
  readonly writable: WritableStream<Uint8Array> | null
  open(options: SerialOptions): Promise<void>
  close(): Promise<void>
  getInfo(): SerialPortInfo
  forget?: () => Promise<void>
}

interface Serial {
  requestPort(options?: SerialPortRequestOptions): Promise<SerialPort>
  getPorts(): Promise<SerialPort[]>
}

interface Navigator {
  readonly serial?: Serial
}
