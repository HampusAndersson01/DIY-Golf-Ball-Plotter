import { useEffect, useState } from 'react'
import { apiConfig, fetchSerialPorts, type SerialPorts } from '../../api/client'
import { useAppStore } from '../../store/appStore'

type Props = {
  onConnect: (port: string) => void
  onApplyConfig: () => void
}

export function MachineCard({ onConnect, onApplyConfig }: Props) {
  const machine = useAppStore((state) => state.machine)
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)
  const busy = useAppStore((state) => state.busy.connecting)
  const appendLog = useAppStore((state) => state.appendLog)
  const [ports, setPorts] = useState<SerialPorts['ports']>([])
  const [selectedPort, setSelectedPort] = useState('')
  const [manual, setManual] = useState(false)
  const [manualPort, setManualPort] = useState('')
  const [portError, setPortError] = useState('')
  const [loadingPorts, setLoadingPorts] = useState(true)
  const connectionPort = (manual ? manualPort : selectedPort).trim()

  function applyPorts(result: SerialPorts) {
    setPorts(result.ports)
    setSelectedPort((current) => result.ports.some((port) => port.device === current) ? current : result.selected_port)
    setPortError('')
  }

  function refreshPorts() {
    setLoadingPorts(true)
    void fetchSerialPorts()
      .then(applyPorts)
      .catch((error) => setPortError(String(error)))
      .finally(() => setLoadingPorts(false))
  }

  useEffect(() => {
    let cancelled = false
    void fetchSerialPorts()
      .then((result) => { if (!cancelled) applyPorts(result) })
      .catch((error) => { if (!cancelled) setPortError(String(error)) })
      .finally(() => { if (!cancelled) setLoadingPorts(false) })
    return () => { cancelled = true }
  }, [])

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Machine</div>
          <h2>Connection</h2>
        </div>
        <span className={`badge ${machine?.connected ? 'good' : 'muted'}`}>{machine?.connected ? 'Online' : 'Offline'}</span>
      </div>

      <div className="stack-col">
        <label>
          <span>Serial / USB port</span>
          <select disabled={busy || machine?.connected || loadingPorts} value={manual ? 'manual' : selectedPort} onChange={(event) => {
            const value = event.target.value
            setManual(value === 'manual')
            if (value !== 'manual') setSelectedPort(value)
          }}>
            <option value="">{loadingPorts ? 'Scanning ports...' : 'Select a serial port'}</option>
            {ports.map((port) => <option key={port.device} value={port.device}>{port.device} — {port.description}</option>)}
            <option value="manual">Enter port manually...</option>
          </select>
        </label>
        {manual && <label>
          <span>Port name or device path</span>
          <input disabled={busy || machine?.connected} placeholder="/dev/ttyUSB0, /dev/ttyACM0 or COM4" value={manualPort} onChange={(event) => setManualPort(event.target.value)} />
        </label>}
        {portError && <p role="alert">{portError}</p>}
        {!loadingPorts && !portError && ports.length === 0 && <p>No serial ports detected. Check the USB cable, then refresh ports.</p>}
        <div className="stack-row">
          <button className="button" disabled={busy || machine?.connected || loadingPorts} onClick={refreshPorts} type="button">Refresh ports</button>
          <button className="button primary" disabled={busy || machine?.connected || !connectionPort} onClick={() => onConnect(connectionPort)} type="button">
            {busy ? 'Connecting...' : 'Connect'}
          </button>
        </div>
      </div>

      <details className="details-panel">
        <summary>Machine settings</summary>
        <div className="stack-col">
          <div className="field-grid compact two">
            <label>
              <span>Max feed X</span>
              <input onChange={(event) => updateSetting('xMaxFeed', Number(event.target.value))} type="number" value={settings.xMaxFeed} />
            </label>
            <label>
              <span>Max feed Y</span>
              <input onChange={(event) => updateSetting('yMaxFeed', Number(event.target.value))} type="number" value={settings.yMaxFeed} />
            </label>
            <label>
              <span>Accel X</span>
              <input onChange={(event) => updateSetting('xAcceleration', Number(event.target.value))} type="number" value={settings.xAcceleration} />
            </label>
            <label>
              <span>Accel Y</span>
              <input onChange={(event) => updateSetting('yAcceleration', Number(event.target.value))} type="number" value={settings.yAcceleration} />
            </label>
          </div>

          <div className="stack-row">
            <button className="button" disabled={!machine?.connected} onClick={onApplyConfig} type="button">
              Apply Settings
            </button>
            <button className="text-button" onClick={() => appendLog(`Endpoint map loaded from ${apiConfig.endpoints.connect}`)} type="button">
              API map
            </button>
          </div>
        </div>
      </details>
    </section>
  )
}
