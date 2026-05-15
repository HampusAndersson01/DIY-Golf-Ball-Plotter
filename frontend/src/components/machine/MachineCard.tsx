import { apiConfig } from '../../api/client'
import { useAppStore } from '../../store/appStore'

type Props = {
  onConnect: () => void
  onApplyConfig: () => void
}

export function MachineCard({ onConnect, onApplyConfig }: Props) {
  const machine = useAppStore((state) => state.machine)
  const settings = useAppStore((state) => state.settings)!
  const updateSetting = useAppStore((state) => state.updateSetting)
  const busy = useAppStore((state) => state.busy.connecting)
  const appendLog = useAppStore((state) => state.appendLog)

  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Machine</div>
          <h2>Connection</h2>
        </div>
        <span className={`badge ${machine?.connected ? 'good' : 'muted'}`}>{machine?.connected ? 'Online' : 'Offline'}</span>
      </div>

      <div className="stack-row">
        <button className="button primary" disabled={busy} onClick={onConnect} type="button">
          {busy ? 'Connecting...' : 'Connect'}
        </button>
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
