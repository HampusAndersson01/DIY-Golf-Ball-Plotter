import type { MachineState } from '../../api/types'

type Props = {
  machine: MachineState | null
  progressPercent: number
  canStop: boolean
  onStop: () => void
  currentKind: string
  elapsedLabel: string
  remainingLabel: string
}

function statusTone(active: boolean, pending = false) {
  if (pending) return 'chip chip-warn'
  return active ? 'chip chip-good' : 'chip chip-muted'
}

export function TopStatusBar({ machine, progressPercent, canStop, onStop, currentKind, elapsedLabel, remainingLabel }: Props) {
  const terminal = isTerminal(machine?.job_state)
  const lineLabel = machine?.progress_total ? 'Streamed lines' : 'Lines'
  const remainingCopy = terminal ? `${remainingLabel} remaining` : `${remainingLabel} left`

  return (
    <header className="top-status-bar">
      <div className="title-block">
        <h1>Golf Ball Plotter</h1>
        <p>{machine?.status ?? 'Idle'}</p>
      </div>

      <div className="top-progress">
        <div className="progress-copy">
          <span>Progress</span>
          <strong>
            {lineLabel} {machine?.current_gcode_line ?? 0} / {machine?.progress_total ?? 0}
          </strong>
          <em>{currentKind}</em>
        </div>
        <div className="progress-track">
          <div className="progress-fill" style={{ width: `${progressPercent}%` }} />
        </div>
        <div className="progress-meta">
          <strong>{progressPercent}%</strong>
          <span>{elapsedLabel} elapsed</span>
          <span>{remainingCopy}</span>
        </div>
      </div>

      <div className="status-cluster">
        <div className="status-strip">
          <div className={statusTone(Boolean(machine?.connected))}>
            <span>Machine</span>
            <strong>{machine?.connected ? 'Connected' : 'Disconnected'}</strong>
          </div>
          <div className={statusTone(Boolean(machine?.calibrated), !machine?.calibrated)}>
            <span>Calibration</span>
            <strong>{machine?.calibrated ? 'Origin locked' : 'Pending'}</strong>
          </div>
          <div className={statusTone(Boolean(machine?.running), Boolean(machine?.paused))}>
            <span>Job</span>
            <strong>{machine?.paused ? 'Paused' : machine?.running ? 'Running' : machine?.status ?? 'Idle'}</strong>
          </div>
        </div>
        <button className="button danger" disabled={!canStop} onClick={onStop} type="button">
          Emergency Stop
        </button>
      </div>
    </header>
  )
}

function isTerminal(jobState?: string | null) {
  const value = jobState?.toLowerCase()
  return value === 'completed' || value === 'stopped' || value === 'aborted' || value === 'error' || value === 'failed'
}
