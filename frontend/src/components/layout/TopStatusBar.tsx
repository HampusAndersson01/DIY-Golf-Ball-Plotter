import type { MachineState } from '../../api/types'
import { FiHelpCircle, FiSettings, FiWifi } from 'react-icons/fi'

type Props = {
  machine: MachineState | null
  progressPercent: number
  canStop: boolean
  onStop: () => void
  currentKind: string
  elapsedLabel: string
  remainingLabel: string
  runReady: boolean
}

function statusTone(active: boolean, pending = false) {
  if (pending) return 'chip chip-warn'
  return active ? 'chip chip-good' : 'chip chip-muted'
}

export function TopStatusBar({ machine, progressPercent, canStop, onStop, currentKind, elapsedLabel, remainingLabel, runReady }: Props) {
  const terminal = isTerminal(machine?.job_state)
  const lineLabel = machine?.progress_total ? 'Streamed lines' : 'Lines'
  const remainingCopy = terminal ? `${remainingLabel} remaining` : `${remainingLabel} left`
  const liveState = machine?.emergency_stopped
    ? 'Error'
    : machine?.paused
      ? 'Paused'
      : machine?.running
        ? 'Running'
        : 'Idle'

  return (
    <header className="top-status-bar">
      <div className="title-block">
        <h1>Golf Ball Plotter</h1>
        <p>{machine?.status ?? 'Idle'}</p>
        <span className={`state-pill ${liveState === 'Running' ? 'good' : liveState === 'Error' ? 'warn' : 'muted'}`}>State: {liveState}</span>
      </div>

      <div className="top-progress">
        <div className="progress-copy">
          <span>Progress</span>
          <strong>{progressPercent}%</strong>
          <span className="progress-divider" aria-hidden="true" />
          <span>Elapsed</span>
          <strong>{elapsedLabel}</strong>
          <span className="progress-divider" aria-hidden="true" />
          <span>Remaining</span>
          <strong>{remainingLabel}</strong>
        </div>
        <div className="progress-track">
          <div className="progress-fill" style={{ width: `${progressPercent}%` }} />
        </div>
        <div className="progress-meta">
          <span>{lineLabel} {machine?.current_gcode_line ?? 0} / {machine?.progress_total ?? 0}</span>
          <span>{currentKind}</span>
          <span>{remainingCopy}</span>
        </div>
      </div>

      <div className="status-cluster">
        <div className="utility-icons">
          <button className="utility-icon-button" type="button" aria-label="Help"><FiHelpCircle /></button>
          <button className="utility-icon-button" type="button" aria-label="Connection"><FiWifi /></button>
          <button className="utility-icon-button" type="button" aria-label="Settings"><FiSettings /></button>
        </div>
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
          <div className={statusTone(runReady, !runReady)}>
            <span>Readiness</span>
            <strong>{runReady ? 'Ready to run' : 'Locked'}</strong>
          </div>
        </div>
        <button className="emergency-stop" disabled={!canStop} onClick={onStop} type="button">
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
