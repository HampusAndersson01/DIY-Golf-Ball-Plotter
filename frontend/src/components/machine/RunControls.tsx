import type { MachineState } from '../../api/types'

type Props = {
  machine: MachineState | null
  runReady: boolean
  runStarting: boolean
  onRun: () => void
  onPause: () => void
  onResume: () => void
  onStop: () => void
}

export function RunControls({ machine, runReady, runStarting, onRun, onPause, onResume, onStop }: Props) {
  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Run</div>
          <h2>Job Control</h2>
        </div>
        <span className={`badge ${runReady ? 'good' : 'muted'}`}>{runReady ? 'Ready' : 'Locked'}</span>
      </div>
      <div className="stack-col">
        <button className="button primary" disabled={!runReady || machine?.running || runStarting} onClick={onRun} type="button">
          {runStarting ? 'Starting…' : 'Run G-code'}
        </button>
        <div className="stack-row three-up">
          <button className="button" disabled={!machine?.running || machine?.paused} onClick={onPause} type="button">
            Pause
          </button>
          <button className="button" disabled={!machine?.paused} onClick={onResume} type="button">
            Resume
          </button>
          <button className="button danger ghost" disabled={!machine?.connected} onClick={onStop} type="button">
            Stop
          </button>
        </div>
      </div>
    </section>
  )
}
