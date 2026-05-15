import type { MachineState } from '../../api/types'

type Props = {
  machine: MachineState | null
  runReady: boolean
  onRun: () => void
  onPause: () => void
  onResume: () => void
  onStop: () => void
}

export function RunControls({ machine, runReady, onRun, onPause, onResume, onStop }: Props) {
  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Run</div>
          <h2>Job Control</h2>
        </div>
        <span className={`badge ${runReady ? 'good' : 'muted'}`}>{runReady ? 'Ready' : 'Locked'}</span>
      </div>

      <p className="panel-copy">Run unlocks only when the machine is connected, origin is calibrated, and generated G-code exists.</p>
      <div className="stack-col">
        <button className="button primary" disabled={!runReady || machine?.running} onClick={onRun} type="button">
          Run Generated G-code
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
