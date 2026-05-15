import type { MachineState } from '../../api/types'

type Props = {
  machine: MachineState | null
  onCalibrate: () => void
  onClear: () => void
}

export function CalibrationCard({ machine, onCalibrate, onClear }: Props) {
  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Origin</div>
          <h2>Calibration</h2>
        </div>
        <span className={`badge ${machine?.calibrated ? 'good' : 'warn'}`}>{machine?.calibrated ? 'Locked' : 'Pending'}</span>
      </div>
      <p className="panel-copy">Jog to the ball center, then lock the current position as the printable origin in one safe action.</p>
      <div className="stack-col">
        <button className="button primary" disabled={!machine?.connected || machine?.running} onClick={onCalibrate} type="button">
          Set Origin &amp; Calibrate
        </button>
        <button className="button" disabled={!machine?.connected} onClick={onClear} type="button">
          Clear Calibration
        </button>
      </div>
    </section>
  )
}
