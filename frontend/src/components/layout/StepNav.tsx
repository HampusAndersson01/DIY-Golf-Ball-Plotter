import type { MachineState } from '../../api/types'

const STEPS = [
  { id: 'connect', label: 'Connect' },
  { id: 'calibrate', label: 'Calibrate' },
  { id: 'prepare', label: 'Prepare' },
  { id: 'generate', label: 'Generate' },
  { id: 'run', label: 'Run' },
]

type Props = {
  machine: MachineState | null
  hasImage: boolean
  hasPreview: boolean
  runReady: boolean
  runLockReason: string | null
  onSelect: (id: string) => void
}

export function StepNav({ machine, hasImage, hasPreview, runReady, runLockReason, onSelect }: Props) {
  const complete = {
    connect: Boolean(machine?.connected),
    calibrate: Boolean(machine?.calibrated),
    prepare: hasImage,
    generate: hasPreview,
    run: Boolean(machine?.running || machine?.paused || runReady),
  }
  const currentStepId =
    !complete.connect ? 'connect'
      : !complete.calibrate ? 'calibrate'
        : !complete.prepare ? 'prepare'
          : !complete.generate ? 'generate'
            : 'run'

  return (
    <nav className="step-nav" aria-label="Workflow">
      {STEPS.map((step, index) => (
        <div key={step.id} className="step-node-wrap">
          <button
            className={`step-pill ${complete[step.id as keyof typeof complete] ? 'is-complete' : ''} ${currentStepId === step.id ? 'is-current' : ''} ${step.id === 'run' && !runReady && !machine?.running && !machine?.paused ? 'is-locked' : ''}`}
            onClick={() => onSelect(step.id)}
            title={step.id === 'run' && runLockReason ? `Run locked: ${runLockReason}` : undefined}
            type="button"
          >
            <span>{complete[step.id as keyof typeof complete] ? '✓' : index + 1}</span>
            <small>{step.label}</small>
          </button>
          {/* connector handled purely by CSS pseudo-element for even spacing */}
        </div>
      ))}

      {runLockReason ? <p className="step-lock-note">Run locked: {runLockReason}</p> : null}
    </nav>
  )
}
