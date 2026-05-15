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
  onSelect: (id: string) => void
}

export function StepNav({ machine, hasImage, hasPreview, onSelect }: Props) {
  const complete = {
    connect: Boolean(machine?.connected),
    calibrate: Boolean(machine?.calibrated),
    prepare: hasImage,
    generate: hasPreview,
    run: Boolean(machine?.running || machine?.paused),
  }

  return (
    <nav className="step-nav" aria-label="Workflow">
      {STEPS.map((step, index) => (
        <button
          key={step.id}
          className={`step-pill ${complete[step.id as keyof typeof complete] ? 'is-complete' : ''}`}
          onClick={() => onSelect(step.id)}
          type="button"
        >
          <span>{index + 1}</span>
          <small>{step.label}</small>
        </button>
      ))}
    </nav>
  )
}
