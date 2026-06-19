import type { MachineState, PreviewPath } from '../../api/types'

type Props = {
  machine: MachineState | null
  currentPath: PreviewPath | null
}

export function CurrentLineOverlay({ machine, currentPath }: Props) {
  const pathStatus = machine?.job_state === 'failed'
    ? 'Failed'
    : machine?.job_state === 'stopped'
      ? 'Stopped'
      : machine?.paused
        ? 'Paused'
        : machine?.running
          ? 'Drawing'
          : 'Idle'

  return (
    <div className="current-line-overlay">
      <div>
        <span>Current line</span>
        <strong>{machine?.current_gcode_line ?? 0}</strong>
      </div>
      <div>
        <span>Path status</span>
        <strong>{pathStatus}</strong>
      </div>
      <div>
        <span>Entity kind</span>
        <strong>{currentPath?.kind ?? 'None'}</strong>
      </div>
      <div>
        <span>Path id</span>
        <strong>{currentPath?.id ?? '--'}</strong>
      </div>
    </div>
  )
}
