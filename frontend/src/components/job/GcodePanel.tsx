type Props = {
  gcode: string[]
}

export function GcodePanel({ gcode }: Props) {
  return (
    <div className="drawer-pane">
      <textarea className="code-box" readOnly value={gcode.join('\n')} />
    </div>
  )
}
