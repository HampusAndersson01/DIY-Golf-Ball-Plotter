type Props = {
  logs: string[]
}

export function LogsPanel({ logs }: Props) {
  return (
    <div className="drawer-pane">
      <pre className="log-box">{logs.join('\n') || 'No logs yet.'}</pre>
    </div>
  )
}
