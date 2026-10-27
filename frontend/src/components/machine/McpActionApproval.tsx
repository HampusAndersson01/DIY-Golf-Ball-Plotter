import { useEffect, useRef, useState } from 'react'

import { canApproveMcpAction, type ApprovalConfirmations, type DashboardMcpRequest } from '../../services/dashboardMcpBridge'

type Props = {
  request: DashboardMcpRequest
  currentJobId: string | null
  bridgeReady: boolean
  busy: boolean
  onApprove: (confirmations: ApprovalConfirmations) => void
  onReject: () => void
}

export function McpActionApproval({ request, currentJobId, bridgeReady, busy, onApprove, onReject }: Props) {
  const [centeredAtOrigin, setCenteredAtOrigin] = useState(false)
  const [exactJobAndAreaClear, setExactJobAndAreaClear] = useState(false)
  const [now, setNow] = useState<number | null>(null)
  const headingRef = useRef<HTMLHeadingElement>(null)
  const confirmations = { centeredAtOrigin, exactJobAndAreaClear }

  useEffect(() => {
    headingRef.current?.focus()
  }, [request.request_id])

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 1000)
    return () => window.clearInterval(timer)
  }, [])

  const expired = now !== null && now / 1000 >= request.expires_at
  const canApprove = bridgeReady && now !== null && !expired && canApproveMcpAction(request, confirmations, currentJobId)
  const title = request.action === 'connect'
    ? 'Connect plotter'
    : request.action === 'generate_diagnostic_job'
      ? 'Generate diagnostic pattern'
      : request.action === 'calibrate'
        ? 'Calibrate origin'
        : 'Run generated job'

  return (
    <section aria-labelledby="mcp-approval-title" aria-live="polite" className="mcp-approval panel" role="region">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Requested through Codex</div>
          <h2 id="mcp-approval-title" ref={headingRef} tabIndex={-1}>{title}</h2>
        </div>
        <span className={`badge ${expired ? 'warn' : 'muted'}`}>{expired ? 'Expired' : request.action === 'connect' && busy ? 'Connecting' : 'Needs your click'}</span>
      </div>

      {request.action === 'calibrate' ? (
        <div className="stack-col">
          <p>Codex request: “{request.arguments.user_confirmation}”</p>
          <p>Calibration sets the current physical position as the machine origin. Confirm only after positioning the pen at the center of the ball.</p>
          <label className="mcp-approval__check">
            <input checked={centeredAtOrigin} onChange={(event) => setCenteredAtOrigin(event.target.checked)} type="checkbox" />
            <span>The pen is physically centered at the ball origin and the plotter is ready.</span>
          </label>
        </div>
      ) : null}

      {request.action === 'run' ? (
        <div className="stack-col">
          <p>Codex request: “{request.arguments.user_instruction}”</p>
          <p>Job ID: <code>{request.arguments.job_id}</code></p>
          {request.arguments.job_id !== currentJobId ? <p role="alert">The generated job changed. This request is locked.</p> : null}
          <label className="mcp-approval__check">
            <input checked={exactJobAndAreaClear} onChange={(event) => setExactJobAndAreaClear(event.target.checked)} type="checkbox" />
            <span>This is the exact job I want, and the machine’s motion area is clear.</span>
          </label>
        </div>
      ) : null}

      {request.action === 'connect' ? <p>Codex approved {request.arguments.com_port}. Connecting only if exactly one matching port is already authorized in this browser.</p> : null}
      {request.action === 'generate_diagnostic_job' ? <p>Generate the fixed “{request.arguments.pattern}” preview and G-code. This does not move the plotter.</p> : null}
      {expired ? <p role="status">This request expired. Ask Codex to submit a new request.</p> : null}
      {!bridgeReady ? <p role="status">Dashboard relay reconnecting. Approval is temporarily disabled.</p> : null}

      <div className="stack-row mcp-approval__actions">
        {request.action !== 'connect' ? <button className="button" disabled={busy || !bridgeReady} onClick={onReject} type="button">Decline</button> : null}
        <button
          className="button primary"
          disabled={request.action === 'connect' || busy || !canApprove}
          onClick={() => onApprove(confirmations)}
          type="button"
        >
          {busy && request.action === 'connect' ? `Connecting ${request.arguments.com_port}…` : busy ? 'Working…' : request.action === 'connect' ? 'Connect approved port' : request.action === 'run' ? 'Approve & run job' : request.action === 'calibrate' ? 'Approve calibration' : 'Generate pattern'}
        </button>
      </div>
    </section>
  )
}
