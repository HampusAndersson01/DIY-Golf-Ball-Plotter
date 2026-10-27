import { afterEach, describe, expect, it, vi } from 'vitest'

import { canApproveMcpAction, completeDashboardRequest, fetchPendingDashboardRequest, initializeDashboardMcpSession, parseDashboardMcpRequest, publishDashboardSnapshot, startDashboardRequest, type ApprovalConfirmations } from './dashboardMcpBridge'

afterEach(() => vi.unstubAllGlobals())

const confirmations: ApprovalConfirmations = { centeredAtOrigin: false, exactJobAndAreaClear: false }

function request(action: unknown, argumentsValue: unknown) {
  return parseDashboardMcpRequest({
    request_id: 'req-1234',
    action,
    arguments: argumentsValue,
    status: 'awaiting_operator',
    created_at: Date.now() / 1000,
    expires_at: Date.now() / 1000 + 60,
    result: null,
  })
}

describe('dashboard MCP request validation', () => {
  it('coalesces concurrent dashboard session initialization requests', async () => {
    let resolveSession!: (response: Response) => void
    const fetchMock = vi.fn().mockImplementation(() => new Promise<Response>((resolve) => {
      resolveSession = resolve
    }))
    vi.stubGlobal('fetch', fetchMock)

    const firstInitialization = initializeDashboardMcpSession()
    const secondInitialization = initializeDashboardMcpSession()

    expect(fetchMock).toHaveBeenCalledTimes(1)
    resolveSession(new Response(JSON.stringify({ ok: true, csrf_token: 'x'.repeat(48) }), { status: 200 }))
    await Promise.all([firstInitialization, secondInitialization])
  })

  it('rejects actions outside the closed dashboard action union', () => {
    expect(() => request('raw_gcode', { command: 'G1 X10' })).toThrow(/unsupported/i)
  })

  it('rejects extra request fields even on an allowlisted action', () => {
    expect(() => request('connect', { command: 'G1 X10' })).toThrow(/connect request arguments/i)
  })

  it('accepts only exact COM names and bounded USB identity fields', () => {
    const parsed = request('connect', { com_port: 'COM10', usb_vendor_id: 0x1a86, usb_product_id: 0x7523 })
    expect(parsed.action === 'connect' && parsed.arguments.com_port).toBe('COM10')
    expect(() => request('connect', { com_port: 'COM10;G1X1', usb_vendor_id: 0x1a86, usb_product_id: 0x7523 })).toThrow(/connect request arguments/i)
    expect(() => request('connect', { com_port: 'COM10', usb_vendor_id: -1, usb_product_id: 0x7523 })).toThrow(/connect request arguments/i)
  })

  it('rejects run requests bound to a stale job identity', () => {
    const parsed = request('run', { job_id: 'old-job-id', user_instruction: 'Run this job now' })
    expect(parsed.action).toBe('run')
    if (parsed.action === 'run') {
      expect(canApproveMcpAction(parsed, { ...confirmations, exactJobAndAreaClear: true }, 'current-job')).toBe(false)
    }
  })

  it('converts the broker monotonic expiry into a browser-local deadline', () => {
    const parsed = parseDashboardMcpRequest({
      request_id: 'request-1234', action: 'calibrate', arguments: { user_confirmation: 'Centered at origin.' },
      status: 'awaiting_operator', created_at: 100, expires_at: 220, result: null,
    })
    expect(canApproveMcpAction(parsed, { ...confirmations, centeredAtOrigin: true }, null)).toBe(true)
  })

  it('requires operator confirmation for calibration and run', () => {
    const calibrate = request('calibrate', { user_confirmation: 'The plotter is centered at origin.' })
    const run = request('run', { job_id: 'job-1234', user_instruction: 'Run job-1234 now' })
    expect(canApproveMcpAction(calibrate, confirmations, 'job-1')).toBe(false)
    expect(canApproveMcpAction(run, confirmations, 'job-1')).toBe(false)
    if (calibrate.action === 'calibrate') {
      expect(canApproveMcpAction(calibrate, { ...confirmations, centeredAtOrigin: true }, 'job-1')).toBe(true)
    }
    if (run.action === 'run') {
      expect(canApproveMcpAction(run, { ...confirmations, exactJobAndAreaClear: true }, 'job-1234')).toBe(true)
    }
  })

  it('keeps dashboard completion payloads within the size cap', async () => {
    vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response(JSON.stringify({ ok: true, csrf_token: 'x'.repeat(48) }), { status: 200 })))
    await initializeDashboardMcpSession()
    await expect(completeDashboardRequest('request-1234', 'completed', { message: 'x'.repeat(17 * 1024) })).rejects.toThrow(/size limit/i)
  })

  it('rejects a snapshot that attempts to include full G-code', async () => {
    const snapshot = {
      machine: { connected: false, calibrated: false, running: false, paused: false, status: 'Offline', current_gcode_line: 0, y_loop_test: null },
      job: { job_id: null, line_count: 0, summary: null },
      logs: [],
      serial_diagnostics: { last_complete_parsed_line: null, last_parsed_ack_line: null, last_parsed_status_line: null, last_ok_at: null, last_status_at: null, port_state: { portOpen: false, readerActive: false, writerActive: false }, unexpected_ok_count: 0, unexpected_error_count: 0, recent_grbl_lines: [] },
      gcode: ['G1 X100'],
    }
    await expect(publishDashboardSnapshot(snapshot as never)).rejects.toThrow(/invalid shape/i)
  })

  it('starts execution through the dashboard CSRF route only', async () => {
    const fetchMock = vi.fn()
      .mockResolvedValueOnce(new Response(JSON.stringify({ ok: true, csrf_token: 'x'.repeat(48) }), { status: 200 }))
      .mockResolvedValueOnce(new Response(JSON.stringify({ ok: true }), { status: 200 }))
    vi.stubGlobal('fetch', fetchMock)
    await initializeDashboardMcpSession()
    await startDashboardRequest('request-1234')

    const [url, init] = fetchMock.mock.calls[1] as [string, RequestInit]
    expect(url).toBe('/api/mcp/dashboard/requests/request-1234/start')
    expect(new Headers(init.headers).get('X-Dashboard-CSRF')).toHaveLength(48)
  })

  it('uses POST for session and polling so browsers send the Origin header', async () => {
    const fetchMock = vi.fn()
      .mockResolvedValueOnce(new Response(JSON.stringify({ ok: true, csrf_token: 'x'.repeat(48) }), { status: 200 }))
      .mockResolvedValueOnce(new Response(JSON.stringify({ ok: true, request: null }), { status: 200 }))
    vi.stubGlobal('fetch', fetchMock)

    await initializeDashboardMcpSession()
    await fetchPendingDashboardRequest()

    const sessionInit = fetchMock.mock.calls[0]?.[1] as RequestInit
    const pollInit = fetchMock.mock.calls[1]?.[1] as RequestInit
    expect(sessionInit.method).toBe('POST')
    expect(pollInit.method).toBe('POST')
    expect(new Headers(pollInit.headers).get('X-Dashboard-CSRF')).toHaveLength(48)
  })
})
