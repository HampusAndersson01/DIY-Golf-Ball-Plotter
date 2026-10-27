# Local Dashboard MCP

This optional local MCP server lets Codex read the active plotter dashboard and submit a small set of typed requests. The React dashboard remains the only owner of Web Serial and the only path that can connect to GRBL or start motion.

## Start the app

Start Flask and the frontend before asking Codex to use the tools. From the repository root, either run both together:

```powershell
.\.venv\Scripts\python.exe dev.py
```

Or start them separately:

```powershell
.\.venv\Scripts\python.exe run.py
```

```powershell
cd frontend
npm run dev
```

Open `http://127.0.0.1:5173/` and keep the dashboard page open. It publishes a limited live snapshot to Flask and polls for typed requests. If the page closes or stops heartbeating, unapproved requests expire and cannot run after a later reconnect.

## Codex registration

The local stdio server is registered as `golfball-plotter-dashboard`. It uses the repository's `.venv` Python and `scripts/dashboard_mcp_server.py`. Codex loads the server from its user configuration; restart or reload Codex after the registration changes.

To remove it later:

```powershell
codex mcp remove golfball-plotter-dashboard
```

Check the registration without printing the token:

```powershell
codex mcp get golfball-plotter-dashboard
```

## Token and local access

Flask creates `instance/dashboard_mcp_token` on first startup. The file is ignored by Git and restricted to the current Windows account where ACL support is available. The MCP server reads only this token file and sends requests to the fixed loopback address `http://127.0.0.1:5000`.

The Flask session-signing key stays in Flask process memory and is never passed to Codex or the MCP server. Restarting Flask invalidates dashboard sessions; the open page establishes a new session automatically. The browser-only CSRF token stays in page memory.

Session initialization is safe to repeat. React development Strict Mode and hot reload can start the bridge effect more than once; the browser coalesces concurrent initialization requests, and Flask returns the existing CSRF token for an already initialized session instead of rotating it. This keeps the next snapshot and polling requests aligned with the session cookie.

Do not copy the token into chat, environment variables, or a checked-in file. If the token file is removed or invalid, restart Flask to generate a new token and restart Codex so it launches a fresh MCP process.

## Available tools

| Tool | Purpose |
| --- | --- |
| `get_machine_status` | Read browser connection, calibration, run, and current line state. |
| `get_dashboard_logs` | Read up to 50 recent safe dashboard log entries. |
| `get_current_job_summary` | Read the current job ID, line count, and summary; it never returns G-code. |
| `get_serial_diagnostics` | Read bounded parsed serial status diagnostics. |
| `generate_diagnostic_job` | Request the fixed `3x3_squares` or `x_axis_rotation_ticks` preview. Generation sends no serial commands. |
| `request_connect` | Connect one exact Windows COM port you name, using its USB identity to find a single matching port that the dashboard browser already has permission to use. |
| `request_calibration` | Request calibration using your exact recent statement that the plotter is centered at origin. |
| `request_run` | Request the exact current job using your exact recent instruction to run it. |
| `get_action_status` | Read whether a request is waiting, executing, or finished. |

There are no MCP tools for raw G-code, arbitrary serial commands, jog, pen/servo control, GRBL settings, Y-loop tests, or approving a motion request.

## Approval and request status

For connection, name the exact COM port you approve (for example, `COM10`) in the MCP request. MCP checks that Windows currently reports that port and sends only its COM name plus USB vendor/product IDs to the dashboard. The dashboard selects from `navigator.serial.getPorts()` and connects only when exactly one previously authorized browser port has that same USB identity. It does not open the browser picker or infer which port you meant. If the browser has no matching permission, authorize the device once with the normal **Connect plotter** button, then retry. If multiple authorized devices share that identity, disconnect the others and retry.

For calibration and job runs, first state your intent explicitly in chat. The MCP tool argument must contain that exact recent statement; Codex must not infer permission from earlier discussion. Then review the request in the dashboard:

- **Calibration:** confirm the pen is physically centered at the ball origin and the machine is ready.
- **Run:** verify the displayed job ID is the one you want and confirm the motion area is clear.

The separate dashboard click is required for calibration and job runs even after you authorize the action in chat. Connecting the exact COM port you approved does not require another dashboard click. The dashboard rechecks connection, calibration, idle state, Y-loop state, and job identity before serial motion. If the job or machine state changes before approval, the request is rejected.

Requests move through `queued`, `awaiting_operator`, and `executing`, then finish as `completed`, `rejected`, `expired`, or `failed`. Unapproved requests expire after 120 seconds or when the dashboard heartbeat is lost. Once approved, a run can exceed that timeout; if the dashboard heartbeat disappears during execution, the request reports `failed` with a reminder to check the plotter. It will never be replayed after reconnect.

## Safety boundary

Keep the golf ball and pen clear while reviewing requests. Use the machine's physical emergency stop if motion is unsafe. This software approval is not a physical interlock.
