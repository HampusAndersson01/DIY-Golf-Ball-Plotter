# Guarded Dashboard MCP Design

**Status:** Approved 2026-10-26  
**Scope:** Local MCP tools for observing and requesting actions through the existing dashboard.

## Intent

Let a local MCP client inspect the Golf Ball Plotter dashboard and ask it to prepare or perform a small set of explicitly permitted actions. The machine is physical hardware, so agent access must not become an unrestricted serial or G-code control path.

## Current system

- The active dashboard is React/Vite. Its `GrblWebSerialService` owns the browser Web Serial connection and streams jobs to GRBL.
- Flask provides the generation and diagnostic APIs. It also has a separate PySerial machine runner; dashboard MCP must not call its serial-control routes.
- Browser serial connection uses a user-activated port picker. An MCP process cannot safely manufacture that browser gesture.
- The dashboard currently asks for confirmation before calibration and before running a generated job. Calibration’s confirmation says the pen is physically at the center of the ball.
- The repository has no MCP server dependency or dashboard bridge today.

## Goals

- Expose machine connection/calibration state, current job summary, dashboard logs, and serial diagnostics to an MCP client.
- Allow safe job preparation, initially by requesting the existing diagnostic pattern generator, without moving the machine.
- Let an MCP client request connection, calibration, or a run, while the dashboard requires an operator action before those operations execute.
- Keep all serial reads and writes in the active browser page and existing `GrblWebSerialService`.
- Make approvals visible in the dashboard and report accepted, rejected, expired, and completed requests to the MCP client.

## Safety requirements

- The MCP server has no raw serial, arbitrary G-code, arbitrary URL, or direct backend machine-route tool.
- Only a fixed allowlist of tool names, arguments, and dashboard operations is accepted.
- `connect` can only open a dashboard prompt; the operator must click Connect and complete the browser’s Web Serial picker.
- Calibration and job execution are requests, never immediate MCP actions. The MCP tool requires the user's explicit statement/request as an argument; the tool description requires an exact recent quote and forbids inferring it. The dashboard must also receive a direct operator click for each request. Calibration approval explicitly confirms that the pen is at the center/origin and the machine is ready. Run approval identifies the exact generated job and confirms the machine area is clear.
- The MCP server cannot approve its own pending requests. MCP bearer-token routes can create/read requests but cannot complete them. Dashboard completion uses an HttpOnly browser session plus a CSRF token held only in the active page, and the dashboard also checks its exact Origin.
- No remote jog, pen movement, servo control, GRBL configuration, or Y-loop tool is included in the initial version.
- Unapproved requests expire and are cancelled if the page disconnects, the machine state becomes incompatible, or the job changes before approval. After a direct dashboard click, the broker marks the request `executing`; it does not apply the short approval timeout to a long print. If the dashboard heartbeat disappears during execution, the request becomes `failed` with an outcome-unknown message and is never replayed after reconnect.
- The local bridge accepts connections only from loopback and authenticates requests with a per-install or per-session secret. It does not expose the app to the LAN.

## Approaches considered

1. **Read-only MCP.** Simple and low risk, but it cannot prepare jobs or request dashboard actions.
2. **Guarded dashboard bridge — selected.** A local stdio MCP server communicates with Flask over loopback. Flask holds a bounded queue of typed requests and receives state updates from the active dashboard. The React page polls for requests and executes them through existing dashboard handlers only after any required operator approval. This preserves browser ownership of Web Serial and supports useful, auditable work while keeping machine motion behind a human confirmation.

The user selected the guarded dashboard bridge and approved requiring both an explicit chat statement and a separate dashboard click for calibration/run. The MCP tool will display the quoted user statement for audit, but the dashboard confirmation remains mandatory. Browser automation that clicks through the UI was excluded because it is brittle and would make the human approval boundary less clear.

## Initial tool surface

Read tools:

- `get_machine_status`
- `get_dashboard_logs`
- `get_current_job_summary` (counts, bounds, current line, and generation identity; not an unrestricted G-code dump)
- `get_serial_diagnostics`

Safe preparation:

- `generate_diagnostic_job` with a fixed supported pattern and validated settings; this generates preview/G-code but never starts motion.

Typed requests requiring dashboard participation:

- `request_connect` — presents guidance; the operator performs the browser picker gesture.
- `request_calibration(user_confirmation: str)` — requires a short exact quote of the user's statement that the machine is centered; dashboard displays it and asks the operator to confirm centered pen/origin before calling the existing calibration handler.
- `request_run(job_id: str, user_instruction: str)` — requires the user's explicit run instruction; dashboard displays it with the exact current job summary and requires operator approval before calling the existing run handler.

An emergency-stop request may be added only if the existing stop behavior and its state effects are documented and reviewed; it is not required for the first version. Other physical controls remain out of scope.

## Components and data flow

1. A Python MCP server runs over stdio as a local MCP client launches it.
2. The MCP server sends authenticated, typed requests to a loopback-only Flask bridge.
3. The active React dashboard publishes a limited machine/job snapshot and polls the bridge for pending requests.
4. The dashboard either reads data, performs safe generation through existing APIs, or presents an operator approval screen.
5. For approved motion, the page calls the same existing handlers and `GrblWebSerialService` used by the dashboard. The bridge returns the result to the MCP tool call.

If no dashboard page is connected, the bridge reports that state and does not queue motion for later surprise execution. Requests are bound to a short-lived request ID and the current job identity; stale requests cannot run a newly generated job.

## Non-goals

- Replacing Web Serial or migrating the dashboard’s normal job runner.
- Giving the MCP process independent access to COM ports or PySerial machine routes.
- Unattended calibration, job start, jogging, pen actuation, arbitrary G-code, or controller configuration.
- Uploading arbitrary files from an MCP host in the initial version.
- Guaranteeing physical safety independent of the operator’s checks or the hardware emergency stop.

## Implementation choices left to the builder

- Use the official MCP Python SDK and pin a compatible release rather than hand-implementing the protocol.
- Keep the stdio channel free of logs; diagnostics go to stderr.
- Keep the bridge local, authenticated, schema-validated, rate-limited, and bounded in queue size. Store its generated MCP token under `instance/` and keep the independent Flask session-signing secret only in process memory; a Flask restart invalidates browser sessions. Ignore the token path in Git and never give the session secret to the MCP process.
- Reuse existing dashboard handlers; isolate bridge state from the legacy backend serial runner.
- The user approved registering the local stdio server in Codex's user MCP configuration; preserve all existing entries and note that the server becomes available after the host reloads its MCP configuration.
- Keep server logs off stdout so MCP stdio frames remain valid.

## Acceptance criteria

- MCP tools are discoverable through a standard local MCP host.
- Read tools report the active browser dashboard’s state, not the Flask serial runner’s separate state.
- A diagnostic generation request updates the dashboard preview and causes no serial writes.
- MCP requests cannot bypass the dashboard approval UI for calibration or run.
- Calibration cannot proceed through the MCP bridge without the operator explicitly confirming centered pen/origin in the dashboard.
- A pending request is rejected on timeout, page loss, changed job identity, disconnected controller, or incompatible machine state.
- No MCP tool accepts raw G-code, a COM port, arbitrary URL, or generic route name.
- Existing non-MCP dashboard workflows continue using the current Web Serial owner.

## Known limits

The confirmation gate protects the MCP bridge and its tools. Other independent automation with access to the browser UI is outside that boundary. A physical interlock is stronger than any software confirmation. The MCP feature must never be described as a substitute for the operator’s physical checks.
