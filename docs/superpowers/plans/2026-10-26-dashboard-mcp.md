# Guarded Dashboard MCP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a local Codex MCP server that can inspect the live dashboard and request a small allowlist of actions while keeping machine motion behind explicit chat authorization and a separate dashboard confirmation.

**Architecture:** A Python stdio MCP server talks to a loopback-only Flask bridge. Flask relays typed requests and a limited dashboard snapshot; the React dashboard remains the only owner of Web Serial and executes approved actions through existing handlers. Motion requests require an explicit user request in chat plus a separate operator approval in the dashboard.

**Tech Stack:** Python 3.11+, Flask, official MCP Python SDK v2 (`mcp>=2,<3`), React 19, TypeScript, Web Serial.

**Spec:** `docs/superpowers/specs/2026-10-26-dashboard-mcp-design.md`

## Global Constraints

- Never let MCP access raw serial, arbitrary G-code, arbitrary URLs, COM ports, or backend machine-control routes.
- Keep Web Serial owned by the active React dashboard and `GrblWebSerialService`.
- The user must explicitly request calibration/run in chat; the dashboard must separately require a direct operator click.
- Calibration approval must confirm the pen is centered at origin and the machine is ready; run approval must identify the exact job and confirm the area is clear.
- Require a live dashboard page; never defer a motion request for later execution.
- Keep the bridge loopback-only; authenticate MCP requests with a per-install secret and dashboard completion with an HttpOnly session plus an in-memory CSRF token unavailable to MCP.
- Do not expose manual jog, pen movement, servo control, GRBL configuration, Y-loop, or an MCP approval tool.
- Preserve all existing Codex MCP configuration when registering this server.

## Review Focus

1. Dashboard absent or stale: actions must fail/expire and never run after a later page reconnect; cover in `tests/test_dashboard_mcp_bridge.py`.
2. Job changes while a run awaits approval: reject the stale request; cover in `frontend/src/services/dashboardMcpBridge.test.ts` and a route/service regression test.
3. Concurrent or repeated motion requests: permit at most one pending motion action; cover in `tests/test_dashboard_mcp_bridge.py`.
4. Unauthorized or non-loopback MCP calls: reject before exposing status or queueing actions; cover in `tests/test_dashboard_mcp_routes.py`.
5. Rejected, expired, or failed operator approval: return a terminal result and send no motion; cover in backend bridge tests and dashboard approval flow tests.
6. Missing or paraphrased user authorization: reject calibration/run requests without the required bounded user statement; cover in `tests/test_dashboard_mcp_server.py`.

---

### Execution setup

- [ ] Review `git status` and create `feature/dashboard-mcp-safety` from the current branch before implementation. Carry the two existing diagnostic G-code edits forward as unstaged work; do not rewrite or stage them as part of the MCP task.
- [ ] Keep the Codex user config unchanged until the implementation and local server are ready to register.

### Task 1: Add the authenticated dashboard request broker

**Files:**
- Create: `app/services/dashboard_mcp_bridge.py`
- Create: `app/routes/mcp_bridge_routes.py`
- Modify: `app/routes/__init__.py`
- Modify: `app/extensions.py`
- Modify: `app/config.py`
- Modify: `.gitignore`
- Test: `tests/test_dashboard_mcp_bridge.py`
- Test: `tests/test_dashboard_mcp_routes.py`

**Interfaces:**
- `DashboardMcpBridge.publish_snapshot(snapshot: dict) -> None`
- `DashboardMcpBridge.get_snapshot() -> dict | None`
- `DashboardMcpBridge.create_request(action: str, arguments: dict) -> dict`
- `DashboardMcpBridge.get_next_dashboard_request() -> dict | None`
- `DashboardMcpBridge.start_request(request_id: str) -> dict` (dashboard session only; moves a clicked request into `executing`)
- `DashboardMcpBridge.complete_request(request_id: str, status: str, result: dict) -> dict`
- `DashboardMcpBridge.get_request(request_id: str) -> dict`
- MCP routes use `/api/mcp/v1/*`; dashboard relay routes use `/api/mcp/dashboard/*`.

- [x] **Step 1: Write the failing broker tests** for the five Review Focus cases, an allowlisted request, request expiry, bounded queue, and terminal result retrieval.
- [x] **Step 2: Run the focused broker tests and verify failure** with missing bridge types/behavior.
- [x] **Step 3: Implement a thread-safe in-memory broker** with a 5-second dashboard heartbeat lease, 120-second request expiry, maximum 10 pending requests, one pending motion request, and terminal outcomes `completed`, `rejected`, `expired`, and `failed`.
- [x] **Step 4: Write failing route tests** for token requirement, loopback-only access, exact route/action allowlist, browser session/CSRF/origin requirements on dashboard completion, and the absence of any MCP approval endpoint.
- [x] **Step 5: Implement Flask routes** that authenticate MCP calls with `X-Dashboard-MCP-Token` loaded from `instance/dashboard_mcp_token`, reject non-loopback MCP clients, validate bounded JSON schemas, and require a browser-only session plus in-memory CSRF token for dashboard relay writes.
- [x] **Step 6: Generate a random MCP token and a separate process-local Flask session secret** on app startup; keep the session-signing secret inaccessible through the MCP interface and add the token file to `.gitignore`.
- [x] **Step 7: Register the bridge in app extensions and routes** without exposing or calling `MachineService`/`SerialService` control methods.
- [x] **Step 8: Add a dashboard-session-only execution transition** so approved long runs do not expire while executing; fail the action if the dashboard heartbeat disappears.
- [x] **Step 9: Run focused broker and route tests** and verify all rejection cases fail closed.

### Task 2: Add the dashboard snapshot and operator approval flow

**Files:**
- Create: `frontend/src/services/dashboardMcpBridge.ts`
- Create: `frontend/src/services/dashboardMcpBridge.test.ts`
- Create: `frontend/src/components/machine/McpActionApproval.tsx`
- Modify: `frontend/src/App.tsx`
- Modify: `frontend/src/components/machine/CalibrationCard.tsx` only if needed to share confirmation copy/state.

**Interfaces:**
- `publishDashboardSnapshot(snapshot: DashboardMcpSnapshot): Promise<void>`
- `fetchPendingDashboardRequest(): Promise<DashboardMcpRequest | null>`
- `completeDashboardRequest(requestId: string, status: DashboardMcpRequestStatus, result?: object): Promise<void>`
- `startDashboardRequest(requestId: string): Promise<void>`
- `canApproveMcpAction(request: DashboardMcpRequest, confirmations: ApprovalConfirmations): boolean`
- Dashboard actions are the closed union `connect | generate_diagnostic_job | calibrate | run`.
- `DashboardMcpSnapshot` contains machine state, capped logs/serial diagnostics, current job ID, G-code line count, and summary; it never contains full G-code.

- [x] **Step 1: Add frontend service tests** proving invalid action names and stale job IDs cannot be accepted, completion payloads remain typed and size-limited, and calibration/run approval stays disabled until the correct confirmations are present.
- [x] **Step 2: Implement the bridge client** using same-origin dashboard relay routes and strict runtime validation of request IDs, action names, snapshot fields, and response sizes; attach the browser-only CSRF token only to dashboard relay writes.
- [x] **Step 3: Implement the approval component** with accessible keyboard/focus behavior and action-specific text. It displays the quoted user request. Calibration requires an explicit checkbox for centered pen/origin; run requires an explicit checkbox for the exact job and clear area. The component has no MCP-callable approval callback or route.
- [x] **Step 4: Establish the browser-only dashboard session** and publish a minimal snapshot/poll while the dashboard is mounted. Include only the latest 50 bounded log entries and a capped diagnostic tail; keep CSRF state in memory and heartbeat loss removes page availability.
- [x] **Step 5: Add a generation identity** whenever the active G-code changes. Bind `request_run` to this ID and re-check it, connection, calibration, idle status, and loop state immediately before invoking the existing run handler.
- [x] **Step 6: Route `connect` through a visible dashboard button** so `navigator.serial.requestPort()` is still called from a human click.
- [x] **Step 7: Route calibration and run through the approval component**, then call the existing `handleCalibrate`/`handleRun` execution code without showing a redundant native confirmation. Existing direct UI buttons retain their current confirmation dialogs.
- [x] **Step 8: Route diagnostic generation only through the existing fixed-pattern handlers.** It may update preview/G-code but must never call a serial method.
- [x] **Step 9: Add a pre-execution acknowledgement for each dashboard click**; calibration/run recheck machine/job state after the acknowledgement and before serial motion. Web Serial port selection stays synchronous from the operator's click, and the broker acknowledgement runs after port selection but before opening the port.
- [x] **Step 10: Run the focused frontend tests** and verify each pending request reaches exactly one terminal result.

### Task 3: Implement the local MCP stdio server

**Files:**
- Create: `scripts/dashboard_mcp_server.py`
- Modify: `pyproject.toml`
- Test: `tests/test_dashboard_mcp_server.py`

**Interfaces:**
- Use `mcp.server.MCPServer` and default stdio transport.
- Expose `get_machine_status()`, `get_dashboard_logs(limit: int = 20)`, `get_current_job_summary()`, `get_serial_diagnostics()`, `generate_diagnostic_job(pattern: Literal["3x3_squares", "x_axis_rotation_ticks"])`, `request_connect()`, `request_calibration(user_confirmation: str)`, `request_run(job_id: str, user_instruction: str)`, and `get_action_status(request_id: str)`.
- Every request tool returns an opaque request ID and current state; no tool accepts generic route names, command text, a serial port, or an approval result.

- [x] **Step 1: Write failing tool tests** for each registered tool, tool argument validation, absent dashboard behavior, and the fact that no approval/raw-command tool exists.
- [x] **Step 2: Run focused MCP server tests and verify failure** before implementation.
- [x] **Step 3: Add `mcp>=2,<3`** to project dependencies and implement the stdio server with typed tools and bounded standard-library HTTP requests to Flask.
- [x] **Step 4: Read only the MCP token from the ignored per-install file**; return a clear unavailable error if Flask/dashboard/token is not ready. The MCP process never reads the Flask session secret or browser CSRF token. Keep logs on stderr and stdout exclusively for MCP frames.
- [x] **Step 5: Document in tool descriptions** that `user_confirmation` and `user_instruction` must be exact, recent user quotes; calibration requires the user to state the machine is centered, and both actions require separate dashboard approval.
- [x] **Step 6: Run focused MCP server tests** and inspect the discovered tool list for forbidden capabilities.

### Task 4: Register Codex and document operation/safety

**Files:**
- Create: `docs/dashboard-mcp.md`
- Update: `README.md` only if the setup guide is the appropriate stable entry point.
- Modify user config through: `codex mcp add` after checking for an existing `golfball-plotter-dashboard` entry.

**Interfaces:**
- Register the absolute `.venv\Scripts\python.exe` and absolute `scripts\dashboard_mcp_server.py` path as a stdio server named `golfball-plotter-dashboard`.
- The server resolves the project root from its own script path and uses the default local Flask URL `http://127.0.0.1:5000`.

- [ ] **Step 1: Document startup order**, token-file location, MCP tool list, request status flow, user approval requirements, and how to remove the server registration.
- [ ] **Step 2: Check the Codex MCP registry** and add the server without overwriting or removing other entries.
- [ ] **Step 3: Inspect the resulting Codex registration** to verify command and script paths while avoiding disclosure of secrets.
- [ ] **Step 4: Run the project’s relevant validation commands**: `pytest -q`, `cd frontend && npm run lint`, and `cd frontend && npm run build`.
- [ ] **Step 5: Perform a no-hardware integration check** that discovers tools, reads an offline dashboard state, and rejects connect/calibrate/run when the page is absent. Do not run a physical motion test.
- [ ] **Step 6: Refresh Graphify with `graphify update .`** and confirm generated graph files remain ignored.
