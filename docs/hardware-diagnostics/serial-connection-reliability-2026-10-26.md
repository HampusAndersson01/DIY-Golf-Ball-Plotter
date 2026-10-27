# Serial connection investigation — 2026-10-26

## Progress

The dashboard still cannot open COM9. The browser remembers permission for one serial device, but the live dashboard snapshot is `connected: false`; its port, reader, and writer are all closed. A browser permission only authorizes device selection. It does not mean Windows opened the port.

The dashboard log provided during this investigation records:

```text
17:03:08  Browser already has permission for 1 serial port. Click Connect plotter to choose one.
17:03:27  Connect failed: Error: opening the selected OS serial port failed: The operating system rejected opening the selected serial port. (NetworkError)
```

This is an operating-system open failure before the GRBL handshake. It does not show that GRBL received a command or that the dashboard established a connection.

## Connection architecture

- Normal dashboard connection and job streaming: `frontend/src/App.tsx` → `frontend/src/services/grblWebSerial.ts` → browser Web Serial → GRBL. The service requests a user-selected port, opens it at 115200 8-N-1 without flow control, starts one reader and writer, and waits for a GRBL handshake before marking the machine connected. CH340 VID `0x1A86` is included in the picker filters.
- Dashboard MCP requests: local MCP server → authenticated loopback Flask bridge → active dashboard page → the same Web Serial service. MCP does not own a serial handle or access arbitrary G-code/serial routes.
- Separate backend fallback: `app/services/pipeline_core.py` and `app/services/serial_service.py` use PySerial only when `SERIAL_PORT` is explicitly configured. That is not the dashboard's normal run path.
- Diagnostic generation: `POST /generate-diagnostic-gcode` generates the job in Flask and does not open serial or move the plotter.

## Host evidence collected

- PySerial enumerated `COM9` as `USB-SERIAL CH340 (COM9)`, VID/PID `1A86:7523`, location `1-8`; the device reports no USB serial number. Current PnP instance: `USB\VID_1A86&PID_7523\6&3365FBAF&0&8`.
- Windows PnP reports status `OK`, class `Ports`, and `CM_PROB_NONE`. The signed driver reports provider `wch.cn`, version `4.0.2026.2`, INF `oem111.inf`.
- At `2026-10-26T15:44:17Z`, a no-write PySerial attempt failed in `serialwin32._reconfigure_port` / `SetCommState`: `SerialException: Cannot configure port ... PermissionError(13, ..., None, 31)`. Win32 error 31 is `ERROR_GEN_FAILURE`. No bytes were written.
- A separate PowerShell/.NET open also failed with Windows' message that the attached device was not functioning. No bytes were written.
- At `2026-10-26T15:49:04Z`, the reusable diagnostic's first open attempt failed earlier, at `CreateFile`, with Win32 error 5 (`Access is denied`). Its conservative default stopped after that failure. This code can indicate an exclusive owner or a device/driver access failure; the exact handle owner was not identified, so this result is not attributed to a particular process.
- At `2026-10-26T16:20:19Z`, a second single-cycle no-write check again failed at `CreateFile` with Win32 error 5. The port remained enumerated; successful opens/closes remained 0/0.
- The Windows driver store also contains CH340 packages `oem80.inf` / `3.9.2024.9` and `oem112.inf` / `3.7.2022.1`. No package switch or device restart was performed.
- `pnputil /enum-devices /instanceid ... /drivers` confirms both older packages match the device ID at the same reported rank but are out-ranked by the installed 4.0.2026.2 package. This makes an A/B rollback to 3.9.2024.9 a reasonable next test. The computer-use bridge timed out while opening MMC, so Device Manager was not reached and the driver was not changed.
- Windows UserPnp logs show successful `CH341SER_A64` driver-install operations for changing device-instance suffixes. They do not show a corresponding failure or explain Win32 error 31.
- Two `dev.py`, two `run.py`, and two dashboard MCP server processes were present during inspection. The backend is configured without `SERIAL_PORT`, so these processes are not expected to open the controller through the PySerial fallback. Their presence alone does not prove serial ownership.
- The live dashboard MCP snapshot later confirmed disconnected state, line 0, and no active reader/writer. Its recent logs contain the browser permission and `NetworkError` lines above.

The current evidence supports a Windows/CH340 open/configuration problem, with a later access-denied result that leaves exclusive ownership as an unresolved possibility. There is no evidence of a GRBL handshake in this session. Do not infer a firmware or streaming root cause from these open failures.

## Diagnostic utility

Run one no-write attempt:

```powershell
.\.venv\Scripts\python.exe scripts/serial_port_diagnostic.py --port COM9
```

To request the 20-cycle acceptance run and save JSON evidence:

```powershell
.\.venv\Scripts\python.exe scripts/serial_port_diagnostic.py --port COM9 --cycles 20 --output docs/hardware-diagnostics/serial-open-cycles.json
```

By default the utility stops on the first failed open or close, writes zero bytes, and reports enumeration, serial parameters, timing, exception, and Win32 classification. It can continue after a failure with `--continue-after-failure`, but repeated attempts are not useful while the port is consistently failing to open.

Evidence from the attempted 20-cycle run is in `serial-open-cycles-2026-10-26.json`: requested 20, attempted 1, successful opens 0, successful closes 0, bytes written 0, Win32 error 5.

## API fix and test results

`app/routes/raster_routes.py` now serializes NumPy arrays using `tolist()` and NumPy scalar values using `item()` at the response debug-payload boundary. The 3×3 route regression asserts usable G-code is returned. A nested array/scalar regression protects the serializer behavior.

Verified so far:

- 3×3 endpoint and NumPy serializer tests: 2 passed.
- Serial diagnostic error-classification tests: 2 passed.
- Dashboard MCP request tests that initially failed because they depended on the host's COM assignment or checked port enumeration before dashboard availability: 2 passed after making the port fixture deterministic and checking dashboard availability first.
- Frontend tests: `npm test` — 49 passed across 3 files.
- Frontend lint: `npm run lint` — passed.
- Frontend production build: `npm run build` — passed with a Vite warning that the generated JavaScript chunk exceeds 500 kB.
- Focused backend verification after the fixes: 21 passed across the diagnostic route, serial diagnostic, and dashboard MCP server test files.
- Full backend run: `.\.venv\Scripts\python.exe -m pytest -q` — 270 passed, 34 skipped, 9 failed. After fixing and rerunning the two dashboard MCP tests above, the seven remaining failures were reproduced separately:
  - Two Arsenal artifact tests could not find `missed_blob_diagnostics.json` and `path_stats.json` in the expected temp directory.
  - Two Arsenal final-coverage tests measured `0.93977` against thresholds `0.95` and `0.94`.
  - Arsenal repair audit reported 24 optimized repairs against a maximum of 20.
  - Rotation-stability test measured `93.076` against a minimum of `94`.
  - Small-countered-component test measured `3.087` against a maximum of `3.0`.
- Real port opening: failed on both recorded cycles; no successful open/close cycle.
- Real dashboard connection: failed at OS port open; no GRBL communication.
- Reconnect cycles, 30-minute soak, and physical streaming past line 246: not run because no serial connection was established.
- Pytest returned exit code 0 on passing focused runs but printed a shutdown logging error from `app/logging_setup.py:51` because its stream handler wrote after the captured stream closed.

No driver rollback, device disable/enable, registry edit, firmware change, or hardware modification was performed.

## Next investigation point before the CH340 driver comparison

The remaining blocker is before GRBL protocol communication. Identify/close any process that may hold COM9, then repeat a single no-write open. If error 31 reproduces without an exclusive owner, compare the installed driver with the available previous `oem80.inf` / `3.9.2024.9` version. Keep `oem111.inf` installed for reversal and do not uninstall either package. Once Windows opens the port, resume the separate dashboard handshake, reconnect, soak, and strict-ACK streaming investigations.

## Follow-up — CH340 driver and Stop/reconnect lifecycle

The earlier sections record the state before the driver comparison. The following results supersede their statements that no driver switch or successful hardware connection had occurred.

- The exact present device `USB\VID_1A86&PID_7523\6&3365FBAF&0&8` was switched using Windows SetupAPI `DiInstallDevice` to `oem112.inf`, WCH `3.7.2022.1`. The active binding was verified through `Win32_PnPSignedDriver`. COM9 remained assigned; `oem111.inf` and `oem80.inf` were preserved. The older 3.9.2024.9 package was not needed.
- After the switch, the no-write serial diagnostic passed 20/20 opens and 20/20 closes with zero bytes written. The dashboard then completed 20 reconnect cycles; each returned `Connected to GRBL`, with the current G-code line at 0. A final post-soak connection also returned the GRBL startup banner and an Idle status report.
- The first status soak queried immediately after opening and recorded 359 replies from 360 requests, with one timeout. The measured rerun waited 2.5 seconds for USB/controller startup, then held COM9 open for 1,800 seconds and sent `?` every five seconds: 360/360 status responses, zero timeouts, and 108.141–126.113 ms response latency. Both result files are in `docs/hardware-diagnostics/`.
- A later real print log shows setup commands receiving `ok`, a 6,927-line Strict Ack job starting, then Stop sending `CTRL-X RESET` and reporting “Soft reset sent.” A Connect attempt 31 seconds later failed with `InvalidStateError: The selected serial port is already open.` This happened after the 3.7.2022.1 switch, so the driver change fixed the OS-open problem but did not prevent this Stop/reconnect lifecycle failure.
- Code inspection found that `closeSerialPort()` swallowed `port.close()` failures and cleared its `activePort` reference regardless. Stop and the interrupted stream can both request cleanup. If the browser rejected the close, a later Connect could therefore skip cleanup and call `open()` on the still-open port. The browser’s original close exception was not recorded, so this is the code path consistent with the observed `InvalidStateError`, not a captured native close error.
- `closeSerialPort()` now serializes concurrent cleanup, retries a failed Web Serial close once after yielding, and retains the port handle and reports a cleanup error if it remains open. Reconnect must clean up that handle before trying `open()` again. The Stop handler now surfaces cleanup failures in the dashboard log and toast.
- Regression coverage simulates Stop interrupting a pending Strict Ack transaction, a transient first `close()` failure, successful retry, and reconnect on the same port. Frontend validation after this fix: `npm test` — 50 passed; `npm run lint` — passed; `npm run build` — passed.

The Stop/reconnect repair has mocked regression coverage but has not yet been retested during a physical print. No motion was initiated for this follow-up. The earlier Strict Ack stall at line 246 remains a separate investigation; this stop log does not identify the cause of that stall.

## Follow-up — browser-retained open port reconnect

During a post-fix handshake check, the ordinary picker path reproduced `InvalidStateError: The selected serial port is already open` while the dashboard's own state showed disconnected and no reader/writer. Reloading the dashboard cleared its JavaScript service state but did not by itself make a subsequent picker attempt complete in the automation session.

The authorized dashboard reconnect path then returned the previously browser-authorized COM9 `SerialPort` object. The service now checks for non-null Web Serial `readable`/`writable` streams and closes that already-open port before calling `open()`. A new regression first failed on the same mock `InvalidStateError`, then passed after this change.

Physical post-fix evidence: the dashboard connected COM9 at 115200, received the GRBL 1.1h startup banner and a live `<Idle|WPos:...>` status report, with the port, reader, and writer active and zero unexpected `ok`/`error` lines. I disconnected using the dashboard control; the dashboard returned to `connected: false`, and port/reader/writer all returned to closed/inactive. No motion or print job was started. This is one physical reconnect/disconnect cycle after the latest fix; the earlier 20-cycle and soak results predate it.

The Stop/cleanup race fix remains covered by simulated strict-ACK streaming and a transient close failure. It has not been physically exercised by stopping an active print. The Strict Ack stall at line 246 also remains untested in hardware and unresolved.

## Follow-up — 2026-10-27 startup and COM9 access denied

- At 11:43 local, the dashboard logged that one Web Serial port was already authorized, then `open()` failed with `NetworkError`. The Flask startup log at 11:43:32 reported `SERIAL_PORT=None`; the `/state` endpoint currently reports `connected: false`, `running: false`, and no active job.
- The inspected `dev.py`/Flask startup path does not open serial. Frontend bootstrap checks `navigator.serial.getPorts()` for permission and does not open the port. Its connect path is user-triggered and the handshake sends only CRLF wakeup bytes and status query; no pen servo command is sent at startup.
- The CH340 remains bound to WCH 3.7.2022.1 (`oem112.inf`). A fresh one-cycle no-write PySerial diagnostic on COM9 still fails at `CreateFile` with Win32 error 5 (`Access is denied`); bytes written 0, successful opens 0. The specific process holding the handle is not identified.
- The dashboard Web Serial service now holds a same-origin Web Lock for the lifetime of its connection. A second updated dashboard tab refuses to open the port and asks the operator to disconnect the first tab. A Windows `NetworkError` also suggests closing other dashboard tabs or serial programs. This prevents contention between updated dashboard tabs; it cannot release an existing handle held by older page code or another program.
- Frontend lint and build pass. The three focused ownership/open tests pass. The full frontend suite ran 52 passing tests and one failed timing-sensitive serial simulation expecting a second `?` status probe; the observed count was one. No serial write or physical motion test was performed.
- Servo movement reported during `python dev.py` startup is not explained by the inspected application startup path. No startup pen command or serial open is logged. The physical cause remains undetermined; do not send speculative servo commands to test it.
