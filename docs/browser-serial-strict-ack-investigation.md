# Browser Web Serial Strict ACK Investigation

## Streaming Architecture

The dashboard "Run G-code" path is frontend-owned:

1. `frontend/src/App.tsx` `handleRun`
2. `frontend/src/services/grblWebSerial.ts` `runGcode`
3. `sendLineAndWaitForAck`
4. Browser Web Serial API
5. GRBL controller

The backend still contains a serial fallback in `app/services/pipeline_core.py`, including `connect_grbl`, `stream_gcode_lines_unlocked`, and `read_next_grbl_line`. That path requires an explicit `SERIAL_PORT` and is not the path used by the browser log line: "streaming ... over Web Serial in Strict Ack mode."

Conclusion: for that log, backend G-code generation is not responsible for missing `ok` bytes. The active runner is the frontend Web Serial service.

## Timeout Diagnostics

Timeout diagnostics now separate:

- raw RX chunks
- current incomplete RX buffer
- complete parsed GRBL lines
- last complete parsed line
- last parsed ACK line
- last parsed status line

A raw fragment such as `o` is no longer reported as the last complete GRBL response. It is reported as `last_raw_fragment` and `current_partial_line_buffer`.

For the observed failure shape:

```text
Last raw fragment: "o"
Current partial line buffer: "o"
No complete "ok" parsed for the active command
```

the failure class is `TIMEOUT_PARTIAL_LINE`.

## Added RX Chunk Trace

Each serial read records:

- timestamp
- read loop id
- byte length
- raw bytes as hex
- decoded text with escaped line endings
- decoder mode
- RX buffer before append
- RX buffer after parsing
- complete parsed lines
- remaining partial buffer

Example target shape:

```text
RX_CHUNK id=3 bytes=[6F] text="o" bufferBefore="" bufferAfter="o" parsed=[]
RX_CHUNK id=3 bytes=[6B 0A] text="k\n" bufferBefore="o" bufferAfter="" parsed=["ok"]
```

## Post-timeout Listening

After a timeout, the frontend keeps the read loop alive for 2 seconds before closing the port. Any late chunks or parsed lines are appended to `last_timeout_debug.post_timeout_observation`.

Interpretation:

- `o` then late `k\n`: timeout/windowing problem.
- `o` with no late bytes: serial/browser/hardware/controller byte loss.
- `Grbl ... ['$' for help]` mid-run: controller reset, often power or noise related.

## Synthetic Strict ACK Tests

`GrblWebSerialService.runStrictAckStressTest(...)` supports:

- `simple_ack`: repeated `G4 P0.001`
- `pen_toggle`: alternating `M3 S700` and `M3 S575`
- `zero_motion`: repeated `G91`, `G1 X0 F1000`, `G90`
- `status_free`: repeated `G4 P0.001` with strict mode status polling disabled

The result includes commands sent, OK count, error count, partial chunks, partial `o` count, timeout count, max ACK latency, and average ACK latency.

## Non-browser Comparison

Use the standalone script:

```bash
python scripts/grbl_strict_ack_compare.py --port COM7 --count 10000 --command "G4 P0.001"
python scripts/grbl_strict_ack_compare.py --port COM7 --scenario pen-toggle --count 5000
python scripts/grbl_strict_ack_compare.py --port COM7 --scenario zero-motion --count 10000
```

Interpretation:

- Python/UGS/Candle/bCNC also loses bytes: suspect hardware, controller, cable, firmware, or servo power.
- Python/UGS succeeds but Web Serial fails: suspect browser Web Serial lifecycle, frontend parser, or tab/device behavior.
- Failures cluster around `M3 S700`/`M3 S575`: suspect servo power draw, electrical noise, or controller brownout.

## Hardware Isolation Checklist

- Do not power the servo from the Arduino 5V pin during stress tests.
- Use a separate servo supply with common ground.
- Add bulk capacitance near the servo rail.
- Use a short, shielded USB cable.
- Avoid unpowered USB hubs.
- Disable Windows USB selective suspend while testing.
- Try another CH340/USB serial adapter or driver version.
- Try 57600 baud only as an isolation test if the firmware supports it.
- Watch for GRBL startup text mid-run, which indicates a controller reset.

## Current Status

The code now proves whether `k\n` arrives after a partial `o`. It does not claim the hardware issue is fixed. A real hardware stress run or non-browser comparison is still required to identify the final root cause.
