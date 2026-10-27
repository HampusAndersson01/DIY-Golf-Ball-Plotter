"""Hold an idle GRBL connection and probe it with read-only status queries.

This utility sends only the real-time ``?`` status query. It never sends G-code.
Use it only after closing the dashboard or any other serial client.
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


class GrblStatusParser:
    """Collect complete GRBL status reports from fragmented serial chunks."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list[str]:
        self._buffer.extend(chunk)
        lines = self._buffer.split(b"\n")
        self._buffer = bytearray(lines.pop())
        reports = []
        for raw_line in lines:
            line = raw_line.rstrip(b"\r").decode("ascii", errors="replace").strip()
            if line.startswith("<") and line.endswith(">"): 
                reports.append(line)
        return reports


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def run_status_soak(
    port: str,
    duration_seconds: float = 1800,
    interval_seconds: float = 5,
    response_timeout_seconds: float = 2,
) -> dict[str, object]:
    if duration_seconds <= 0 or interval_seconds <= 0 or response_timeout_seconds <= 0:
        raise ValueError("duration, interval, and response timeout must be positive")

    started_at = _utc_now()
    port_opened_at: str | None = None
    started = time.monotonic()
    deadline = started + duration_seconds
    next_query = started
    query_count = 0
    response_count = 0
    timeout_count = 0
    bytes_written = 0
    latencies_ms: list[float] = []
    first_status: str | None = None
    last_status: str | None = None
    timeout_query_numbers: list[int] = []
    last_progress = 0
    parser = GrblStatusParser()
    failure: str | None = None

    try:
        with serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        ) as device:
            port_opened_at = _utc_now()
            # Opening many USB-serial Arduino boards resets the controller. Let
            # its startup banner finish before measuring steady-state queries.
            time.sleep(2.5)
            started_at = _utc_now()
            started = time.monotonic()
            deadline = started + duration_seconds
            next_query = started
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now < next_query:
                    time.sleep(min(next_query - now, deadline - now))
                    continue

                sent_at = time.monotonic()
                bytes_written += device.write(b"?")
                device.flush()
                query_count += 1
                response_deadline = min(deadline, sent_at + response_timeout_seconds)
                matched_status = None
                while time.monotonic() < response_deadline:
                    chunk = device.read(128)
                    if not chunk:
                        continue
                    reports = parser.feed(chunk)
                    if reports:
                        matched_status = reports[-1]
                        break

                if matched_status is None:
                    timeout_count += 1
                    timeout_query_numbers.append(query_count)
                else:
                    response_count += 1
                    latencies_ms.append((time.monotonic() - sent_at) * 1000)
                    first_status = first_status or matched_status
                    last_status = matched_status

                elapsed = time.monotonic() - started
                minute = int(elapsed // 60)
                if minute > last_progress:
                    last_progress = minute
                    print(
                        f"progress: {elapsed:.0f}s, queries={query_count}, "
                        f"responses={response_count}, timeouts={timeout_count}",
                        flush=True,
                    )
                next_query += interval_seconds
                if next_query <= time.monotonic():
                    next_query = time.monotonic() + interval_seconds
    except Exception as error:  # Preserve partial evidence for serial/USB failures.
        failure = f"{type(error).__name__}: {error}"

    ended = time.monotonic()
    return {
        "started_at_utc": started_at,
        "port_opened_at_utc": port_opened_at,
        "ended_at_utc": _utc_now(),
        "port": port,
        "baudrate": 115200,
        "serial_format": "8-N-1, no flow control",
        "duration_requested_seconds": duration_seconds,
        "duration_observed_seconds": round(ended - started, 3),
        "startup_settle_seconds": 2.5,
        "query": "? (GRBL real-time status; no G-code or motion command)",
        "query_interval_seconds": interval_seconds,
        "response_timeout_seconds": response_timeout_seconds,
        "queries_sent": query_count,
        "bytes_written": bytes_written,
        "status_responses": response_count,
        "timeouts": timeout_count,
        "timeout_query_numbers": timeout_query_numbers,
        "min_response_ms": round(min(latencies_ms), 3) if latencies_ms else None,
        "max_response_ms": round(max(latencies_ms), 3) if latencies_ms else None,
        "average_response_ms": round(sum(latencies_ms) / len(latencies_ms), 3) if latencies_ms else None,
        "first_status": first_status,
        "last_status": last_status,
        "error": failure,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Non-motion GRBL status communication soak test")
    parser.add_argument("--port", required=True, help="Serial port, for example COM9")
    parser.add_argument("--duration-seconds", type=float, default=1800)
    parser.add_argument("--interval-seconds", type=float, default=5)
    parser.add_argument("--response-timeout-seconds", type=float, default=2)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = run_status_soak(
        args.port,
        args.duration_seconds,
        args.interval_seconds,
        args.response_timeout_seconds,
    )
    encoded = json.dumps(result, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0 if result["error"] is None and result["timeouts"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
