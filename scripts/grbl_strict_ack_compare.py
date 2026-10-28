"""Strict-ACK GRBL serial comparison test outside the browser.

Usage:
    python scripts/grbl_strict_ack_compare.py --port COM7 --count 10000 --command "G4 P0.001"
    python scripts/grbl_strict_ack_compare.py --port COM7 --scenario pen-toggle --count 5000

Requires pyserial:
    python -m pip install pyserial
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import Iterable

import serial


@dataclass
class AckStats:
    commands_sent: int = 0
    ok_count: int = 0
    error_count: int = 0
    timeout_count: int = 0
    partial_o_count: int = 0
    max_ack_latency_ms: float = 0.0
    total_ack_latency_ms: float = 0.0


def scenario_commands(name: str, count: int, command: str) -> Iterable[str]:
    for index in range(count):
        if name == "simple":
            yield command
        elif name == "pen-toggle":
            yield "M3 S700" if index % 2 == 0 else "M3 S575"
        elif name == "zero-motion":
            yield "G91"
            yield "G1 X0 F1000"
            yield "G90"
        else:
            raise ValueError(f"Unknown scenario: {name}")


def read_complete_line(ser: serial.Serial, timeout_s: float, raw_log: list[dict[str, object]], stats: AckStats) -> str | None:
    deadline = time.monotonic() + timeout_s
    buffer = b""
    while time.monotonic() < deadline:
        waiting = int(ser.in_waiting or 0)
        if waiting <= 0:
            time.sleep(0.001)
            continue
        chunk = ser.read(waiting)
        if not chunk:
            continue
        buffer += chunk
        raw_log.append(
            {
                "at": time.time(),
                "bytes_hex": " ".join(f"{byte:02X}" for byte in chunk),
                "text": chunk.decode(errors="replace"),
                "buffer": buffer.decode(errors="replace"),
            }
        )
        if buffer == b"o":
            stats.partial_o_count += 1
        normalized = buffer.replace(b"\r", b"\n")
        if b"\n" not in normalized:
            continue
        line, _remainder = normalized.split(b"\n", 1)
        return line.decode(errors="replace").strip()
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Non-browser strict ACK GRBL serial stress test")
    parser.add_argument("--port", required=True, help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=10000)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--scenario", choices=["simple", "pen-toggle", "zero-motion"], default="simple")
    parser.add_argument("--command", default="G4 P0.001")
    parser.add_argument("--raw-log-limit", type=int, default=200)
    args = parser.parse_args()

    stats = AckStats()
    raw_log: list[dict[str, object]] = []

    with serial.Serial(args.port, args.baud, timeout=0) as ser:
        ser.reset_input_buffer()
        ser.write(b"\r\n\r\n")
        time.sleep(2.0)
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        for command in scenario_commands(args.scenario, args.count, args.command):
            payload = f"{command}\n".encode("ascii")
            started = time.monotonic()
            ser.write(payload)
            stats.commands_sent += 1
            line = read_complete_line(ser, args.timeout, raw_log, stats)
            latency_ms = (time.monotonic() - started) * 1000.0
            stats.max_ack_latency_ms = max(stats.max_ack_latency_ms, latency_ms)
            stats.total_ack_latency_ms += latency_ms
            if line is None:
                stats.timeout_count += 1
                break
            if line == "ok":
                stats.ok_count += 1
            elif line.lower().startswith("error") or line.startswith("ALARM:"):
                stats.error_count += 1
                break

    result = {
        "port": args.port,
        "baud": args.baud,
        "scenario": args.scenario,
        "requested_count": args.count,
        "commands_sent": stats.commands_sent,
        "ok_count": stats.ok_count,
        "error_count": stats.error_count,
        "timeout_count": stats.timeout_count,
        "partial_o_count": stats.partial_o_count,
        "max_ack_latency_ms": round(stats.max_ack_latency_ms, 3),
        "average_ack_latency_ms": round(stats.total_ack_latency_ms / max(1, stats.commands_sent), 3),
        "raw_log_tail": raw_log[-args.raw_log_limit :],
    }
    print(json.dumps(result, indent=2))
    return 1 if stats.timeout_count or stats.error_count else 0


if __name__ == "__main__":
    raise SystemExit(main())
