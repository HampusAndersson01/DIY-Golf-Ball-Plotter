"""Safe, no-write serial port open/close diagnostic for Windows and PySerial.

Example:
    python scripts/serial_port_diagnostic.py --port COM9 --cycles 20 --output docs/hardware-diagnostics/serial-open-cycles.json

The tool never sends bytes to the controller. It stops after the first failed
cycle by default so a persistent OS/driver failure is not hammered repeatedly.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import re
import sys
import time
from pathlib import Path
from typing import Any

import serial
from serial.tools import list_ports


SERIAL_ARGUMENTS: dict[str, Any] = {
    "baudrate": 115200,
    "bytesize": serial.EIGHTBITS,
    "parity": serial.PARITY_NONE,
    "stopbits": serial.STOPBITS_ONE,
    "timeout": 1,
    "write_timeout": 1,
    "xonxoff": False,
    "rtscts": False,
    "dsrdtr": False,
}


def extract_windows_error_code(error: BaseException) -> int | None:
    """Return a Win32 code, including one nested in PySerial's error text."""
    for current in _exception_chain(error):
        winerror = getattr(current, "winerror", None)
        if isinstance(winerror, int):
            return winerror
        if isinstance(current, OSError) and isinstance(current.errno, int):
            return current.errno
        message = str(current)
        match = re.search(r"\bWinError\s+(\d+)\b", message, re.IGNORECASE)
        if match:
            return int(match.group(1))
        match = re.search(r",\s*(\d+)\)\s*\)?\s*$", message)
        if match:
            return int(match.group(1))
    return None


def _exception_chain(error: BaseException):
    current: BaseException | None = error
    seen: set[int] = set()
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        yield current
        current = current.__cause__ or current.__context__


def classify_windows_serial_error(error: BaseException) -> str:
    code = extract_windows_error_code(error)
    if code in {2, 3}:
        return "device_not_found"
    if code == 5:
        return "access_denied_or_port_in_use"
    if code == 32:
        return "port_already_in_use"
    if code == 31:
        return "driver_or_device_configuration_failure"
    if isinstance(error, PermissionError):
        return "permission_denied"
    return "serial_open_error"


def enumerate_ports() -> list[dict[str, Any]]:
    return [
        {
            "device": port.device,
            "description": port.description,
            "hwid": port.hwid,
            "vid": port.vid,
            "pid": port.pid,
            "serial_number": port.serial_number,
            "location": port.location,
        }
        for port in list_ports.comports()
    ]


def run_cycle(port_name: str, cycle: int) -> dict[str, Any]:
    started_at = datetime.now(timezone.utc)
    timer = time.perf_counter()
    result: dict[str, Any] = {
        "cycle": cycle,
        "started_at_utc": started_at.isoformat(),
        "port": port_name,
        "serial_arguments": SERIAL_ARGUMENTS.copy(),
        "bytes_written": 0,
        "opened": False,
        "closed": False,
    }
    connection: serial.Serial | None = None
    try:
        connection = serial.Serial(port=port_name, **SERIAL_ARGUMENTS)
        result["opened"] = connection.is_open
        connection.close()
        result["closed"] = not connection.is_open
    except Exception as error:  # Diagnostic output must preserve driver exceptions.
        result.update(
            {
                "exception_type": type(error).__name__,
                "exception": str(error),
                "windows_error_code": extract_windows_error_code(error),
                "classification": classify_windows_serial_error(error),
            }
        )
        if connection is not None and connection.is_open:
            try:
                connection.close()
                result["closed_after_error"] = not connection.is_open
            except Exception as close_error:
                result["close_exception"] = f"{type(close_error).__name__}: {close_error}"
    result["duration_ms"] = round((time.perf_counter() - timer) * 1000, 2)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM9", help="Windows serial port (default: COM9)")
    parser.add_argument("--cycles", type=int, default=1, help="Sequential open/close attempts (1..100)")
    parser.add_argument("--continue-after-failure", action="store_true", help="Continue after a failed cycle")
    parser.add_argument("--output", type=Path, help="Optional path for a JSON evidence file")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 1 <= args.cycles <= 100:
        build_parser().error("--cycles must be between 1 and 100")
    ports = enumerate_ports()
    matching_ports = [port for port in ports if port["device"].upper() == args.port.upper()]
    attempts: list[dict[str, Any]] = []
    stop_reason = None
    for cycle in range(1, args.cycles + 1):
        attempt = run_cycle(args.port, cycle)
        attempts.append(attempt)
        if not attempt["opened"] or not attempt["closed"]:
            if not args.continue_after_failure:
                stop_reason = "stopped after first failed open/close cycle"
                break

    result = {
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "device_enumerated": bool(matching_ports),
        "matching_devices": matching_ports,
        "all_enumerated_ports": ports,
        "serial_arguments": SERIAL_ARGUMENTS.copy(),
        "bytes_written": 0,
        "requested_cycles": args.cycles,
        "attempted_cycles": len(attempts),
        "successful_opens": sum(bool(item["opened"]) for item in attempts),
        "successful_closes": sum(bool(item["closed"]) for item in attempts),
        "attempts": attempts,
        "stop_reason": stop_reason,
    }
    serialized = json.dumps(result, indent=2, ensure_ascii=False)
    print(serialized)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized + "\n", encoding="utf-8")
    return 0 if len(attempts) == args.cycles and all(item["opened"] and item["closed"] for item in attempts) else 1


if __name__ == "__main__":
    raise SystemExit(main())
