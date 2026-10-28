from __future__ import annotations

from copy import copy
import errno
import logging
import os
import sys
import threading
import time
from typing import Any

import serial
from serial.tools import list_ports

from . import pipeline_core


class SerialService:
    def __init__(self, config: dict[str, Any], state) -> None:
        self.logger = logging.getLogger(__name__)
        self.config = config
        self.state = state
        self.lock = threading.RLock()
        pipeline_core.configure_runtime(config, state.raw, self.lock)
        self.logger.info(
            "SerialService initialized: port=%s baud=%s streaming_mode=%s",
            config.get("SERIAL_PORT"),
            config.get("BAUD_RATE"),
            config.get("DEFAULT_STREAMING_MODE"),
        )

    def list_ports(self) -> dict[str, Any]:
        ports = sorted(list_ports.comports(), key=lambda port: port.device)
        configured = self.config.get("SERIAL_PORT", "")
        if configured and not any(port.device == configured for port in ports):
            for port in list(ports):
                if os.path.realpath(configured) == os.path.realpath(port.device):
                    # Preserve stable /dev/serial/by-id aliases in the selector.
                    alias = copy(port)
                    alias.device = configured
                    ports.insert(0, alias)
                    break
        devices = [port.device for port in ports]
        usb_devices = {os.path.realpath(port.device): port.device for port in ports if port.vid is not None}
        selected = configured if configured in devices else (next(iter(usb_devices.values())) if len(usb_devices) == 1 else "")
        return {
            "ports": [{"device": port.device, "description": port.description, "hwid": port.hwid} for port in ports],
            "selected_port": selected,
            "configured_port": configured,
        }

    def connect(self, port: str | None = None):
        with self.lock:
            if port is not None and not isinstance(port, str):
                raise ValueError("Select a valid serial port.")
            selected = (port if port is not None else self.config.get("SERIAL_PORT", "")).strip()
            if self.has_live_serial():
                if selected and selected != self.config.get("SERIAL_PORT"):
                    raise ValueError("Already connected. Restart the backend before changing serial ports.")
                return pipeline_core.grbl
            if not selected:
                selected = self.list_ports()["selected_port"]
            if not selected:
                raise ValueError("Select a serial port from the Connection panel. Refresh ports after plugging in the USB controller.")
            self.config["SERIAL_PORT"] = selected
            pipeline_core.configure_runtime(self.config, self.state.raw, self.lock)
            self.logger.info("Opening serial connection to GRBL on %s", selected)
            try:
                ser = pipeline_core.connect_grbl()
            except serial.SerialException as exc:
                if sys.platform.startswith("linux") and exc.errno == errno.EACCES:
                    raise serial.SerialException(
                        f"Permission denied opening {selected}. Add your Linux user to the device's serial group "
                        '(usually dialout): sudo usermod -aG dialout "$USER". Log out and back in, then restart the backend.'
                    ) from exc
                raise
            pipeline_core.grbl = ser
            self.logger.info("Serial connection ready: is_open=%s", getattr(ser, "is_open", True))
            return ser

    def send_command(self, command: str, timeout: float = 15):
        pipeline_core.configure_runtime(self.config, self.state.raw, self.lock)
        self.logger.info("Sending GRBL command: %s", command)
        return pipeline_core.send_to_grbl(command, timeout=timeout)

    def send_many(self, commands: list[str], delay: float = 0.04, wait_idle_between: bool = True):
        pipeline_core.configure_runtime(self.config, self.state.raw, self.lock)
        self.logger.info(
            "Sending %d GRBL commands (delay=%.3fs wait_idle_between=%s)",
            len(commands),
            delay,
            wait_idle_between,
        )
        return pipeline_core.send_many(commands, delay=delay, wait_idle_between=wait_idle_between)

    def read_available_lines(self, ser):
        return pipeline_core.read_available_lines(ser)

    def wait_until_idle_unlocked(self, ser, timeout: float = 60):
        return pipeline_core.wait_until_idle_unlocked(ser, timeout=timeout)

    def send_to_grbl_unlocked(self, ser, command: str, timeout: float = 15):
        self.logger.debug("Sending unlocked GRBL command: %s", command)
        return pipeline_core.send_to_grbl_unlocked(ser, command, timeout=timeout)

    def stream_gcode_lines_unlocked(
        self,
        ser,
        lines: list[str],
        *,
        response_timeout: float = 20,
        should_stop=None,
        wait_while_paused=None,
        on_line_sent=None,
    ):
        self.logger.info(
            "Streaming %d G-code lines to GRBL (response_timeout=%.2fs)",
            len(lines),
            response_timeout,
        )
        return pipeline_core.stream_gcode_lines_unlocked(
            ser,
            lines,
            response_timeout=response_timeout,
            should_stop=should_stop,
            wait_while_paused=wait_while_paused,
            on_line_sent=on_line_sent,
        )

    def get_serial(self):
        return self.connect()

    def has_live_serial(self) -> bool:
        ser = getattr(pipeline_core, "grbl", None)
        return bool(ser and getattr(ser, "is_open", True))

    def soft_reset(self, *, settle_seconds: float = 1.0) -> list[str]:
        with self.lock:
            ser = self.get_serial()
            self.logger.warning("Sending soft reset to GRBL")
            ser.write(b"\x18")
            time.sleep(settle_seconds)
            lines = self.read_available_lines(ser)
            self.logger.info("Soft reset response lines=%d", len(lines))
            return lines
