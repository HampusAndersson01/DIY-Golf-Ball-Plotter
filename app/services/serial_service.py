from __future__ import annotations

import logging
import time
import threading
from typing import Any

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

    def connect(self):
        with self.lock:
            self.logger.info("Opening serial connection to GRBL on %s", self.config.get("SERIAL_PORT"))
            ser = pipeline_core.connect_grbl()
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
