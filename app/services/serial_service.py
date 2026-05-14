from __future__ import annotations

import threading
from typing import Any

from . import pipeline_core


class SerialService:
    def __init__(self, config: dict[str, Any], state) -> None:
        self.config = config
        self.state = state
        self.lock = threading.RLock()
        pipeline_core.configure_runtime(config, state.raw, self.lock)

    def connect(self):
        with self.lock:
            ser = pipeline_core.connect_grbl()
            pipeline_core.grbl = ser
            return ser

    def send_command(self, command: str, timeout: float = 15):
        pipeline_core.configure_runtime(self.config, self.state.raw, self.lock)
        return pipeline_core.send_to_grbl(command, timeout=timeout)

    def send_many(self, commands: list[str], delay: float = 0.04, wait_idle_between: bool = True):
        pipeline_core.configure_runtime(self.config, self.state.raw, self.lock)
        return pipeline_core.send_many(commands, delay=delay, wait_idle_between=wait_idle_between)

    def read_available_lines(self, ser):
        return pipeline_core.read_available_lines(ser)

    def wait_until_idle_unlocked(self, ser, timeout: float = 60):
        return pipeline_core.wait_until_idle_unlocked(ser, timeout=timeout)

    def send_to_grbl_unlocked(self, ser, command: str, timeout: float = 15):
        return pipeline_core.send_to_grbl_unlocked(ser, command, timeout=timeout)

    def get_serial(self):
        return self.connect()
