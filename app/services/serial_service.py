from __future__ import annotations

import threading
from typing import Any

from ._legacy import configure_runtime, legacy


class SerialService:
    def __init__(self, config: dict[str, Any], state) -> None:
        self.config = config
        self.state = state
        self.lock = threading.RLock()
        configure_runtime(config, state.raw, self.lock)

    def connect(self):
        with self.lock:
            ser = legacy.connect_grbl()
            legacy.grbl = ser
            return ser

    def send_command(self, command: str, timeout: float = 15):
        configure_runtime(self.config, self.state.raw, self.lock)
        return legacy.send_to_grbl(command, timeout=timeout)

    def send_many(self, commands: list[str], delay: float = 0.04, wait_idle_between: bool = True):
        configure_runtime(self.config, self.state.raw, self.lock)
        return legacy.send_many(commands, delay=delay, wait_idle_between=wait_idle_between)

    def read_available_lines(self, ser):
        return legacy.read_available_lines(ser)

    def wait_until_idle_unlocked(self, ser, timeout: float = 60):
        return legacy.wait_until_idle_unlocked(ser, timeout=timeout)

    def send_to_grbl_unlocked(self, ser, command: str, timeout: float = 15):
        return legacy.send_to_grbl_unlocked(ser, command, timeout=timeout)

    def get_serial(self):
        return self.connect()
