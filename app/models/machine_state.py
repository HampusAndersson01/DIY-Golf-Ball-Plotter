from __future__ import annotations

import os
import threading
from dataclasses import dataclass, field
from typing import Any


@dataclass
class MachineState:
    default_pen_up_s: int
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)
    _data: dict[str, Any] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._data = {
            "connected": False,
            "calibrated": False,
            "running": False,
            "paused": False,
            "status": "Not connected",
            "last_svg_name": None,
            "last_gcode": [],
            "last_preview": [],
            "last_error": None,
            "progress_total": 0,
            "progress_done": 0,
            "run_started_at": None,
            "pause_started_at": None,
            "paused_duration_seconds": 0.0,
            "current_servo_s": self.default_pen_up_s,
            "server_pid": os.getpid(),
        }

    @property
    def raw(self) -> dict[str, Any]:
        return self._data

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._data)

    def update(self, **kwargs: Any) -> None:
        with self._lock:
            self._data.update(kwargs)

    def get_servo(self, fallback: int) -> int:
        with self._lock:
            value = self._data.get("current_servo_s", fallback)
        try:
            return int(value)
        except (TypeError, ValueError):
            return fallback

    def set_servo(self, s_value: int) -> None:
        self.update(current_servo_s=int(s_value))
