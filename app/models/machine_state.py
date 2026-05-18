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
            "machine_position_trusted": False,
            "emergency_stopped": False,
            "running": False,
            "paused": False,
            "status": "Not connected",
            "last_svg_name": None,
            "last_gcode": [],
            "last_preview": [],
            "last_error": None,
            "last_timeout_debug": None,
            "progress_total": 0,
            "progress_done": 0,
            "run_started_at": None,
            "pause_started_at": None,
            "paused_duration_seconds": 0.0,
            "current_gcode_line": 0,
            "current_path_id": None,
            "current_path_kind": None,
            "current_preview_point_index": 0,
            "current_servo_s": self.default_pen_up_s,
            "current_position_x": 0.0,
            "current_position_y": 0.0,
            "motor_hold_enabled": False,
            "motors": {
                "method": "grbl_$1_step_idle_delay",
                "connected": False,
                "calibration_locked": False,
                "policy": "release_before_calibration",
                "hold_active": False,
                "desired_dollar_1": None,
                "applied_dollar_1": None,
                "last_known_dollar_1": None,
                "x_expected_holding": False,
                "y_expected_holding": False,
                "applying": False,
                "queued_apply_reason": None,
                "last_apply_reason": None,
                "last_apply_ok": None,
                "last_error": None,
            },
            "stepper_hold_debug": {
                "reason": None,
                "connected": False,
                "calibration_locked": False,
                "desired_policy": "release_before_calibration",
                "desired_dollar_1": None,
                "previous_applied_dollar_1": None,
                "new_applied_dollar_1": None,
                "readback_dollar_1": None,
                "streaming_active": False,
                "queued": False,
                "ok": None,
            },
            "last_job_finalization": None,
            "last_stream_event": None,
            "streaming_mode": "buffered",
            "streaming": {
                "mode": "buffered",
                "current_line": 0,
                "current_path_id": None,
                "current_path_kind": None,
                "pending_buffer_chars": 0,
                "pending_commands": 0,
                "last_response_age_sec": 0.0,
                "last_grbl_status": None,
                "ok_count": 0,
                "error_count": 0,
                "sent_count": 0,
                "acked_count": 0,
                "total_lines": 0,
                "streaming_active": False,
            },
            "job": {
                "status": "idle",
                "finalized_successfully": False,
                "current_line": 0,
                "total_lines": 0,
                "sent_line_count": 0,
                "acked_line_count": 0,
                "remaining_lines": 0,
                "completion_guard_passed": False,
                "finalization_reason": None,
                "cleanup_status": "idle",
                "pen_state": "up",
                "machine_returned_home": False,
                "motor_hold_preserved": False,
                "abort_requested": False,
                "error": None,
                "streaming_active": False,
                "pending_queue_length": 0,
                "pending_buffer_chars": 0,
                "machine_state": None,
                "finalization_debug": None,
                "premature_finalization_debug": None,
                "streaming_controller": "backend_thread",
            },
            "last_summary": None,
            "y_loop_test": {
                "enabled": False,
                "center_y": 0.0,
                "distance": 10.0,
                "feedrate": 1200.0,
                "dwell_sec": 0.25,
                "phase": "idle",
                "cycles_completed": 0,
            },
            "movement_test": {
                "active": False,
                "axis": "Y",
                "x_motor_holding": False,
                "y_motor_holding": False,
                "amplitude_deg": 10.0,
                "feedrate": 1200.0,
                "cycle_count": 0,
            },
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
