from __future__ import annotations

import threading
import time
from typing import Any, Callable

from .gcode_service import GcodeService

JobFinalizationReason = str


class JobRunner:
    def __init__(self, state, serial_service, config) -> None:
        self.state = state
        self.serial_service = serial_service
        self.config = config
        self.gcode_service = GcodeService()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._stop_reason: JobFinalizationReason = "abort"
        self._pause_requested = False
        self._before_start: Callable[[], None] | None = None
        self._lifecycle_callback: Callable[[str, dict[str, Any]], None] | None = None

    def set_before_start(self, callback: Callable[[], None]) -> None:
        self._before_start = callback

    def set_lifecycle_callback(self, callback: Callable[[str, dict[str, Any]], None]) -> None:
        self._lifecycle_callback = callback

    def _emit_lifecycle(self, event: str, **payload: Any) -> None:
        if self._lifecycle_callback is not None:
            self._lifecycle_callback(event, payload)

    def start(self) -> None:
        if self._before_start is not None:
            self._before_start()
        snapshot = self.state.snapshot()
        if snapshot["running"]:
            raise ValueError("A job is already running")
        if not snapshot["calibrated"]:
            raise ValueError("Machine is not calibrated. Jog to the ball center, then use 'Set Origin & Calibrate'.")
        if not snapshot["last_gcode"]:
            raise ValueError("No G-code generated yet")
        with self._lock:
            self._stop_requested = False
            self._stop_reason = "abort"
            self._pause_requested = False
            self._thread = threading.Thread(target=self._worker, args=(list(snapshot["last_gcode"]),), daemon=True)
            self._thread.start()
        self._emit_lifecycle("job_start")

    def pause(self) -> None:
        with self._lock:
            self._pause_requested = True
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"!")
        snapshot = self.state.snapshot()
        pause_started_at = snapshot.get("pause_started_at") or time.time()
        self.state.update(paused=True, status="Feed hold requested", pause_started_at=pause_started_at)
        self._emit_lifecycle("pause_job")

    def resume(self) -> None:
        snapshot = self.state.snapshot()
        pause_started_at = snapshot.get("pause_started_at")
        paused_duration_seconds = float(snapshot.get("paused_duration_seconds") or 0.0)
        if pause_started_at:
            paused_duration_seconds += max(0.0, time.time() - float(pause_started_at))
        with self._lock:
            self._pause_requested = False
        self._emit_lifecycle("resume_job")
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"~")
        self.state.update(
            paused=False,
            status="Resume requested",
            pause_started_at=None,
            paused_duration_seconds=paused_duration_seconds,
        )

    def request_stop(self, *, reason: JobFinalizationReason = "abort") -> None:
        with self._lock:
            self._stop_requested = True
            self._stop_reason = reason
            self._pause_requested = False

    @staticmethod
    def _resolve_preview_progress(preview_paths: list[dict], stream_line: int) -> tuple[str | None, str | None, int]:
        for entry in preview_paths:
            start_line = entry.get("gcode_start_line")
            end_line = entry.get("gcode_end_line")
            if start_line is None or end_line is None:
                continue
            if int(start_line) <= stream_line <= int(end_line):
                points = entry.get("points") or []
                if len(points) < 2:
                    return entry.get("id"), entry.get("kind"), 0
                segment_index = max(0, min(stream_line - int(start_line), len(points) - 2))
                return entry.get("id"), entry.get("kind"), segment_index + 1
        return None, None, 0

    def finalize_job(self, reason: JobFinalizationReason, *, machine_position_trusted: bool, not_in_alarm: bool = True) -> dict:
        snapshot = self.state.snapshot()
        serial_connected = bool(snapshot.get("connected") and self.serial_service.has_live_serial())
        pen_up_attempted = False
        pen_up_ok = False
        home_attempted = False
        home_ok = False
        motor_hold_enabled = bool(snapshot.get("connected") and snapshot.get("calibrated"))

        can_return_home = {
            "calibration_locked": bool(snapshot.get("calibrated")),
            "serial_connected": serial_connected,
            "machine_position_trusted": bool(machine_position_trusted),
            "not_in_alarm": bool(not_in_alarm),
            "not_emergency_stopped": not reason == "emergency_stop",
        }
        should_home = reason == "complete" or (reason in {"abort", "fail", "timeout", "grbl_error"} and all(can_return_home.values()))

        if serial_connected:
            with self.serial_service.lock:
                ser = self.serial_service.get_serial()
                try:
                    pen_up_attempted = True
                    self.serial_service.send_to_grbl_unlocked(ser, "$X", timeout=10)
                    self.serial_service.send_to_grbl_unlocked(ser, f"M3 S{self.config['DEFAULT_PEN_UP_S']}", timeout=10)
                    self.serial_service.send_to_grbl_unlocked(
                        ser,
                        f"G4 P{max(0.0, float(self.config['DEFAULT_PEN_UP_DWELL_MS'])) / 1000.0:.3f}",
                        timeout=10,
                    )
                    pen_up_ok = True
                    self.state.set_servo(int(self.config["DEFAULT_PEN_UP_S"]))
                except Exception:
                    pen_up_ok = False

                if should_home:
                    try:
                        home_attempted = True
                        self.serial_service.send_to_grbl_unlocked(ser, "G21", timeout=10)
                        self.serial_service.send_to_grbl_unlocked(ser, "G90", timeout=10)
                        self.serial_service.send_to_grbl_unlocked(
                            ser,
                            f"G0 X0 Y0 F{float(self.config['DEFAULT_TRAVEL_FEED']):.3f}",
                            timeout=20,
                        )
                        home_ok = True
                        self.state.update(current_position_x=0.0, current_position_y=0.0)
                    except Exception:
                        home_ok = False

        result = {
            "job_finalization": {
                "reason": reason,
                "serial_connected": serial_connected,
                "pen_up_attempted": pen_up_attempted,
                "pen_up_ok": pen_up_ok,
                "home_attempted": home_attempted,
                "home_ok": home_ok,
                "motor_hold_enabled": motor_hold_enabled,
                "can_return_home": can_return_home,
            }
        }
        self.state.update(
            motor_hold_enabled=motor_hold_enabled,
            last_job_finalization=result["job_finalization"],
            machine_position_trusted=bool(machine_position_trusted if not home_ok else True),
            emergency_stopped=reason == "emergency_stop",
            status="Job finalized. Pen up, returned home, motor hold preserved."
            if pen_up_ok and (home_ok or not home_attempted) and motor_hold_enabled
            else f"Job finalized with {reason}.",
        )
        return result

    def _worker(self, gcode: list[str]) -> None:
        stream_lines = [line for line in gcode if self.gcode_service.is_streamable_line(line)]
        preview_paths = list(self.state.snapshot().get("last_preview") or [])
        final_reason: JobFinalizationReason = "complete"
        machine_position_trusted = True
        not_in_alarm = True
        try:
            started_at = time.time()
            self.state.update(
                running=True,
                paused=False,
                status="Running",
                progress_total=len(stream_lines),
                progress_done=0,
                last_error=None,
                run_started_at=started_at,
                pause_started_at=None,
                paused_duration_seconds=0.0,
                current_gcode_line=0,
                current_path_id=None,
                current_preview_point_index=0,
                streaming={
                    "mode": self.state.snapshot().get("streaming_mode", "buffered"),
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
                },
            )
            with self.serial_service.lock:
                ser = self.serial_service.get_serial()
                self.serial_service.send_to_grbl_unlocked(ser, "$X", timeout=10)

                def should_stop() -> bool:
                    with self._lock:
                        if self._stop_requested:
                            self.state.update(status="Stopping")
                            return True
                    return False

                def wait_while_paused() -> None:
                    with self._lock:
                        paused = self._pause_requested
                    while paused:
                        pause_snapshot = self.state.snapshot()
                        pause_started_at = pause_snapshot.get("pause_started_at") or time.time()
                        self.state.update(paused=True, status="Paused", pause_started_at=pause_started_at)
                        time.sleep(0.1)
                        with self._lock:
                            if self._stop_requested:
                                self.state.update(status="Stopping")
                                return
                            paused = self._pause_requested
                    self.state.update(paused=False)

                def on_line_sent(line: str, sent_count: int) -> None:
                    current_path_id, current_path_kind, current_preview_point_index = self._resolve_preview_progress(preview_paths, sent_count)
                    self.state.update(
                        status=f"Running: {line}",
                        progress_done=sent_count,
                        current_gcode_line=sent_count,
                        current_path_id=current_path_id,
                        current_path_kind=current_path_kind,
                        current_preview_point_index=current_preview_point_index,
                    )

                self.serial_service.stream_gcode_lines_unlocked(
                    ser,
                    stream_lines,
                    response_timeout=20,
                    should_stop=should_stop,
                    wait_while_paused=wait_while_paused,
                    on_line_sent=on_line_sent,
                )
                with self._lock:
                    if self._stop_requested:
                        final_reason = self._stop_reason
                    else:
                        idle_ok = self.serial_service.wait_until_idle_unlocked(ser, timeout=120)
                        not_in_alarm = idle_ok
                        final_reason = "complete" if idle_ok else "grbl_error"
                        machine_position_trusted = idle_ok
        except Exception as exc:
            timeout_debug = self.state.snapshot().get("last_timeout_debug")
            if isinstance(timeout_debug, dict) and timeout_debug.get("line_index"):
                current_path_id, current_path_kind, _ = self._resolve_preview_progress(preview_paths, int(timeout_debug["line_index"]))
                timeout_debug["current_path_id"] = current_path_id or ""
                timeout_debug["current_path_kind"] = current_path_kind or ""
                self.state.update(last_timeout_debug=timeout_debug)
            message = str(exc)
            final_reason = "timeout" if "Timed out waiting for GRBL response" in message else "grbl_error" if "GRBL streaming error" in message else "fail"
            machine_position_trusted = final_reason not in {"timeout", "fail"}
            not_in_alarm = "ALARM:" not in message
            self.state.update(last_error=message, status=f"Error: {message}")
        finally:
            snapshot = self.state.snapshot()
            paused_duration_seconds = float(snapshot.get("paused_duration_seconds") or 0.0)
            pause_started_at = snapshot.get("pause_started_at")
            if pause_started_at:
                paused_duration_seconds += max(0.0, time.time() - float(pause_started_at))

            finalization = self.finalize_job(
                final_reason,
                machine_position_trusted=machine_position_trusted,
                not_in_alarm=not_in_alarm,
            )

            self.state.update(
                running=False,
                paused=False,
                pause_started_at=None,
                paused_duration_seconds=paused_duration_seconds,
                current_path_id=None,
                current_path_kind=None,
                streaming={
                    "mode": self.state.snapshot().get("streaming_mode", "buffered"),
                    "current_line": 0,
                    "current_path_id": None,
                    "current_path_kind": None,
                    "pending_buffer_chars": 0,
                    "pending_commands": 0,
                    "last_response_age_sec": 0.0,
                    "last_grbl_status": None,
                    "ok_count": 0,
                    "error_count": 0,
                    "sent_count": snapshot.get("current_gcode_line", 0),
                },
                last_job_finalization=finalization["job_finalization"],
            )
            self._emit_lifecycle("finalize_job", reason=final_reason)
            finalization["job_finalization"]["motor_hold_enabled"] = bool(self.state.snapshot().get("motor_hold_enabled"))
            with self._lock:
                self._stop_requested = False
                self._stop_reason = "abort"
                self._pause_requested = False
