from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

from .gcode_service import GcodeService
from .runtime_estimation_service import build_runtime_snapshot

JobFinalizationReason = str


class JobRunner:
    def __init__(self, state, serial_service, config) -> None:
        self.logger = logging.getLogger(__name__)
        self.state = state
        self.serial_service = serial_service
        self.config = config
        self.gcode_service = GcodeService()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._stop_reason: JobFinalizationReason = "user_stop"
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
        with self._lock:
            snapshot = self.state.snapshot()
            if snapshot["running"] or (self._thread is not None and self._thread.is_alive()):
                raise ValueError("A job is already running")
            if not snapshot["calibrated"]:
                raise ValueError("Machine is not calibrated. Jog to the ball center, then use 'Set Origin & Calibrate'.")
            if not snapshot["last_gcode"]:
                raise ValueError("No G-code generated yet")
            self._stop_requested = False
            self._stop_reason = "user_stop"
            self._pause_requested = False
            started_at = time.time()
            self.state.update(
                running=True,
                paused=False,
                status="Starting",
                job_state="starting",
                progress_total=0,
                progress_done=0,
                current_gcode_line=0,
                current_path_id=None,
                current_path_kind=None,
                current_preview_point_index=0,
                run_started_at=started_at,
                run_finished_at=None,
                job_started_at=started_at,
                job_finished_at=None,
                pause_started_at=None,
                paused_duration_seconds=0.0,
                job_elapsed_seconds=0.0,
                job_estimated_remaining_seconds=max(0.0, float((snapshot.get("last_summary") or {}).get("estimated_runtime_seconds") or 0.0)),
                job_estimated_total_seconds=max(0.0, float((snapshot.get("last_summary") or {}).get("estimated_runtime_seconds") or 0.0)),
                runtime_estimate_multiplier=1.0,
                last_error=None,
                last_stream_event="job_start_requested",
            )
            try:
                self._thread = threading.Thread(target=self._worker, args=(list(snapshot["last_gcode"]),), daemon=True)
                self._thread.start()
            except Exception:
                self.state.update(running=False, status=snapshot.get("status") or "Idle")
                self._thread = None
                raise
        self.logger.info("Job start requested with %d total G-code lines", len(snapshot["last_gcode"]))
        self._emit_lifecycle("job_start")

    def pause(self) -> None:
        with self._lock:
            self._pause_requested = True
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"!")
        snapshot = self.state.snapshot()
        pause_started_at = snapshot.get("pause_started_at") or time.time()
        timing = build_runtime_snapshot({**snapshot, "paused": True, "pause_started_at": pause_started_at, "job_state": "paused"}, now_seconds=pause_started_at)
        self.state.update(paused=True, status="Feed hold requested", job_state="paused", pause_started_at=pause_started_at, **timing)
        self.logger.info("Job pause requested")
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
            job_state="running",
            pause_started_at=None,
            paused_duration_seconds=paused_duration_seconds,
            **build_runtime_snapshot({**snapshot, "paused": False, "pause_started_at": None, "paused_duration_seconds": paused_duration_seconds, "job_state": "running"}),
        )
        self.logger.info("Job resume requested")

    def request_stop(self, *, reason: JobFinalizationReason = "user_stop") -> None:
        with self._lock:
            self._stop_requested = True
            self._stop_reason = "user_stop" if reason == "abort" else reason
            self._pause_requested = False
        self.logger.warning("Job stop requested with reason=%s", self._stop_reason)

    @staticmethod
    def _build_job_snapshot(
        *,
        snapshot: dict[str, Any],
        total_lines: int | None = None,
        sent_line_count: int | None = None,
        acked_line_count: int | None = None,
        pending_queue_length: int | None = None,
        pending_buffer_chars: int | None = None,
        status: str | None = None,
        finalization_reason: str | None = None,
        finalized_successfully: bool | None = None,
        cleanup_status: str | None = None,
        pen_state: str | None = None,
        machine_returned_home: bool | None = None,
        motor_hold_preserved: bool | None = None,
        abort_requested: bool | None = None,
        error: str | None = None,
        completion_guard_passed: bool | None = None,
        finalization_debug: dict[str, Any] | None = None,
        premature_finalization_debug: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        existing = dict(snapshot.get("job") or {})
        total = int(total_lines if total_lines is not None else existing.get("total_lines") or snapshot.get("progress_total") or 0)
        sent = int(sent_line_count if sent_line_count is not None else existing.get("sent_line_count") or snapshot.get("streaming", {}).get("sent_count") or snapshot.get("current_gcode_line") or 0)
        acked = int(acked_line_count if acked_line_count is not None else existing.get("acked_line_count") or snapshot.get("streaming", {}).get("acked_count") or 0)
        pending_queue = int(pending_queue_length if pending_queue_length is not None else existing.get("pending_queue_length") or snapshot.get("streaming", {}).get("pending_commands") or 0)
        pending_buffer = int(pending_buffer_chars if pending_buffer_chars is not None else existing.get("pending_buffer_chars") or snapshot.get("streaming", {}).get("pending_buffer_chars") or 0)
        remaining = max(0, total - acked)
        job = {
            **existing,
            "status": status if status is not None else existing.get("status", "idle"),
            "finalized_successfully": bool(finalized_successfully) if finalized_successfully is not None else bool(existing.get("finalized_successfully", False)),
            "current_line": acked,
            "total_lines": total,
            "sent_line_count": sent,
            "acked_line_count": acked,
            "remaining_lines": remaining,
            "completion_guard_passed": bool(completion_guard_passed) if completion_guard_passed is not None else bool(existing.get("completion_guard_passed", False)),
            "finalization_reason": finalization_reason if finalization_reason is not None else existing.get("finalization_reason"),
            "cleanup_status": cleanup_status if cleanup_status is not None else existing.get("cleanup_status", "idle"),
            "pen_state": pen_state if pen_state is not None else existing.get("pen_state", "up"),
            "machine_returned_home": bool(machine_returned_home) if machine_returned_home is not None else bool(existing.get("machine_returned_home", False)),
            "motor_hold_preserved": bool(motor_hold_preserved) if motor_hold_preserved is not None else bool(existing.get("motor_hold_preserved", False)),
            "abort_requested": bool(abort_requested) if abort_requested is not None else bool(existing.get("abort_requested", False)),
            "error": error if error is not None else existing.get("error"),
            "streaming_active": bool(snapshot.get("streaming", {}).get("streaming_active", False)),
            "pending_queue_length": pending_queue,
            "pending_buffer_chars": pending_buffer,
            "machine_state": snapshot.get("streaming", {}).get("last_grbl_status") or snapshot.get("status"),
            "finalization_debug": finalization_debug if finalization_debug is not None else existing.get("finalization_debug"),
            "premature_finalization_debug": premature_finalization_debug if premature_finalization_debug is not None else existing.get("premature_finalization_debug"),
            "streaming_controller": "backend_thread",
        }
        return job

    @staticmethod
    def _can_mark_job_complete(job: dict[str, Any]) -> bool:
        return (
            int(job.get("sent_line_count", 0)) >= int(job.get("total_lines", 0))
            and int(job.get("acked_line_count", 0)) >= int(job.get("total_lines", 0))
            and int(job.get("pending_queue_length", 0)) == 0
            and int(job.get("pending_buffer_chars", 0)) == 0
            and str(job.get("machine_state") or "").startswith("Idle")
            and not bool(job.get("abort_requested"))
            and not job.get("error")
        )

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
        if reason == "complete":
            reason = "completed_all_lines"
        elif reason == "abort":
            reason = "user_stop"
        self.logger.info(
            "Finalizing job: reason=%s machine_position_trusted=%s not_in_alarm=%s",
            reason,
            machine_position_trusted,
            not_in_alarm,
        )
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
            "not_emergency_stopped": reason != "emergency_stop",
        }
        should_home = reason == "completed_all_lines" or (reason in {"user_stop", "unknown_interruption", "fail", "timeout", "grbl_error", "sender_desync", "connection_lost", "page_sleep_or_timer_gap"} and all(can_return_home.values()))

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
                    self.logger.exception("Failed during pen-up cleanup")
                    pen_up_ok = False

                if should_home:
                    try:
                        home_attempted = True
                        self.serial_service.send_to_grbl_unlocked(ser, "G21", timeout=10)
                        self.serial_service.send_to_grbl_unlocked(ser, "G90", timeout=10)
                        self.serial_service.send_to_grbl_unlocked(
                            ser,
                            "G0 X0 Y0",
                            timeout=20,
                        )
                        home_ok = True
                        self.state.update(current_position_x=0.0, current_position_y=0.0)
                    except Exception:
                        self.logger.exception("Failed while returning machine home during cleanup")
                        home_ok = False

        current_streaming = dict(snapshot.get("streaming") or {})
        job = self._build_job_snapshot(
            snapshot=snapshot,
            total_lines=int(current_streaming.get("total_lines") or snapshot.get("progress_total") or 0),
            sent_line_count=int(current_streaming.get("sent_count") or snapshot.get("current_gcode_line") or 0),
            acked_line_count=int(current_streaming.get("acked_count") or 0),
            pending_queue_length=int(current_streaming.get("pending_commands") or 0),
            pending_buffer_chars=int(current_streaming.get("pending_buffer_chars") or 0),
            abort_requested=bool(self._stop_requested),
            error=snapshot.get("last_error"),
        )
        completed = self._can_mark_job_complete(job)
        remaining_lines = max(0, int(job["total_lines"]) - int(job["acked_line_count"]))
        premature_finalization_debug = None
        attempted_success_message = "Job finalized. Pen up, returned home, motor hold preserved."
        if reason == "completed_all_lines" and not completed:
            premature_finalization_debug = {
                "attempted_success_message": attempted_success_message,
                "allowed_to_mark_complete": False,
                "current_line": int(job["acked_line_count"]),
                "total_lines": int(job["total_lines"]),
                "remaining_lines": remaining_lines,
                "sent_line_count": int(job["sent_line_count"]),
                "acked_line_count": int(job["acked_line_count"]),
                "pending_queue_length": int(job["pending_queue_length"]),
                "pending_buffer_chars": int(job["pending_buffer_chars"]),
                "machine_state": job.get("machine_state"),
                "streaming_active": bool(job.get("streaming_active")),
                "job_abort_requested": bool(job.get("abort_requested")),
                "last_grbl_response": current_streaming.get("last_grbl_status"),
                "last_stream_event": snapshot.get("last_stream_event"),
                "finalization_reason": "unknown_interruption",
            }
            reason = "unknown_interruption"

        cleanup_status = "complete" if pen_up_ok and (home_ok or not home_attempted) else "partial"
        final_status = "complete" if reason == "completed_all_lines" and completed else "stopped" if reason == "user_stop" else "error"
        finalized_successfully = final_status == "complete"
        if finalized_successfully:
            message = "Print completed. Pen up, returned home, motor hold preserved."
        elif reason == "user_stop":
            message = f"Print stopped before completion at line {int(job['acked_line_count'])} / {int(job['total_lines'])}. Cleanup completed: pen up, returned home, motor hold preserved." if cleanup_status == "complete" else f"Print stopped before completion at line {int(job['acked_line_count'])} / {int(job['total_lines'])}. Cleanup status: {cleanup_status}."
        elif reason == "unknown_interruption":
            message = f"Machine is idle but sender has not completed all lines. Treating as stream interruption, not success. Line {int(job['acked_line_count'])} / {int(job['total_lines'])}. Cleanup completed." if cleanup_status == "complete" else f"Machine is idle but sender has not completed all lines. Cleanup status: {cleanup_status}."
        elif reason == "connection_lost":
            message = f"Print interrupted before completion due to connection loss. Line {int(job['acked_line_count'])} / {int(job['total_lines'])}. Cleanup completed." if cleanup_status == "complete" else f"Print interrupted before completion due to connection loss. Cleanup status: {cleanup_status}."
        elif reason == "timeout":
            message = f"Print interrupted before completion due to GRBL timeout. Line {int(job['acked_line_count'])} / {int(job['total_lines'])}. Cleanup completed." if cleanup_status == "complete" else f"Print interrupted before completion due to GRBL timeout. Cleanup status: {cleanup_status}."
        else:
            message = f"Print did not complete all G-code lines. Cleanup completed: pen up, returned home, motor hold preserved." if cleanup_status == "complete" else f"Print did not complete all G-code lines. Cleanup status: {cleanup_status}."

        finalization_debug = {
            "reason": reason,
            "completed": completed,
            "currentLine": int(job["acked_line_count"]),
            "sentLineCount": int(job["sent_line_count"]),
            "ackedLineCount": int(job["acked_line_count"]),
            "totalLines": int(job["total_lines"]),
            "remainingLines": remaining_lines,
            "pendingQueueLength": int(job["pending_queue_length"]),
            "pendingBufferChars": int(job["pending_buffer_chars"]),
            "machineState": job.get("machine_state"),
            "abortRequested": bool(job.get("abort_requested")),
            "error": job.get("error"),
            "lastStreamEvent": snapshot.get("last_stream_event"),
        }
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
                "finalized_successfully": finalized_successfully,
                "cleanup_status": cleanup_status,
                "completion_guard_passed": completed,
                "remaining_lines": remaining_lines,
                "finalization_debug": finalization_debug,
                "premature_finalization_debug": premature_finalization_debug,
            }
        }
        updated_job = self._build_job_snapshot(
            snapshot=snapshot,
            status=final_status,
            finalization_reason=reason,
            finalized_successfully=finalized_successfully,
            cleanup_status=cleanup_status,
            pen_state="up" if pen_up_ok else "unknown",
            machine_returned_home=home_ok,
            motor_hold_preserved=motor_hold_enabled,
            abort_requested=bool(self._stop_requested),
            error=snapshot.get("last_error"),
            completion_guard_passed=completed,
            finalization_debug=finalization_debug,
            premature_finalization_debug=premature_finalization_debug,
        )
        final_job_state = "completed" if finalized_successfully else "stopped" if reason == "user_stop" else "failed" if reason == "fail" else "error"
        finished_at = time.time()
        timing_snapshot = {
            **snapshot,
            "paused": False,
            "pause_started_at": None,
            "run_finished_at": finished_at,
            "job_finished_at": finished_at,
            "job_state": final_job_state,
        }
        timing = build_runtime_snapshot(timing_snapshot, now_seconds=finished_at)
        last_summary = snapshot.get("last_summary")
        if isinstance(last_summary, dict):
            updated_summary = dict(last_summary)
            updated_summary["actual_runtime_seconds"] = timing["job_elapsed_seconds"]
            if finalized_successfully and timing["job_estimated_total_seconds"] > 0:
                updated_summary["actual_vs_estimated_ratio"] = timing["runtime_estimate_multiplier"]
            last_summary = updated_summary
        self.state.update(
            motor_hold_enabled=motor_hold_enabled,
            job=updated_job,
            last_job_finalization=result["job_finalization"],
            machine_position_trusted=bool(machine_position_trusted if not home_ok else True),
            emergency_stopped=reason == "emergency_stop",
            run_finished_at=finished_at,
            job_finished_at=finished_at,
            job_state=final_job_state,
            last_summary=last_summary,
            status=message,
            **timing,
        )
        self.logger.info(
            "Job finalization complete: status=%s cleanup_status=%s completed=%s acked=%s total=%s",
            final_status,
            cleanup_status,
            finalized_successfully,
            job["acked_line_count"],
            job["total_lines"],
        )
        return result

    def _worker(self, gcode: list[str]) -> None:
        stream_lines = [line for line in gcode if self.gcode_service.is_streamable_line(line)]
        self.logger.info("Job worker started: streamable_lines=%d", len(stream_lines))
        preview_paths = list(self.state.snapshot().get("last_preview") or [])
        final_reason: JobFinalizationReason = "completed_all_lines"
        machine_position_trusted = True
        not_in_alarm = True
        stream_result: dict[str, Any] = {
            "sent_count": 0,
            "acked_count": 0,
            "total_lines": len(stream_lines),
            "pending_queue_length": 0,
            "pending_buffer_chars": 0,
            "last_grbl_status": None,
            "last_stream_event": "not_started",
            "streaming_active": False,
        }
        try:
            started_at = time.time()
            self.state.update(
                running=True,
                paused=False,
                status="Running",
                job_state="running",
                progress_total=len(stream_lines),
                progress_done=0,
                last_error=None,
                run_started_at=started_at,
                run_finished_at=None,
                job_started_at=started_at,
                job_finished_at=None,
                pause_started_at=None,
                paused_duration_seconds=0.0,
                job_elapsed_seconds=0.0,
                job_estimated_total_seconds=max(0.0, float((self.state.snapshot().get("last_summary") or {}).get("estimated_runtime_seconds") or 0.0)),
                job_estimated_remaining_seconds=max(0.0, float((self.state.snapshot().get("last_summary") or {}).get("estimated_runtime_seconds") or 0.0)),
                runtime_estimate_multiplier=1.0,
                current_gcode_line=0,
                current_path_id=None,
                current_preview_point_index=0,
                last_stream_event="job_started",
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
                    "acked_count": 0,
                    "total_lines": len(stream_lines),
                    "streaming_active": True,
                },
                job=self._build_job_snapshot(
                    snapshot=self.state.snapshot(),
                    total_lines=len(stream_lines),
                    sent_line_count=0,
                    acked_line_count=0,
                    pending_queue_length=0,
                    pending_buffer_chars=0,
                    status="running",
                    finalization_reason=None,
                    finalized_successfully=False,
                    cleanup_status="idle",
                    pen_state="down",
                    machine_returned_home=False,
                    motor_hold_preserved=bool(self.state.snapshot().get("motor_hold_enabled")),
                    abort_requested=False,
                    error=None,
                    completion_guard_passed=False,
                    finalization_debug=None,
                    premature_finalization_debug=None,
                ),
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
                    state_snapshot = self.state.snapshot()
                    current_path_id, current_path_kind, current_preview_point_index = self._resolve_preview_progress(preview_paths, sent_count)
                    if current_path_id is None:
                        current_path_id = state_snapshot.get("current_path_id")
                        current_path_kind = state_snapshot.get("current_path_kind")
                        current_preview_point_index = int(state_snapshot.get("current_preview_point_index") or 0)
                    streaming_snapshot = state_snapshot.get("streaming", {})
                    self.state.update(
                        status=f"Running: {line}",
                        progress_done=sent_count,
                        current_gcode_line=sent_count,
                        current_path_id=current_path_id,
                        current_path_kind=current_path_kind,
                        current_preview_point_index=current_preview_point_index,
                        job=self._build_job_snapshot(
                            snapshot=self.state.snapshot(),
                            total_lines=len(stream_lines),
                            sent_line_count=sent_count,
                            acked_line_count=int(streaming_snapshot.get("acked_count") or 0),
                            pending_queue_length=int(streaming_snapshot.get("pending_commands") or 0),
                            pending_buffer_chars=int(streaming_snapshot.get("pending_buffer_chars") or 0),
                            status="running",
                            abort_requested=False,
                            error=None,
                            completion_guard_passed=False,
                        ),
                        **build_runtime_snapshot({**self.state.snapshot(), "progress_done": sent_count, "job_state": "running"}),
                    )

                stream_result = self.serial_service.stream_gcode_lines_unlocked(
                    ser,
                    stream_lines,
                    response_timeout=20,
                    should_stop=should_stop,
                    wait_while_paused=wait_while_paused,
                    on_line_sent=on_line_sent,
                )
                self.logger.info(
                    "Streaming finished: sent=%s acked=%s pending_queue=%s pending_buffer=%s event=%s",
                    stream_result.get("sent_count"),
                    stream_result.get("acked_count"),
                    stream_result.get("pending_queue_length"),
                    stream_result.get("pending_buffer_chars"),
                    stream_result.get("last_stream_event"),
                )
                self.state.update(
                    progress_done=int(stream_result["acked_count"]),
                    current_gcode_line=int(stream_result["acked_count"]),
                    last_stream_event=stream_result.get("last_stream_event"),
                    job=self._build_job_snapshot(
                        snapshot=self.state.snapshot(),
                        total_lines=int(stream_result["total_lines"]),
                        sent_line_count=int(stream_result["sent_count"]),
                        acked_line_count=int(stream_result["acked_count"]),
                        pending_queue_length=int(stream_result["pending_queue_length"]),
                        pending_buffer_chars=int(stream_result["pending_buffer_chars"]),
                        status="running",
                        abort_requested=bool(self._stop_requested),
                        error=None,
                        completion_guard_passed=False,
                    ),
                    **build_runtime_snapshot({**self.state.snapshot(), "progress_done": int(stream_result["acked_count"]), "job_state": "running"}),
                )
                with self._lock:
                    if self._stop_requested:
                        final_reason = self._stop_reason
                    else:
                        idle_ok = self.serial_service.wait_until_idle_unlocked(ser, timeout=120)
                        not_in_alarm = idle_ok
                        all_lines_sent = int(stream_result["sent_count"]) >= len(stream_lines)
                        all_lines_acked = int(stream_result["acked_count"]) >= len(stream_lines)
                        queue_drained = int(stream_result["pending_queue_length"]) == 0 and int(stream_result["pending_buffer_chars"]) == 0
                        if idle_ok and all_lines_sent and all_lines_acked and queue_drained:
                            stream_result["last_grbl_status"] = "Idle"
                            streaming_snapshot = dict(self.state.snapshot().get("streaming") or {})
                            streaming_snapshot["last_grbl_status"] = "Idle"
                            self.state.update(streaming=streaming_snapshot)
                            final_reason = "completed_all_lines"
                        elif idle_ok:
                            stream_result["last_grbl_status"] = "Idle"
                            streaming_snapshot = dict(self.state.snapshot().get("streaming") or {})
                            streaming_snapshot["last_grbl_status"] = "Idle"
                            self.state.update(streaming=streaming_snapshot)
                            final_reason = "unknown_interruption"
                            self.state.update(last_error="Machine is idle but sender has not completed all lines.")
                        else:
                            final_reason = "grbl_error"
                        machine_position_trusted = idle_ok
        except Exception as exc:
            self.logger.exception("Job worker failed")
            timeout_debug = self.state.snapshot().get("last_timeout_debug")
            if isinstance(timeout_debug, dict) and timeout_debug.get("line_index"):
                current_path_id, current_path_kind, _ = self._resolve_preview_progress(preview_paths, int(timeout_debug["line_index"]))
                timeout_debug["current_path_id"] = current_path_id or ""
                timeout_debug["current_path_kind"] = current_path_kind or ""
                self.state.update(last_timeout_debug=timeout_debug)
            message = str(exc)
            if "Timed out waiting for GRBL response" in message:
                failure_class = ((self.state.snapshot().get("last_timeout_debug") or {}).get("failure_class") or "")
                final_reason = "connection_lost" if failure_class == "communication_lost" else "timeout"
            elif "GRBL streaming error" in message:
                final_reason = "grbl_error"
            else:
                final_reason = "fail"
            machine_position_trusted = final_reason not in {"timeout", "fail", "connection_lost", "unknown_interruption"}
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
                progress_done=int(stream_result.get("acked_count", snapshot.get("current_gcode_line", 0))),
                current_gcode_line=int(stream_result.get("acked_count", snapshot.get("current_gcode_line", 0))),
                streaming={
                    "mode": self.state.snapshot().get("streaming_mode", "buffered"),
                    "current_line": int(stream_result.get("acked_count", snapshot.get("current_gcode_line", 0))),
                    "current_path_id": None,
                    "current_path_kind": None,
                    "pending_buffer_chars": int(stream_result.get("pending_buffer_chars", 0)),
                    "pending_commands": int(stream_result.get("pending_queue_length", 0)),
                    "last_response_age_sec": 0.0,
                    "last_grbl_status": stream_result.get("last_grbl_status"),
                    "ok_count": int(stream_result.get("acked_count", 0)),
                    "error_count": 0,
                    "sent_count": int(stream_result.get("sent_count", snapshot.get("current_gcode_line", 0))),
                    "acked_count": int(stream_result.get("acked_count", 0)),
                    "total_lines": len(stream_lines),
                    "streaming_active": False,
                },
                last_job_finalization=finalization["job_finalization"],
                last_stream_event=stream_result.get("last_stream_event", snapshot.get("last_stream_event")),
                **build_runtime_snapshot({
                    **self.state.snapshot(),
                    "running": False,
                    "paused": False,
                    "pause_started_at": None,
                    "paused_duration_seconds": paused_duration_seconds,
                }),
            )
            self._emit_lifecycle("finalize_job", reason=final_reason)
            finalization["job_finalization"]["motor_hold_enabled"] = bool(self.state.snapshot().get("motor_hold_enabled"))
            with self._lock:
                self._stop_requested = False
                self._stop_reason = "user_stop"
                self._pause_requested = False
            self.logger.info("Job worker finished with final_reason=%s", final_reason)
