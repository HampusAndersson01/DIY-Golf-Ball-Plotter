from __future__ import annotations

import threading
import time

from .gcode_service import GcodeService


class JobRunner:
    def __init__(self, state, serial_service) -> None:
        self.state = state
        self.serial_service = serial_service
        self.gcode_service = GcodeService()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_requested = False
        self._pause_requested = False

    def start(self) -> None:
        snapshot = self.state.snapshot()
        if snapshot["running"]:
            raise ValueError("A job is already running")
        if not snapshot["calibrated"]:
            raise ValueError("Machine is not calibrated. Jog to the ball center, then use 'Set Origin & Calibrate'.")
        if not snapshot["last_gcode"]:
            raise ValueError("No G-code generated yet")
        with self._lock:
            self._stop_requested = False
            self._pause_requested = False
            self._thread = threading.Thread(target=self._worker, args=(list(snapshot["last_gcode"]),), daemon=True)
            self._thread.start()

    def pause(self) -> None:
        with self._lock:
            self._pause_requested = True
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"!")
        snapshot = self.state.snapshot()
        pause_started_at = snapshot.get("pause_started_at") or time.time()
        self.state.update(paused=True, status="Feed hold requested", pause_started_at=pause_started_at)

    def resume(self) -> None:
        snapshot = self.state.snapshot()
        pause_started_at = snapshot.get("pause_started_at")
        paused_duration_seconds = float(snapshot.get("paused_duration_seconds") or 0.0)
        if pause_started_at:
            paused_duration_seconds += max(0.0, time.time() - float(pause_started_at))
        with self._lock:
            self._pause_requested = False
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"~")
        self.state.update(
            paused=False,
            status="Resume requested",
            pause_started_at=None,
            paused_duration_seconds=paused_duration_seconds,
        )

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            self._pause_requested = False

    def _worker(self, gcode: list[str]) -> None:
        stream_lines = [line for line in gcode if self.gcode_service.is_streamable_line(line)]
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
            )
            with self.serial_service.lock:
                ser = self.serial_service.get_serial()
                def should_stop() -> bool:
                    with self._lock:
                        if self._stop_requested:
                            self.state.update(status="Stopped")
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
                                self.state.update(status="Stopped")
                                return
                            paused = self._pause_requested
                    self.state.update(paused=False)

                def on_line_sent(line: str, sent_count: int) -> None:
                    self.state.update(status=f"Running: {line}", progress_done=sent_count)

                self.serial_service.stream_gcode_lines_unlocked(
                    ser,
                    stream_lines,
                    response_timeout=20,
                    should_stop=should_stop,
                    wait_while_paused=wait_while_paused,
                    on_line_sent=on_line_sent,
                )
                self.serial_service.wait_until_idle_unlocked(ser, timeout=120)
            if self.state.snapshot()["status"] != "Stopped":
                self.state.update(status="Finished")
        except Exception as exc:
            self.state.update(last_error=str(exc), status=f"Error: {exc}")
        finally:
            snapshot = self.state.snapshot()
            paused_duration_seconds = float(snapshot.get("paused_duration_seconds") or 0.0)
            pause_started_at = snapshot.get("pause_started_at")
            if pause_started_at:
                paused_duration_seconds += max(0.0, time.time() - float(pause_started_at))
            self.state.update(
                running=False,
                paused=False,
                pause_started_at=None,
                paused_duration_seconds=paused_duration_seconds,
            )
            with self._lock:
                self._stop_requested = False
                self._pause_requested = False
