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
            raise ValueError("Machine is not calibrated. Jog/zero first, then click 'I Have Calibrated'.")
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
        self.state.update(paused=True, status="Feed hold requested")

    def resume(self) -> None:
        with self._lock:
            self._pause_requested = False
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"~")
        self.state.update(paused=False, status="Resume requested")

    def request_stop(self) -> None:
        with self._lock:
            self._stop_requested = True
            self._pause_requested = False

    def _worker(self, gcode: list[str]) -> None:
        stream_lines = [line for line in gcode if self.gcode_service.is_streamable_line(line)]
        try:
            self.state.update(
                running=True,
                paused=False,
                status="Running",
                progress_total=len(stream_lines),
                progress_done=0,
                last_error=None,
            )
            with self.serial_service.lock:
                ser = self.serial_service.get_serial()
                for line in stream_lines:
                    with self._lock:
                        if self._stop_requested:
                            self.state.update(status="Stopped")
                            break
                        paused = self._pause_requested
                    while paused:
                        self.state.update(paused=True, status="Paused")
                        time.sleep(0.1)
                        with self._lock:
                            if self._stop_requested:
                                self.state.update(status="Stopped")
                                return
                            paused = self._pause_requested
                    self.state.update(paused=False, status=f"Running: {line}")
                    self.serial_service.send_to_grbl_unlocked(ser, line, timeout=20)
                    progress = self.state.snapshot()["progress_done"] + 1
                    self.state.update(progress_done=progress)
                self.serial_service.wait_until_idle_unlocked(ser, timeout=120)
            if self.state.snapshot()["status"] != "Stopped":
                self.state.update(status="Finished")
        except Exception as exc:
            self.state.update(last_error=str(exc), status=f"Error: {exc}")
        finally:
            self.state.update(running=False, paused=False)
            with self._lock:
                self._stop_requested = False
                self._pause_requested = False
