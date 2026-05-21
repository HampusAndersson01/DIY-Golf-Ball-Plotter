from __future__ import annotations

import threading
import time

import pytest

from app.models.machine_state import MachineState
from app.services.job_runner import JobRunner
from app.services.machine_service import MachineService
from app.services.validation_service import ValidationService


BASE_CONFIG = {
    "DEFAULT_X_MAX_FEED": 6000.0,
    "DEFAULT_Y_MAX_FEED": 6000.0,
    "DEFAULT_X_ACCELERATION": 100.0,
    "DEFAULT_Y_ACCELERATION": 100.0,
    "MOTOR_FULL_STEPS_PER_REV": 200,
    "X_MICROSTEPS": 16,
    "Y_MICROSTEPS": 16,
    "DEFAULT_PEN_UP_S": 575,
    "DEFAULT_PEN_DOWN_S": 700,
    "DEFAULT_SERVO_RAMP_ENABLED": True,
    "DEFAULT_SERVO_RAMP_STEP": 20,
    "DEFAULT_SERVO_RAMP_DELAY_MS": 10.0,
    "DEFAULT_PEN_UP_DWELL_MS": 30.0,
    "DEFAULT_PEN_DOWN_DWELL_MS": 60.0,
    "DEFAULT_TRAVEL_FEED": 3000.0,
    "DEFAULT_DRAW_FEED": 1200.0,
    "Y_DRAW_MIN": -45.0,
    "Y_DRAW_MAX": 45.0,
    "MOTOR_HOLD_ENABLED_AFTER_CALIBRATION": True,
    "STEPPER_RELEASE_IDLE_DELAY_VALUE": 0,
    "STEPPER_HOLD_IDLE_DELAY_VALUE": 255,
    "STEPPER_HOLD_ENGAGE_DELTA_DEG": 0.25,
    "STEPPER_HOLD_ENGAGE_FEED": 120.0,
}


class FakeSerial:
    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.is_open = True

    def write(self, payload: bytes) -> int:
        self.writes.append(payload)
        return len(payload)


class FakeSerialService:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.serial = FakeSerial()
        self.command_calls: list[str] = []
        self.send_many_calls: list[list[str]] = []
        self.unlocked_calls: list[str] = []
        self.current_dollar1 = 0

    def connect(self):
        return self.serial

    def get_serial(self):
        return self.serial

    def has_live_serial(self) -> bool:
        return True

    def soft_reset(self, *, settle_seconds: float = 1.0):
        self.serial.write(b"\x18")
        return []

    def send_command(self, command: str, timeout: float = 15):
        self.command_calls.append(command)
        if command.startswith("$1="):
            self.current_dollar1 = int(command.split("=", 1)[1])
            return "ok"
        if command == "$$":
            return f"$1={self.current_dollar1}\nok"
        return "ok"

    def send_many(self, commands: list[str], delay: float = 0.04, wait_idle_between: bool = True):
        self.send_many_calls.append(list(commands))
        return "\n".join(f"{command} -> ok" for command in commands)

    def send_to_grbl_unlocked(self, ser, command: str, timeout: float = 15):
        self.unlocked_calls.append(command)
        return "ok"

    def stream_gcode_lines_unlocked(self, ser, lines: list[str], **kwargs):
        on_line_sent = kwargs.get("on_line_sent")
        should_stop = kwargs.get("should_stop")
        count = 0
        for line in lines:
            if should_stop and should_stop():
                return count
            count += 1
            if on_line_sent:
                on_line_sent(line, count)
        return count

    def wait_until_idle_unlocked(self, ser, timeout: float = 60):
        return True

    def read_available_lines(self, ser):
        return []


class FakeJobRunner:
    def __init__(self) -> None:
        self.stop_reasons: list[str] = []

    def request_stop(self, *, reason: str = "abort") -> None:
        self.stop_reasons.append(reason)


def build_machine_service(*, serial_service: FakeSerialService | None = None):
    state = MachineState(default_pen_up_s=BASE_CONFIG["DEFAULT_PEN_UP_S"])
    service = MachineService(
        config=BASE_CONFIG,
        state=state,
        serial_service=serial_service or FakeSerialService(),
        validation_service=ValidationService(),
        job_runner=FakeJobRunner(),
    )
    return service, state


def build_job_runner_with_machine_service():
    serial_service = FakeSerialService()
    state = MachineState(default_pen_up_s=BASE_CONFIG["DEFAULT_PEN_UP_S"])
    runner = JobRunner(state=state, serial_service=serial_service, config=BASE_CONFIG)
    service = MachineService(
        config=BASE_CONFIG,
        state=state,
        serial_service=serial_service,
        validation_service=ValidationService(),
        job_runner=runner,
    )
    runner.set_lifecycle_callback(service.handle_job_lifecycle_event)
    return runner, service, state, serial_service


def test_connect_releases_both_motors_before_calibration():
    serial_service = FakeSerialService()
    service, state = build_machine_service(serial_service=serial_service)

    service.connect()

    snapshot = state.snapshot()
    assert serial_service.command_calls[:2] == ["$1=0", "$$"]
    assert serial_service.serial.writes == [b"\x18"]
    assert snapshot["connected"] is True
    assert snapshot["motor_hold_enabled"] is False
    assert snapshot["motors"]["policy"] == "release_before_calibration"
    assert snapshot["motors"]["desired_dollar_1"] == 0
    assert snapshot["motors"]["applied_dollar_1"] == 0
    assert snapshot["motors"]["x_expected_holding"] is False
    assert snapshot["motors"]["y_expected_holding"] is False


def test_calibration_enables_motor_hold_and_keeps_locked_state():
    serial_service = FakeSerialService()
    service, state = build_machine_service(serial_service=serial_service)
    state.update(connected=True)

    response = service.zero_and_mark_calibrated()

    assert serial_service.send_many_calls[0] == ["$X", "G21", "G92 X0 Y0", "G90"]
    assert serial_service.send_many_calls[1] == ["$X", "G21", "G91", "G1 X0.2500 Y0.2500 F120.000", "G1 X-0.2500 Y-0.2500 F120.000", "G90"]
    assert serial_service.command_calls[-2:] == ["$1=255", "$$"]
    assert state.snapshot()["calibrated"] is True
    assert state.snapshot()["motor_hold_enabled"] is True
    assert state.snapshot()["motors"]["policy"] == "hold_after_calibration"
    assert state.snapshot()["motors"]["applied_dollar_1"] == 255
    assert state.snapshot()["motors"]["x_expected_holding"] is True
    assert state.snapshot()["motors"]["y_expected_holding"] is True
    assert "Stepper hold policy applied: $1=255" in response


def test_clear_calibration_releases_motor_hold_for_easy_manual_movement():
    serial_service = FakeSerialService()
    service, state = build_machine_service(serial_service=serial_service)
    state.update(connected=True, calibrated=True, motor_hold_enabled=True, machine_position_trusted=True)

    service.clear_calibrated()

    assert serial_service.command_calls[-2:] == ["$1=0", "$$"]
    assert serial_service.serial.writes == [b"\x18"]
    snapshot = state.snapshot()
    assert snapshot["calibrated"] is False
    assert snapshot["machine_position_trusted"] is False
    assert snapshot["motor_hold_enabled"] is False
    assert snapshot["motors"]["policy"] == "release_before_calibration"
    assert snapshot["motors"]["applied_dollar_1"] == 0


def test_manual_policy_test_forces_release_write_when_cached_state_is_stale():
    serial_service = FakeSerialService()
    service, state = build_machine_service(serial_service=serial_service)
    state.update(
        connected=True,
        calibrated=False,
        motors={
            **state.snapshot()["motors"],
            "applied_dollar_1": 0,
            "last_known_dollar_1": 0,
        },
    )
    serial_service.current_dollar1 = 255

    result = service.apply_stepper_hold_policy_for_test()

    assert serial_service.command_calls[-2:] == ["$1=0", "$$"]
    assert serial_service.serial.writes == [b"\x18"]
    assert result["applied_dollar_1"] == 0
    assert state.snapshot()["motors"]["hold_active"] is False


def test_normal_job_completion_finalizes_pen_up_before_home_and_keeps_hold():
    runner, service, state, serial_service = build_job_runner_with_machine_service()
    state.update(connected=True, calibrated=True, machine_position_trusted=True, motor_hold_enabled=True)

    result = runner.finalize_job("complete", machine_position_trusted=True)
    service.handle_job_lifecycle_event("finalize_job", {"reason": "complete"})

    assert result["job_finalization"]["pen_up_ok"] is True
    assert result["job_finalization"]["home_ok"] is True
    assert serial_service.unlocked_calls[:5] == ["$X", "M3 S575", "G4 P0.030", "G21", "G90"]
    assert serial_service.unlocked_calls[5].startswith("G0 X0 Y0 F3000.000")
    assert serial_service.command_calls[-2:] == ["$1=255", "$$"]
    assert state.snapshot()["motors"]["x_expected_holding"] is True
    assert state.snapshot()["motors"]["y_expected_holding"] is True


def test_start_marks_running_before_worker_thread_begins_and_rejects_second_start(monkeypatch: pytest.MonkeyPatch):
    runner, _, state, _ = build_job_runner_with_machine_service()
    state.update(connected=True, calibrated=True, last_gcode=["G1 X1 Y1"])

    started_threads: list[threading.Thread] = []

    def fake_start(thread: threading.Thread) -> None:
        started_threads.append(thread)

    monkeypatch.setattr(threading.Thread, "start", fake_start)

    runner.start()

    snapshot = state.snapshot()
    assert snapshot["running"] is True
    assert snapshot["status"] == "Starting"
    assert started_threads

    with pytest.raises(ValueError, match="already running"):
        runner.start()


def test_abort_and_fail_finalization_follow_home_safety_rules():
    runner, service, state, serial_service = build_job_runner_with_machine_service()
    state.update(connected=True, calibrated=True, machine_position_trusted=True, motor_hold_enabled=True)

    abort_result = runner.finalize_job("abort", machine_position_trusted=True)
    service.handle_job_lifecycle_event("finalize_job", {"reason": "abort"})

    assert abort_result["job_finalization"]["pen_up_attempted"] is True
    assert abort_result["job_finalization"]["home_attempted"] is True
    assert state.snapshot()["motors"]["x_expected_holding"] is True
    assert state.snapshot()["motors"]["y_expected_holding"] is True

    serial_service.unlocked_calls.clear()
    serial_service.command_calls.clear()
    fail_result = runner.finalize_job("fail", machine_position_trusted=False)
    service.handle_job_lifecycle_event("finalize_job", {"reason": "fail"})

    assert fail_result["job_finalization"]["pen_up_attempted"] is True
    assert fail_result["job_finalization"]["home_attempted"] is False
    assert fail_result["job_finalization"]["reason"] == "fail"
    assert state.snapshot()["last_job_finalization"]["reason"] == "fail"
    assert state.snapshot()["motors"]["applied_dollar_1"] == 255
    assert state.snapshot()["motors"]["last_apply_ok"] is True


def test_y_loop_test_requires_connection_calibration_and_no_active_job():
    service, state = build_machine_service()

    with pytest.raises(ValueError, match="Connect the machine"):
        service.start_y_loop_test({}, BASE_CONFIG)

    state.update(connected=True)
    with pytest.raises(ValueError, match="Calibrate the machine"):
        service.start_y_loop_test({}, BASE_CONFIG)

    state.update(calibrated=True, running=True)
    with pytest.raises(ValueError, match="active print"):
        service.start_y_loop_test({}, BASE_CONFIG)


def test_y_loop_test_sends_pen_up_moves_around_center_and_returns_to_center():
    serial_service = FakeSerialService()
    service, state = build_machine_service(serial_service=serial_service)
    state.update(connected=True, calibrated=True, machine_position_trusted=True, current_position_y=0.0)

    service.start_y_loop_test({"distance": 10, "feedrate": 1200, "dwell_sec": 0.25}, BASE_CONFIG)

    deadline = time.time() + 1.0
    while time.time() < deadline:
        if state.snapshot()["y_loop_test"]["cycles_completed"] >= 1:
            break
        time.sleep(0.01)

    service.stop_y_loop_test()

    commands = [command for batch in serial_service.send_many_calls for command in batch]
    assert commands[:6] == ["$X", "G21", "G91", "G1 X0.2500 Y0.2500 F120.000", "G1 X-0.2500 Y-0.2500 F120.000", "G90"]
    assert ["$X", "M3 S575", "G4 P0.030", "G90"] == commands[6:10]
    assert "G1 Y10.000 F1200.000" in commands
    assert "G1 Y-10.000 F1200.000" in commands
    assert commands[-1] == "G1 Y0.000 F1200.000"
    assert state.snapshot()["y_loop_test"]["enabled"] is False
    assert state.snapshot()["current_position_y"] == 0.0
    assert state.snapshot()["motor_hold_enabled"] is True
    assert state.snapshot()["movement_test"]["active"] is False
    assert state.snapshot()["movement_test"]["x_motor_holding"] is True
    assert state.snapshot()["movement_test"]["y_motor_holding"] is True


def test_y_loop_test_rejects_out_of_range_travel():
    service, state = build_machine_service()
    state.update(connected=True, calibrated=True, current_position_y=40.0)

    with pytest.raises(ValueError, match="exceed safe travel"):
        service.start_y_loop_test({"distance": 10}, BASE_CONFIG)
