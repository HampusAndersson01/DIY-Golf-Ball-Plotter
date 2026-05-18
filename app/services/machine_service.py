from __future__ import annotations

import re
import threading
import time
from typing import Any

from . import pipeline_core


class StepperHoldPolicyManager:
    def __init__(self, *, config, state, serial_service) -> None:
        self.config = config
        self.state = state
        self.serial_service = serial_service
        self._lock = threading.Lock()

    def desired_policy(self, connected: bool, calibration_locked: bool) -> str:
        if not connected or not calibration_locked:
            return "release_before_calibration"
        return "hold_after_calibration"

    def desired_dollar1_for_policy(self, policy: str) -> int:
        if policy == "hold_after_calibration":
            return int(self.config["STEPPER_HOLD_IDLE_DELAY_VALUE"])
        return int(self.config["STEPPER_RELEASE_IDLE_DELAY_VALUE"])

    def is_streaming_active(self) -> bool:
        snapshot = self.state.snapshot()
        return bool(snapshot.get("running") and not snapshot.get("paused"))

    def _build_motor_state(self, *, connected: bool, calibration_locked: bool, policy: str, desired_dollar1: int, **overrides: Any) -> dict[str, Any]:
        applied = overrides.get("applied_dollar_1")
        last_known = overrides.get("last_known_dollar_1")
        hold_active = policy == "hold_after_calibration" and applied == desired_dollar1 and desired_dollar1 == int(self.config["STEPPER_HOLD_IDLE_DELAY_VALUE"])
        state = {
            "method": "grbl_$1_step_idle_delay",
            "connected": connected,
            "calibration_locked": calibration_locked,
            "policy": policy,
            "hold_active": hold_active,
            "desired_dollar_1": desired_dollar1,
            "applied_dollar_1": applied,
            "last_known_dollar_1": last_known,
            "x_expected_holding": hold_active,
            "y_expected_holding": hold_active,
            "applying": bool(overrides.get("applying", False)),
            "queued_apply_reason": overrides.get("queued_apply_reason"),
            "last_apply_reason": overrides.get("last_apply_reason"),
            "last_apply_ok": overrides.get("last_apply_ok"),
            "last_error": overrides.get("last_error"),
        }
        return state

    def _parse_dollar1(self, response: str) -> int | None:
        match = re.search(r"^\$1=(\d+)", response, flags=re.MULTILINE)
        return int(match.group(1)) if match else None

    def _read_back_dollar1_if_supported(self) -> int | None:
        response = self.serial_service.send_command("$$", timeout=10)
        return self._parse_dollar1(response)

    def _engage_hold_if_needed(self, previous_applied: Any, desired_dollar1: int) -> None:
        hold_value = int(self.config["STEPPER_HOLD_IDLE_DELAY_VALUE"])
        if desired_dollar1 != hold_value or previous_applied == hold_value:
            return

        configured_delta = float(self.config.get("STEPPER_HOLD_ENGAGE_DELTA_DEG", 0.25))
        feed = float(self.config.get("STEPPER_HOLD_ENGAGE_FEED", 120.0))
        x_steps_per_degree = (float(self.config["MOTOR_FULL_STEPS_PER_REV"]) * float(self.config["X_MICROSTEPS"])) / 360.0
        y_steps_per_degree = (float(self.config["MOTOR_FULL_STEPS_PER_REV"]) * float(self.config["Y_MICROSTEPS"])) / 360.0
        min_steps_per_degree = min(x_steps_per_degree, y_steps_per_degree)
        minimum_effective_delta = (2.0 / min_steps_per_degree) if min_steps_per_degree > 0 else configured_delta
        delta = max(configured_delta, minimum_effective_delta)
        if delta <= 0 or feed <= 0:
            return

        # The move must be large enough to generate real step pulses at the configured
        # microstep resolution, otherwise GRBL may accept it without energizing the drivers.
        self.serial_service.send_many(
            [
                "$X",
                "G21",
                "G91",
                f"G1 X{delta:.4f} Y{delta:.4f} F{feed:.3f}",
                f"G1 X{-delta:.4f} Y{-delta:.4f} F{feed:.3f}",
                "G90",
            ],
            wait_idle_between=True,
        )

    def _reset_after_release_if_needed(self, *, force: bool, desired_dollar1: int) -> None:
        if not force:
            return
        release_value = int(self.config["STEPPER_RELEASE_IDLE_DELAY_VALUE"])
        if desired_dollar1 != release_value:
            return
        self.serial_service.soft_reset()

    def apply(self, reason: str, *, force: bool = False) -> dict[str, Any]:
        snapshot = self.state.snapshot()
        connected = bool(snapshot.get("connected"))
        calibration_locked = bool(snapshot.get("calibrated"))
        policy = self.desired_policy(connected, calibration_locked)
        desired_dollar1 = self.desired_dollar1_for_policy(policy)
        motors_snapshot = dict(snapshot.get("motors") or {})
        previous_applied = motors_snapshot.get("applied_dollar_1")

        if not connected:
            motors = self._build_motor_state(
                connected=False,
                calibration_locked=False,
                policy="release_before_calibration",
                desired_dollar1=desired_dollar1,
                applied_dollar_1=None,
                last_known_dollar_1=motors_snapshot.get("last_known_dollar_1"),
                applying=False,
                queued_apply_reason=None,
                last_apply_reason=reason,
                last_apply_ok=None,
                last_error=None,
            )
            debug = {
                "reason": reason,
                "connected": False,
                "calibration_locked": False,
                "desired_policy": "release_before_calibration",
                "desired_dollar_1": desired_dollar1,
                "previous_applied_dollar_1": previous_applied,
                "new_applied_dollar_1": None,
                "readback_dollar_1": None,
                "streaming_active": False,
                "queued": False,
                "ok": None,
            }
            self.state.update(motor_hold_enabled=False, motors=motors, stepper_hold_debug=debug)
            return motors

        if self.is_streaming_active():
            motors = self._build_motor_state(
                connected=connected,
                calibration_locked=calibration_locked,
                policy=policy,
                desired_dollar1=desired_dollar1,
                applied_dollar_1=motors_snapshot.get("applied_dollar_1"),
                last_known_dollar_1=motors_snapshot.get("last_known_dollar_1"),
                applying=False,
                queued_apply_reason=reason,
                last_apply_reason=motors_snapshot.get("last_apply_reason"),
                last_apply_ok=motors_snapshot.get("last_apply_ok"),
                last_error=motors_snapshot.get("last_error"),
            )
            debug = {
                "reason": reason,
                "connected": connected,
                "calibration_locked": calibration_locked,
                "desired_policy": policy,
                "desired_dollar_1": desired_dollar1,
                "previous_applied_dollar_1": previous_applied,
                "new_applied_dollar_1": motors_snapshot.get("applied_dollar_1"),
                "readback_dollar_1": motors_snapshot.get("last_known_dollar_1"),
                "streaming_active": True,
                "queued": True,
                "ok": motors_snapshot.get("last_apply_ok"),
            }
            self.state.update(motors=motors, stepper_hold_debug=debug)
            return motors

        if previous_applied == desired_dollar1 and not force:
            readback_dollar1 = self._read_back_dollar1_if_supported()
            if readback_dollar1 is not None and readback_dollar1 != desired_dollar1:
                previous_applied = readback_dollar1
            else:
                motors = self._build_motor_state(
                    connected=connected,
                    calibration_locked=calibration_locked,
                    policy=policy,
                    desired_dollar1=desired_dollar1,
                    applied_dollar_1=desired_dollar1,
                    last_known_dollar_1=readback_dollar1 if readback_dollar1 is not None else motors_snapshot.get("last_known_dollar_1", desired_dollar1),
                    applying=False,
                    queued_apply_reason=None,
                    last_apply_reason=reason,
                    last_apply_ok=True,
                    last_error=None,
                )
                debug = {
                    "reason": reason,
                    "connected": connected,
                    "calibration_locked": calibration_locked,
                    "desired_policy": policy,
                    "desired_dollar_1": desired_dollar1,
                    "previous_applied_dollar_1": previous_applied,
                    "new_applied_dollar_1": desired_dollar1,
                    "readback_dollar_1": motors.get("last_known_dollar_1"),
                    "streaming_active": False,
                    "queued": False,
                    "ok": True,
                }
                self.state.update(
                    motor_hold_enabled=motors["hold_active"],
                    motors=motors,
                    stepper_hold_debug=debug,
                )
                return motors

        with self._lock:
            applying = self._build_motor_state(
                connected=connected,
                calibration_locked=calibration_locked,
                policy=policy,
                desired_dollar1=desired_dollar1,
                applied_dollar_1=previous_applied,
                last_known_dollar_1=motors_snapshot.get("last_known_dollar_1"),
                applying=True,
                queued_apply_reason=None,
                last_apply_reason=motors_snapshot.get("last_apply_reason"),
                last_apply_ok=motors_snapshot.get("last_apply_ok"),
                last_error=None,
            )
            self.state.update(motors=applying)
            config_command = f"$1={desired_dollar1}"
            readback_dollar1 = None
            try:
                response = self.serial_service.send_command(config_command, timeout=10)
                if "ok" not in response.lower():
                    raise RuntimeError(f"Stepper hold policy failed to apply: {response}")
                self._reset_after_release_if_needed(force=force, desired_dollar1=desired_dollar1)
                self._engage_hold_if_needed(previous_applied, desired_dollar1)
                readback_dollar1 = self._read_back_dollar1_if_supported()
                if readback_dollar1 is not None and readback_dollar1 != desired_dollar1:
                    raise RuntimeError(f"Stepper hold policy failed. Expected $1={desired_dollar1}, got $1={readback_dollar1}")
                motors = self._build_motor_state(
                    connected=connected,
                    calibration_locked=calibration_locked,
                    policy=policy,
                    desired_dollar1=desired_dollar1,
                    applied_dollar_1=desired_dollar1,
                    last_known_dollar_1=readback_dollar1 if readback_dollar1 is not None else desired_dollar1,
                    applying=False,
                    queued_apply_reason=None,
                    last_apply_reason=reason,
                    last_apply_ok=True,
                    last_error=None,
                )
                debug = {
                    "reason": reason,
                    "connected": connected,
                    "calibration_locked": calibration_locked,
                    "desired_policy": policy,
                    "desired_dollar_1": desired_dollar1,
                    "previous_applied_dollar_1": previous_applied,
                    "new_applied_dollar_1": desired_dollar1,
                    "readback_dollar_1": readback_dollar1 if readback_dollar1 is not None else desired_dollar1,
                    "streaming_active": False,
                    "queued": False,
                    "ok": True,
                }
                self.state.update(
                    motor_hold_enabled=motors["hold_active"],
                    motors=motors,
                    stepper_hold_debug=debug,
                )
                return motors
            except Exception as exc:
                motors = self._build_motor_state(
                    connected=connected,
                    calibration_locked=calibration_locked,
                    policy=policy,
                    desired_dollar1=desired_dollar1,
                    applied_dollar_1=previous_applied,
                    last_known_dollar_1=readback_dollar1 if readback_dollar1 is not None else motors_snapshot.get("last_known_dollar_1"),
                    applying=False,
                    queued_apply_reason=None,
                    last_apply_reason=reason,
                    last_apply_ok=False,
                    last_error=str(exc),
                )
                debug = {
                    "reason": reason,
                    "connected": connected,
                    "calibration_locked": calibration_locked,
                    "desired_policy": policy,
                    "desired_dollar_1": desired_dollar1,
                    "previous_applied_dollar_1": previous_applied,
                    "new_applied_dollar_1": previous_applied,
                    "readback_dollar_1": readback_dollar1,
                    "streaming_active": False,
                    "queued": False,
                    "ok": False,
                }
                self.state.update(
                    motor_hold_enabled=bool(previous_applied == self.config["STEPPER_HOLD_IDLE_DELAY_VALUE"]),
                    motors=motors,
                    stepper_hold_debug=debug,
                )
                raise


class MachineService:
    def __init__(self, *, config, state, serial_service, validation_service, job_runner) -> None:
        self.config = config
        self.state = state
        self.serial_service = serial_service
        self.validation = validation_service
        self.job_runner = job_runner
        self.stepper_policy = StepperHoldPolicyManager(config=config, state=state, serial_service=serial_service)
        self._y_loop_lock = threading.Lock()
        self._y_loop_thread: threading.Thread | None = None
        self._y_loop_stop_requested = False

    def connect(self) -> None:
        self.serial_service.connect()
        self.state.update(connected=True, emergency_stopped=False)
        self.apply_stepper_hold_policy("connect", force=True)

    def prepare_for_job_start(self) -> None:
        self.stop_y_loop_test(force=True)
        self.apply_stepper_hold_policy("before_run_gcode")

    def apply_stepper_hold_policy(self, reason: str, *, force: bool = False) -> dict[str, Any]:
        return self.stepper_policy.apply(reason, force=force)

    def handle_job_lifecycle_event(self, event: str, payload: dict[str, Any]) -> None:
        if event == "pause_job":
            self.apply_stepper_hold_policy("after_pause")
            return
        if event == "resume_job":
            self.apply_stepper_hold_policy("before_resume")
            return
        if event == "finalize_job":
            final_reason = payload.get("reason", "finalize_job")
            self.apply_stepper_hold_policy(f"after_{final_reason}")
            return

    def send_command(self, data):
        cmd = str(data.get("command", "")).strip()
        response = self.serial_service.send_command(cmd)
        if cmd.startswith("$1="):
            self.state.update(status="Warning: direct $1 write outside StepperHoldPolicyManager")
        return cmd, response

    def reset(self) -> str:
        self.stop_y_loop_test(force=True)
        self.job_runner.request_stop(reason="emergency_stop")
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"\x18")
            time.sleep(1)
            lines = self.serial_service.read_available_lines(ser)
        self.state.update(
            connected=True,
            calibrated=False,
            machine_position_trusted=False,
            emergency_stopped=True,
            status="Soft reset sent - calibration cleared",
        )
        self.apply_stepper_hold_policy("after_emergency_stop")
        return "\n".join(lines) if lines else "RESET SENT"

    def apply_config(self, data, config) -> str:
        x_max_feed = self.validation.validate_feed(data.get("x_max_feed", config["DEFAULT_X_MAX_FEED"]))
        y_max_feed = self.validation.validate_feed(data.get("y_max_feed", config["DEFAULT_Y_MAX_FEED"]))
        x_acceleration = float(data.get("x_acceleration", config["DEFAULT_X_ACCELERATION"]))
        y_acceleration = float(data.get("y_acceleration", config["DEFAULT_Y_ACCELERATION"]))
        if x_acceleration <= 0 or y_acceleration <= 0:
            raise ValueError("Acceleration must be greater than 0")
        if x_acceleration > 10000 or y_acceleration > 10000:
            raise ValueError("Acceleration is too high")
        x_steps = (config["MOTOR_FULL_STEPS_PER_REV"] * config["X_MICROSTEPS"]) / 360.0
        y_steps = (config["MOTOR_FULL_STEPS_PER_REV"] * config["Y_MICROSTEPS"]) / 360.0
        commands = [
            "$X",
            "$30=1000",
            "$31=0",
            "$32=0",
            "$22=0",
            "$20=0",
            "$21=0",
            f"$100={x_steps:.6f}",
            f"$110={x_max_feed:.3f}",
            f"$120={x_acceleration:.3f}",
            "$130=100000",
            f"$101={y_steps:.6f}",
            f"$111={y_max_feed:.3f}",
            f"$121={y_acceleration:.3f}",
            "$131=90",
            "$102=80.000",
            "$112=500.000",
            "$122=50.000",
            "$132=10",
            "G21",
            "G90",
        ]
        response = self.serial_service.send_many(commands, wait_idle_between=False)
        self.state.update(status="GRBL settings applied")
        return response

    def _build_pen_payload(self, data, config, *, up: bool):
        default_s = config["DEFAULT_PEN_UP_S"] if up else config["DEFAULT_PEN_DOWN_S"]
        default_start = self.state.get_servo(config["DEFAULT_PEN_DOWN_S"] if up else config["DEFAULT_PEN_UP_S"])
        s_value = self.validation.validate_servo_s(data.get("s", default_s))
        start_s = self.validation.validate_servo_s(data.get("start_s", default_start))
        ramp_enabled = self.validation.validate_bool(data.get("servo_ramp_enabled", config["DEFAULT_SERVO_RAMP_ENABLED"]))
        ramp_step = self.validation.validate_non_negative_int(
            data.get("servo_ramp_step", config["DEFAULT_SERVO_RAMP_STEP"]),
            "Servo ramp step",
            minimum=1,
            maximum=200,
        )
        ramp_delay_ms = self.validation.validate_non_negative_float(
            data.get("servo_ramp_delay_ms", config["DEFAULT_SERVO_RAMP_DELAY_MS"]),
            "Servo ramp delay",
            maximum=1000,
        )
        dwell_label = "Pen up dwell" if up else "Pen down dwell"
        dwell_default = config["DEFAULT_PEN_UP_DWELL_MS"] if up else config["DEFAULT_PEN_DOWN_DWELL_MS"]
        dwell_key = "pen_up_dwell_ms" if up else "pen_down_dwell_ms"
        dwell_ms = self.validation.validate_non_negative_float(data.get(dwell_key, dwell_default), dwell_label, maximum=5000)
        commands = [
            "$X",
            *pipeline_core.build_pen_position_commands(
                start_s,
                s_value,
                ramp_enabled=ramp_enabled,
                ramp_step=ramp_step,
                ramp_delay_ms=ramp_delay_ms,
                dwell_ms=dwell_ms,
            ),
        ]
        return s_value, commands

    def pen_up(self, data, config):
        s_value, commands = self._build_pen_payload(data, config, up=True)
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.set_servo(s_value)
        return f"PEN UP M3 S{s_value}", response

    def pen_down(self, data, config):
        s_value, commands = self._build_pen_payload(data, config, up=False)
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.set_servo(s_value)
        return f"PEN DOWN M3 S{s_value}", response

    def pen_test(self, data, config):
        up_s = self.validation.validate_servo_s(data.get("up_s", config["DEFAULT_PEN_UP_S"]))
        down_s = self.validation.validate_servo_s(data.get("down_s", config["DEFAULT_PEN_DOWN_S"]))
        start_s = self.validation.validate_servo_s(data.get("start_s", self.state.get_servo(up_s)))
        ramp_enabled = self.validation.validate_bool(data.get("servo_ramp_enabled", config["DEFAULT_SERVO_RAMP_ENABLED"]))
        ramp_step = self.validation.validate_non_negative_int(
            data.get("servo_ramp_step", config["DEFAULT_SERVO_RAMP_STEP"]),
            "Servo ramp step",
            minimum=1,
            maximum=200,
        )
        ramp_delay_ms = self.validation.validate_non_negative_float(
            data.get("servo_ramp_delay_ms", config["DEFAULT_SERVO_RAMP_DELAY_MS"]),
            "Servo ramp delay",
            maximum=1000,
        )
        up_dwell_ms = self.validation.validate_non_negative_float(
            data.get("pen_up_dwell_ms", config["DEFAULT_PEN_UP_DWELL_MS"]),
            "Pen up dwell",
            maximum=5000,
        )
        down_dwell_ms = self.validation.validate_non_negative_float(
            data.get("pen_down_dwell_ms", config["DEFAULT_PEN_DOWN_DWELL_MS"]),
            "Pen down dwell",
            maximum=5000,
        )
        commands = ["$X"]
        commands.extend(
            pipeline_core.build_pen_position_commands(
                start_s,
                down_s,
                ramp_enabled=ramp_enabled,
                ramp_step=ramp_step,
                ramp_delay_ms=ramp_delay_ms,
                dwell_ms=down_dwell_ms,
            )
        )
        commands.extend(
            pipeline_core.build_pen_position_commands(
                down_s,
                up_s,
                ramp_enabled=ramp_enabled,
                ramp_step=ramp_step,
                ramp_delay_ms=ramp_delay_ms,
                dwell_ms=up_dwell_ms,
            )
        )
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.set_servo(up_s)
        return f"PEN TEST S{up_s}/S{down_s}", response

    def servo_off(self, config) -> str:
        response = self.serial_service.send_many(["$X", "M5"], wait_idle_between=True)
        self.state.set_servo(config["DEFAULT_PEN_UP_S"])
        return response

    def jog(self, data, config):
        if self.state.snapshot().get("y_loop_test", {}).get("enabled"):
            raise ValueError("Stop the Y Axis Current Test Loop before jogging manually.")
        axis = str(data.get("axis", "X")).upper()
        if axis not in {"X", "Y"}:
            raise ValueError("Axis must be X or Y")
        degrees = self.validation.validate_degrees(data.get("degrees", 0))
        feed = self.validation.validate_feed(data.get("feed", config["DEFAULT_TRAVEL_FEED"]))
        commands = ["$X", "G21", "G91", f"G1 {axis}{degrees:.6f} F{feed:.3f}", "G4 P0.01", "G90"]
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        snapshot = self.state.snapshot()
        next_x = float(snapshot.get("current_position_x", 0.0))
        next_y = float(snapshot.get("current_position_y", 0.0))
        if axis == "X":
            next_x += degrees
        else:
            next_y += degrees
        self.state.update(
            current_position_x=next_x,
            current_position_y=next_y,
            machine_position_trusted=bool(snapshot.get("machine_position_trusted") or snapshot.get("calibrated")),
            status=f"Jogged {axis}{degrees:.3f}",
        )
        return f"JOG {axis}{degrees:.3f}", response

    def zero_position(self) -> str:
        response = self.serial_service.send_many(["$X", "G21", "G92 X0 Y0", "G90"], wait_idle_between=True)
        self.state.update(
            calibrated=False,
            machine_position_trusted=True,
            current_position_x=0.0,
            current_position_y=0.0,
            status="Zero set - click calibrated when physically ready",
        )
        return response

    def zero_and_mark_calibrated(self) -> str:
        response = self.serial_service.send_many(["$X", "G21", "G92 X0 Y0", "G90"], wait_idle_between=True)
        self.state.update(
            calibrated=True,
            machine_position_trusted=True,
            emergency_stopped=False,
            current_position_x=0.0,
            current_position_y=0.0,
            status="Origin set and calibrated",
        )
        motors = self.apply_stepper_hold_policy("set_origin_and_calibrate")
        return f"{response}\nStepper hold policy applied: $1={motors['applied_dollar_1']}"

    def go_home(self, data, config) -> str:
        pen_up_s = self.validation.validate_servo_s(data.get("pen_up_s", config["DEFAULT_PEN_UP_S"]))
        travel_feed = self.validation.validate_feed(data.get("travel_feed", config["DEFAULT_TRAVEL_FEED"]))
        start_s = self.validation.validate_servo_s(data.get("start_s", self.state.get_servo(config["DEFAULT_PEN_DOWN_S"])))
        ramp_enabled = self.validation.validate_bool(data.get("servo_ramp_enabled", config["DEFAULT_SERVO_RAMP_ENABLED"]))
        ramp_step = self.validation.validate_non_negative_int(
            data.get("servo_ramp_step", config["DEFAULT_SERVO_RAMP_STEP"]),
            "Servo ramp step",
            minimum=1,
            maximum=200,
        )
        ramp_delay_ms = self.validation.validate_non_negative_float(
            data.get("servo_ramp_delay_ms", config["DEFAULT_SERVO_RAMP_DELAY_MS"]),
            "Servo ramp delay",
            maximum=1000,
        )
        pen_up_dwell_ms = self.validation.validate_non_negative_float(
            data.get("pen_up_dwell_ms", config["DEFAULT_PEN_UP_DWELL_MS"]),
            "Pen up dwell",
            maximum=5000,
        )
        commands = ["$X"]
        commands.extend(
            pipeline_core.build_pen_position_commands(
                start_s,
                pen_up_s,
                ramp_enabled=ramp_enabled,
                ramp_step=ramp_step,
                ramp_delay_ms=ramp_delay_ms,
                dwell_ms=pen_up_dwell_ms,
            )
        )
        commands.extend(["G21", "G90", f"G0 X0.0000 Y0.0000 F{travel_feed:.3f}"])
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.set_servo(pen_up_s)
        self.state.update(
            current_position_x=0.0,
            current_position_y=0.0,
            machine_position_trusted=True,
            status="Returned to X0 Y0 with pen up",
        )
        return response

    def mark_calibrated(self) -> None:
        self.state.update(calibrated=True, machine_position_trusted=True, emergency_stopped=False, status="Calibrated and ready")
        self.apply_stepper_hold_policy("mark_calibrated")

    def clear_calibrated(self) -> None:
        self.stop_y_loop_test(force=True)
        self.state.update(calibrated=False, machine_position_trusted=False, status="Calibration cleared")
        self.apply_stepper_hold_policy("clear_calibration", force=True)

    def stop_active_job(self) -> str:
        self.job_runner.request_stop(reason="user_stop")
        self.state.update(status="Stop requested")
        return "Stop requested"

    def apply_stepper_hold_policy_for_test(self) -> dict[str, Any]:
        return self.apply_stepper_hold_policy("manual_policy_test", force=True)

    def start_y_loop_test(self, data, config) -> str:
        snapshot = self.state.snapshot()
        if not snapshot.get("connected"):
            raise ValueError("Connect the machine before starting the Y Axis Current Test Loop.")
        if not snapshot.get("calibrated"):
            raise ValueError("Calibrate the machine before starting the Y Axis Current Test Loop.")
        if snapshot.get("running"):
            raise ValueError("Stop the active print before starting the Y Axis Current Test Loop.")
        if snapshot.get("y_loop_test", {}).get("enabled"):
            raise ValueError("Y Axis Current Test Loop is already running.")

        distance = self.validation.validate_non_negative_float(data.get("distance", 10.0), "Y loop distance", maximum=90)
        if distance <= 0:
            raise ValueError("Y loop distance must be greater than 0.")
        feedrate = self.validation.validate_feed(data.get("feedrate", config["DEFAULT_DRAW_FEED"]))
        dwell_sec = self.validation.validate_non_negative_float(data.get("dwell_sec", 0.1), "Y loop dwell", maximum=5)
        pen_up_s = self.validation.validate_servo_s(data.get("pen_up_s", config["DEFAULT_PEN_UP_S"]))
        pen_up_dwell_ms = self.validation.validate_non_negative_float(
            data.get("pen_up_dwell_ms", config["DEFAULT_PEN_UP_DWELL_MS"]),
            "Pen up dwell",
            maximum=5000,
        )
        center_y = float(snapshot.get("current_position_y", 0.0))
        y_min = float(config.get("Y_DRAW_MIN", -45.0))
        y_max = float(config.get("Y_DRAW_MAX", 45.0))
        if center_y + distance > y_max or center_y - distance < y_min:
            raise ValueError(
                f"Y loop range would exceed safe travel. Current Y is {center_y:.3f}; with distance {distance:.3f} it must stay within {y_min:.3f}..{y_max:.3f}."
            )

        self.apply_stepper_hold_policy("start_y_axis_current_test")
        self.state.update(
            y_loop_test={
                "enabled": True,
                "center_y": center_y,
                "distance": distance,
                "feedrate": feedrate,
                "dwell_sec": dwell_sec,
                "phase": "starting",
                "cycles_completed": 0,
            },
            movement_test={
                "active": True,
                "axis": "Y",
                "x_motor_holding": True,
                "y_motor_holding": True,
                "amplitude_deg": distance,
                "feedrate": feedrate,
                "cycle_count": 0,
            },
            status="Y Axis Current Test Loop starting",
            last_error=None,
        )
        with self._y_loop_lock:
            self._y_loop_stop_requested = False
            self._y_loop_thread = threading.Thread(
                target=self._run_y_loop_test,
                args=(center_y, distance, feedrate, dwell_sec, pen_up_s, pen_up_dwell_ms),
                daemon=True,
            )
            self._y_loop_thread.start()
        return "Y Axis Current Test Loop started"

    def stop_y_loop_test(self, *, force: bool = False) -> str:
        with self._y_loop_lock:
            thread = self._y_loop_thread
            if thread is None:
                self.state.update(
                    y_loop_test={
                        **self.state.snapshot().get("y_loop_test", {}),
                        "enabled": False,
                        "phase": "idle",
                    }
                )
                return "Y Axis Current Test Loop already stopped"
            self._y_loop_stop_requested = True
        thread.join(timeout=10 if force else 5)
        if thread.is_alive():
            raise RuntimeError("Y Axis Current Test Loop did not stop cleanly.")
        with self._y_loop_lock:
            self._y_loop_thread = None
        return "Y Axis Current Test Loop stopped"

    def _run_y_loop_test(
        self,
        center_y: float,
        distance: float,
        feedrate: float,
        dwell_sec: float,
        pen_up_s: int,
        pen_up_dwell_ms: float,
    ) -> None:
        positive_y = center_y + distance
        negative_y = center_y - distance
        cycles_completed = 0
        try:
            pen_up_commands = ["$X", f"M3 S{pen_up_s}", f"G4 P{max(0.0, pen_up_dwell_ms) / 1000.0:.3f}", "G90"]
            self.serial_service.send_many(pen_up_commands, wait_idle_between=True)
            self.state.set_servo(pen_up_s)
            self.state.update(status=f"Y Axis Current Test Loop running around Y{center_y:.3f}")
            while True:
                if self._should_stop_y_loop():
                    break
                self._send_y_loop_phase("to_positive", positive_y, feedrate, dwell_sec)
                if self._should_stop_y_loop():
                    break
                self._send_y_loop_phase("to_negative", negative_y, feedrate, dwell_sec)
                if self._should_stop_y_loop():
                    break
                self._send_y_loop_phase("to_center", center_y, feedrate, 0.0)
                cycles_completed += 1
                self.state.update(
                    current_position_y=center_y,
                    y_loop_test={**self.state.snapshot().get("y_loop_test", {}), "enabled": True, "phase": "idle", "cycles_completed": cycles_completed},
                    movement_test={
                        "active": True,
                        "axis": "Y",
                        "x_motor_holding": True,
                        "y_motor_holding": True,
                        "amplitude_deg": distance,
                        "feedrate": feedrate,
                        "cycle_count": cycles_completed,
                    },
                )
        except Exception as exc:
            self.state.update(last_error=str(exc), status=f"Y Axis Current Test Loop error: {exc}")
            raise
        finally:
            try:
                self.serial_service.send_many(
                    ["$X", f"M3 S{pen_up_s}", f"G4 P{max(0.0, pen_up_dwell_ms) / 1000.0:.3f}", "G90", f"G1 Y{center_y:.3f} F{feedrate:.3f}"],
                    wait_idle_between=True,
                )
                self.state.set_servo(pen_up_s)
                self.state.update(current_position_y=center_y, machine_position_trusted=True)
                self.apply_stepper_hold_policy("stop_y_axis_current_test")
            finally:
                motors = self.state.snapshot().get("motors", {})
                self.state.update(
                    y_loop_test={**self.state.snapshot().get("y_loop_test", {}), "enabled": False, "phase": "idle", "cycles_completed": cycles_completed},
                    movement_test={
                        "active": False,
                        "axis": "Y",
                        "x_motor_holding": bool(motors.get("x_expected_holding")),
                        "y_motor_holding": bool(motors.get("y_expected_holding")),
                        "amplitude_deg": distance,
                        "feedrate": feedrate,
                        "cycle_count": cycles_completed,
                    },
                    status="Y Axis Current Test Loop stopped",
                )
                with self._y_loop_lock:
                    self._y_loop_thread = None
                    self._y_loop_stop_requested = False

    def _send_y_loop_phase(self, phase: str, target_y: float, feedrate: float, dwell_sec: float) -> None:
        commands = ["$X", "G90", f"G1 Y{target_y:.3f} F{feedrate:.3f}"]
        if dwell_sec > 0:
            commands.append(f"G4 P{dwell_sec:.3f}")
        self.state.update(y_loop_test={**self.state.snapshot().get("y_loop_test", {}), "enabled": True, "phase": phase})
        self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.update(current_position_y=target_y)

    def _should_stop_y_loop(self) -> bool:
        with self._y_loop_lock:
            return self._y_loop_stop_requested
