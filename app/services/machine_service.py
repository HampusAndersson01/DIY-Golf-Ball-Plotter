from __future__ import annotations

import time

from ._legacy import legacy


class MachineService:
    def __init__(self, *, config, state, serial_service, validation_service, job_runner) -> None:
        self.config = config
        self.state = state
        self.serial_service = serial_service
        self.validation = validation_service
        self.job_runner = job_runner

    def connect(self) -> None:
        self.serial_service.connect()

    def send_command(self, data):
        cmd = str(data.get("command", "")).strip()
        response = self.serial_service.send_command(cmd)
        return cmd, response

    def reset(self) -> str:
        self.job_runner.request_stop()
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"\x18")
            time.sleep(1)
            lines = self.serial_service.read_available_lines(ser)
        self.state.update(calibrated=False, status="Soft reset sent - calibration cleared")
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
            *legacy.build_pen_position_commands(
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
            legacy.build_pen_position_commands(
                start_s,
                down_s,
                ramp_enabled=ramp_enabled,
                ramp_step=ramp_step,
                ramp_delay_ms=ramp_delay_ms,
                dwell_ms=down_dwell_ms,
            )
        )
        commands.extend(
            legacy.build_pen_position_commands(
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
        axis = str(data.get("axis", "X")).upper()
        if axis not in {"X", "Y"}:
            raise ValueError("Axis must be X or Y")
        degrees = self.validation.validate_degrees(data.get("degrees", 0))
        feed = self.validation.validate_feed(data.get("feed", config["DEFAULT_TRAVEL_FEED"]))
        commands = ["$X", "G21", "G91", f"G1 {axis}{degrees:.6f} F{feed:.3f}", "G4 P0.01", "G90"]
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.update(calibrated=False, status="Jogged - calibration cleared until you confirm again")
        return f"JOG {axis}{degrees:.3f}", response

    def zero_position(self) -> str:
        response = self.serial_service.send_many(["$X", "G21", "G92 X0 Y0", "G90"], wait_idle_between=True)
        self.state.update(calibrated=False, status="Zero set - click calibrated when physically ready")
        return response

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
            legacy.build_pen_position_commands(
                start_s,
                pen_up_s,
                ramp_enabled=ramp_enabled,
                ramp_step=ramp_step,
                ramp_delay_ms=ramp_delay_ms,
                dwell_ms=pen_up_dwell_ms,
            )
        )
        commands.extend(["G21", "G90", f"G1 X0.0000 Y0.0000 F{travel_feed:.3f}"])
        response = self.serial_service.send_many(commands, wait_idle_between=True)
        self.state.set_servo(pen_up_s)
        self.state.update(status="Returned to X0 Y0 with pen up")
        return response

    def mark_calibrated(self) -> None:
        self.state.update(calibrated=True, status="Calibrated and ready")

    def clear_calibrated(self) -> None:
        self.state.update(calibrated=False, status="Calibration cleared")

    def stop_active_job(self) -> str:
        self.job_runner.request_stop()
        with self.serial_service.lock:
            ser = self.serial_service.get_serial()
            ser.write(b"!")
            time.sleep(0.1)
            ser.write(b"\x18")
            time.sleep(1)
            lines = self.serial_service.read_available_lines(ser)
        self.state.update(calibrated=False, status="Stopped - calibration cleared")
        return "\n".join(lines) if lines else "Stopped"
