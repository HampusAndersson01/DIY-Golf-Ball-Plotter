from flask import Blueprint, current_app, request

from app.extensions import get_machine_service
from app.utils.response_utils import json_error, json_ok, log_exception

machine_bp = Blueprint("machine", __name__)


@machine_bp.post("/connect")
def connect_route():
    try:
        get_machine_service().connect()
        return json_ok(command="CONNECT", response="Connected")
    except Exception as exc:
        log_exception("Machine connect failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/command")
def command():
    data = request.get_json(force=True)
    try:
        cmd, response = get_machine_service().send_command(data)
        return json_ok(command=cmd, response=response)
    except Exception as exc:
        log_exception("Machine command failed", exc, command=data.get("command"))
        return json_error(str(exc), status=500)


@machine_bp.post("/reset")
def reset():
    try:
        response = get_machine_service().reset()
        return json_ok(command="CTRL-X RESET", response=response)
    except Exception as exc:
        log_exception("Machine reset failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/apply-config")
def apply_config():
    try:
        response = get_machine_service().apply_config(request.get_json(force=True), current_app.config)
        return json_ok(command="APPLY GRBL SETTINGS", response=response)
    except Exception as exc:
        log_exception("Apply config failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/pen-up")
def pen_up():
    try:
        command_name, response = get_machine_service().pen_up(request.get_json(force=True), current_app.config)
        return json_ok(command=command_name, response=response)
    except Exception as exc:
        log_exception("Pen up failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/pen-down")
def pen_down():
    try:
        command_name, response = get_machine_service().pen_down(request.get_json(force=True), current_app.config)
        return json_ok(command=command_name, response=response)
    except Exception as exc:
        log_exception("Pen down failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/pen-test")
def pen_test():
    try:
        command_name, response = get_machine_service().pen_test(request.get_json(force=True), current_app.config)
        return json_ok(command=command_name, response=response)
    except Exception as exc:
        log_exception("Pen test failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/servo-off")
def servo_off():
    try:
        response = get_machine_service().servo_off(current_app.config)
        return json_ok(command="SERVO OFF M5", response=response)
    except Exception as exc:
        log_exception("Servo off failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/jog")
def jog():
    try:
        command_name, response = get_machine_service().jog(request.get_json(force=True), current_app.config)
        return json_ok(command=command_name, response=response)
    except Exception as exc:
        log_exception("Jog failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/zero-position")
def zero_position():
    try:
        response = get_machine_service().zero_position()
        return json_ok(command="G92 X0 Y0", response=response)
    except Exception as exc:
        log_exception("Zero position failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/zero-and-mark-calibrated")
def zero_and_mark_calibrated():
    try:
        response = get_machine_service().zero_and_mark_calibrated()
        return json_ok(
            command="ZERO AND MARK CALIBRATED",
            response=response,
            calibrated=True,
        )
    except Exception as exc:
        log_exception("Zero and calibrate failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/go-home")
def go_home():
    try:
        response = get_machine_service().go_home(request.get_json(force=True), current_app.config)
        return json_ok(command="GO HOME X0 Y0 WITH PEN UP", response=response)
    except Exception as exc:
        log_exception("Go home failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/mark-calibrated")
def mark_calibrated():
    get_machine_service().mark_calibrated()
    return json_ok(command="MARK CALIBRATED", response="Runner unlocked")


@machine_bp.post("/clear-calibrated")
def clear_calibrated():
    get_machine_service().clear_calibrated()
    return json_ok(command="CLEAR CALIBRATION", response="Runner locked")


@machine_bp.post("/stepper-hold/apply")
def apply_stepper_hold_policy():
    try:
        result = get_machine_service().apply_stepper_hold_policy_for_test()
        return json_ok(command="APPLY STEPPER HOLD POLICY", response=f"$1={result['applied_dollar_1']}")
    except Exception as exc:
        log_exception("Stepper hold policy test failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/y-loop/start")
def start_y_loop():
    try:
        response = get_machine_service().start_y_loop_test(request.get_json(force=True), current_app.config)
        return json_ok(command="START Y LOOP TEST", response=response)
    except Exception as exc:
        log_exception("Start Y loop failed", exc)
        return json_error(str(exc), status=500)


@machine_bp.post("/y-loop/stop")
def stop_y_loop():
    try:
        response = get_machine_service().stop_y_loop_test()
        return json_ok(command="STOP Y LOOP TEST", response=response)
    except Exception as exc:
        log_exception("Stop Y loop failed", exc)
        return json_error(str(exc), status=500)
