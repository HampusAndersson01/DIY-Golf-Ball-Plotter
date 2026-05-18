from flask import Blueprint

from app.extensions import get_job_runner, get_machine_service
from app.utils.response_utils import json_error, json_ok, log_exception

job_bp = Blueprint("jobs", __name__)


@job_bp.post("/run-gcode")
def run_gcode_route():
    try:
        get_job_runner().start()
        return json_ok(command="RUN GENERATED G-CODE", response="Started")
    except Exception as exc:
        log_exception("Run G-code failed", exc)
        return json_error(str(exc), status=500)


@job_bp.post("/pause")
def pause():
    try:
        get_job_runner().pause()
        return json_ok(command="FEED HOLD !", response="Pause requested")
    except Exception as exc:
        log_exception("Pause failed", exc)
        return json_error(str(exc), status=500)


@job_bp.post("/resume")
def resume():
    try:
        get_job_runner().resume()
        return json_ok(command="CYCLE START ~", response="Resume requested")
    except Exception as exc:
        log_exception("Resume failed", exc)
        return json_error(str(exc), status=500)


@job_bp.post("/stop")
def stop():
    try:
        response = get_machine_service().stop_active_job()
        return json_ok(command="STOP + SOFT RESET", response=response)
    except Exception as exc:
        log_exception("Stop job failed", exc)
        return json_error(str(exc), status=500)
