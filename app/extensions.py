from __future__ import annotations

from flask import Flask, current_app

from .models.machine_state import MachineState
from .services.gcode_service import GcodeService
from .services.geometry_service import GeometryService
from .services.job_runner import JobRunner
from .services.machine_service import MachineService
from .services.raster_analysis_service import RasterAnalysisService
from .services.self_test_service import SelfTestService
from .services.serial_service import SerialService
from .services.svg_parser import SvgParser
from .services.toolpath_service import ToolpathService
from .services.validation_service import ValidationService


def init_extensions(app: Flask) -> None:
    state = MachineState(default_pen_up_s=app.config["DEFAULT_PEN_UP_S"])
    serial_service = SerialService(app.config, state)
    validation_service = ValidationService()
    svg_parser = SvgParser(app.config, state)
    raster_analysis_service = RasterAnalysisService(app.config, state)
    geometry_service = GeometryService()
    toolpath_service = ToolpathService()
    gcode_service = GcodeService()
    self_test_service = SelfTestService()
    job_runner = JobRunner(state=state, serial_service=serial_service, config=app.config)
    machine_service = MachineService(
        config=app.config,
        state=state,
        serial_service=serial_service,
        validation_service=validation_service,
        job_runner=job_runner,
    )
    job_runner.set_before_start(machine_service.prepare_for_job_start)
    job_runner.set_lifecycle_callback(machine_service.handle_job_lifecycle_event)

    app.extensions["machine_state"] = state
    app.extensions["serial_service"] = serial_service
    app.extensions["validation_service"] = validation_service
    app.extensions["svg_parser"] = svg_parser
    app.extensions["raster_analysis_service"] = raster_analysis_service
    app.extensions["geometry_service"] = geometry_service
    app.extensions["toolpath_service"] = toolpath_service
    app.extensions["gcode_service"] = gcode_service
    app.extensions["self_test_service"] = self_test_service
    app.extensions["job_runner"] = job_runner
    app.extensions["machine_service"] = machine_service


def get_state() -> MachineState:
    return current_app.extensions["machine_state"]


def get_serial_service() -> SerialService:
    return current_app.extensions["serial_service"]


def get_validation_service() -> ValidationService:
    return current_app.extensions["validation_service"]


def get_svg_parser() -> SvgParser:
    return current_app.extensions["svg_parser"]


def get_raster_analysis_service() -> RasterAnalysisService:
    return current_app.extensions["raster_analysis_service"]


def get_geometry_service() -> GeometryService:
    return current_app.extensions["geometry_service"]


def get_toolpath_service() -> ToolpathService:
    return current_app.extensions["toolpath_service"]


def get_gcode_service() -> GcodeService:
    return current_app.extensions["gcode_service"]


def get_self_test_service() -> SelfTestService:
    return current_app.extensions["self_test_service"]


def get_job_runner() -> JobRunner:
    return current_app.extensions["job_runner"]


def get_machine_service() -> MachineService:
    return current_app.extensions["machine_service"]
