import logging
import threading
import time

from flask import Blueprint, Response, current_app, jsonify

from app.extensions import get_state
from app.services.runtime_estimation_service import build_runtime_snapshot

ui_bp = Blueprint("ui", __name__)
logger = logging.getLogger(__name__)
_state_poll_lock = threading.Lock()
_state_poll_started_at = time.monotonic()
_state_poll_count = 0


def build_frontend_config(config) -> dict:
    return {
        "defaults": {
            "xMaxFeed": config["DEFAULT_X_MAX_FEED"],
            "yMaxFeed": config["DEFAULT_Y_MAX_FEED"],
            "xAcceleration": config["DEFAULT_X_ACCELERATION"],
            "yAcceleration": config["DEFAULT_Y_ACCELERATION"],
            "drawFeed": config["DEFAULT_DRAW_FEED"],
            "travelFeed": config["DEFAULT_TRAVEL_FEED"],
            "artworkScalePercent": 100,
            "originAnchor": "center",
            "originOffsetXmm": 0.0,
            "originOffsetYmm": 0.0,
            "lineThicknessMm": config["DEFAULT_LINE_THICKNESS_MM"],
            "penUpS": config["DEFAULT_PEN_UP_S"],
            "penDownS": config["DEFAULT_PEN_DOWN_S"],
            "penUpDwellMs": config["DEFAULT_PEN_UP_DWELL_MS"],
            "penDownDwellMs": config["DEFAULT_PEN_DOWN_DWELL_MS"],
            "servoRampEnabled": config["DEFAULT_SERVO_RAMP_ENABLED"],
            "servoRampStep": config["DEFAULT_SERVO_RAMP_STEP"],
            "servoRampDelayMs": config["DEFAULT_SERVO_RAMP_DELAY_MS"],
            "sampleStepDeg": config["DEFAULT_SAMPLE_STEP_DEG"],
            "maxPrintXSpanDeg": config["DEFAULT_MAX_PRINT_X_SPAN_DEG"],
            "marginPercent": config["DEFAULT_MARGIN_PERCENT"],
            "rotationDeg": config["DEFAULT_ROTATION_DEG"],
            "wallCount": config["DEFAULT_WALL_COUNT"],
            "infillDensity": config["DEFAULT_INFILL_DENSITY"],
            "infillSpacingMm": config["DEFAULT_INFILL_SPACING_MM"],
            "customInfillSpacingEnabled": False,
            "infillAngleDeg": config["DEFAULT_INFILL_ANGLE_DEG"],
            "fillStrategy": config["DEFAULT_FILL_STRATEGY"],
            "alternateFillAngleDeg": config["DEFAULT_ALTERNATE_FILL_ANGLE_DEG"],
            "minFillAreaMm2": config["DEFAULT_MIN_FILL_AREA_MM2"],
            "minFillWidthMm": config["DEFAULT_MIN_FILL_WIDTH_MM"],
            "simplifyToleranceMm": config["DEFAULT_SIMPLIFY_TOLERANCE_MM"],
            "removeDuplicatePaths": config["DEFAULT_REMOVE_DUPLICATE_PATHS"],
            "minSegmentLengthMm": config["DEFAULT_MIN_SEGMENT_LENGTH_MM"],
            "allowPenDownInfillConnectors": config["DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS"],
            "thinDetailMode": config["DEFAULT_THIN_DETAIL_MODE"],
            "thinDetailMinAreaMm2": config["DEFAULT_THIN_DETAIL_MIN_AREA_MM2"],
            "thinDetailSimplifyMm": config["DEFAULT_THIN_DETAIL_SIMPLIFY_MM"],
            "thinDetailOverlap": config["DEFAULT_THIN_DETAIL_OVERLAP"],
            "rasterMaxColors": config["DEFAULT_RASTER_MAX_COLORS"],
            "rasterColorTolerance": config["DEFAULT_RASTER_COLOR_TOLERANCE"],
            "rasterMinComponentAreaPx": config["DEFAULT_RASTER_MIN_COMPONENT_AREA_PX"],
            "rasterMaskOpenRadiusPx": config["DEFAULT_RASTER_MASK_OPEN_RADIUS_PX"],
            "rasterMaskCloseRadiusPx": config["DEFAULT_RASTER_MASK_CLOSE_RADIUS_PX"],
            "rasterMinRegionAreaPx": config["DEFAULT_RASTER_MIN_REGION_AREA_PX"],
            "rasterRegionSimplifyPx": config["DEFAULT_RASTER_REGION_SIMPLIFY_PX"],
            "outlineAfterFill": config["DEFAULT_OUTLINE_AFTER_FILL"],
            "streamingMode": config["DEFAULT_STREAMING_MODE"],
            "releaseIdleDelayMs": config["STEPPER_RELEASE_IDLE_DELAY_VALUE"],
            "yLoopDistance": 10.0,
            "yLoopFeedrate": config["DEFAULT_DRAW_FEED"],
            "yLoopDwellSec": 0.25,
        },
    }


@ui_bp.get("/")
def index():
    return jsonify({
        "ok": True,
        "service": "golfball-plotter-backend",
        "message": "Flask is running as the backend API. Start the React dashboard with Vite on http://127.0.0.1:5173.",
        "bootstrap": "/api/bootstrap",
    })


@ui_bp.get("/api/bootstrap")
def frontend_bootstrap():
    return jsonify({
        "ok": True,
        **build_frontend_config(current_app.config),
    })


@ui_bp.get("/state")
def get_machine_state():
    global _state_poll_started_at, _state_poll_count
    snapshot = get_state().snapshot()
    snapshot.update(build_runtime_snapshot(snapshot))
    config = current_app.config
    snapshot["defaults"] = {
        "pen_up_s": config["DEFAULT_PEN_UP_S"],
        "pen_down_s": config["DEFAULT_PEN_DOWN_S"],
        "pen_up_dwell_ms": config["DEFAULT_PEN_UP_DWELL_MS"],
        "pen_down_dwell_ms": config["DEFAULT_PEN_DOWN_DWELL_MS"],
        "servo_ramp_enabled": config["DEFAULT_SERVO_RAMP_ENABLED"],
        "servo_ramp_step": config["DEFAULT_SERVO_RAMP_STEP"],
        "servo_ramp_delay_ms": config["DEFAULT_SERVO_RAMP_DELAY_MS"],
    }
    with _state_poll_lock:
        _state_poll_count += 1
        elapsed = time.monotonic() - _state_poll_started_at
        if elapsed >= 60.0:
            logger.info(
                "State polling summary: requests=%d duration_sec=%.1f connected=%s calibrated=%s running=%s paused=%s status=%s",
                _state_poll_count,
                elapsed,
                snapshot.get("connected"),
                snapshot.get("calibrated"),
                snapshot.get("running"),
                snapshot.get("paused"),
                snapshot.get("status"),
            )
            _state_poll_started_at = time.monotonic()
            _state_poll_count = 0
    return jsonify(snapshot)


@ui_bp.get("/favicon.ico")
def favicon():
    return Response(status=204)
