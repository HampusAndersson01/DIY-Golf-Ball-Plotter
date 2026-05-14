import pytest

from app.services.validation_service import ValidationService


def make_config():
    return {
        "DEFAULT_DRAW_FEED": 1200.0,
        "DEFAULT_TRAVEL_FEED": 3000.0,
        "DEFAULT_SAMPLE_STEP_DEG": 1.0,
        "DEFAULT_MARGIN_PERCENT": 4.0,
        "DEFAULT_ROTATION_DEG": 0.0,
        "DEFAULT_PARSER_MODE": "visible_geometry",
        "DEFAULT_COLOR_MAPPING_MODE": False,
        "DEFAULT_LINE_THICKNESS_MM": 0.75,
        "DEFAULT_ENABLE_FILL": True,
        "DEFAULT_TRACE_STROKE_ONLY_PATHS": True,
        "DEFAULT_FILL_ONLY_DARK_SVG_FILLS": True,
        "DEFAULT_FILL_MODE": "slicer",
        "DEFAULT_WALL_COUNT": 1,
        "DEFAULT_INFILL_PATTERN": "zigzag",
        "DEFAULT_INFILL_DENSITY": 100.0,
        "DEFAULT_INFILL_SPACING_MM": 0.75,
        "DEFAULT_INFILL_ANGLE_DEG": 0.0,
        "DEFAULT_OUTLINE_AFTER_FILL": False,
        "DEFAULT_MIN_FILL_AREA_MM2": 1.0,
        "DEFAULT_MIN_FILL_WIDTH_MM": 0.75,
        "DEFAULT_SIMPLIFY_TOLERANCE_MM": 0.05,
        "DEFAULT_REMOVE_DUPLICATE_PATHS": True,
        "DEFAULT_SMALL_SHAPE_MODE": "single-wall",
        "DEFAULT_MIN_SEGMENT_LENGTH_MM": 0.5,
        "DEFAULT_TRAVEL_OPTIMIZATION": "nearest-neighbor",
        "DEFAULT_PEN_UP_S": 575,
        "DEFAULT_PEN_DOWN_S": 700,
        "DEFAULT_SERVO_RAMP_ENABLED": True,
        "DEFAULT_SERVO_RAMP_STEP": 20,
        "DEFAULT_SERVO_RAMP_DELAY_MS": 10.0,
        "DEFAULT_PEN_UP_DWELL_MS": 30.0,
        "DEFAULT_PEN_DOWN_DWELL_MS": 60.0,
    }


def test_invalid_margin_percent_raises():
    service = ValidationService()
    with pytest.raises(ValueError, match="Margin percent must be between 0 and 25"):
        service.parse_generate_gcode_form({"margin_percent": "30"}, make_config())


def test_invalid_servo_value_raises():
    service = ValidationService()
    with pytest.raises(ValueError):
        service.validate_servo_s(1001)
