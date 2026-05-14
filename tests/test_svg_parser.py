from app.models.machine_state import MachineState
from app.services.svg_parser import SvgParser


CONFIG = {
    "DEFAULT_PEN_UP_S": 575,
    "SERIAL_PORT": "COM12",
    "BAUD_RATE": 115200,
    "MOTOR_FULL_STEPS_PER_REV": 200,
    "X_MICROSTEPS": 16,
    "Y_MICROSTEPS": 16,
    "X_DRAW_MIN": -180.0,
    "X_DRAW_MAX": 180.0,
    "Y_DRAW_MIN": -45.0,
    "Y_DRAW_MAX": 45.0,
    "BALL_CENTER_X": 0.0,
    "BALL_CENTER_Y": 0.0,
    "BALL_DIAMETER_MM": 42.67,
    "DEFAULT_X_MAX_FEED": 6000.0,
    "DEFAULT_Y_MAX_FEED": 6000.0,
    "DEFAULT_X_ACCELERATION": 100.0,
    "DEFAULT_Y_ACCELERATION": 100.0,
    "DEFAULT_DRAW_FEED": 1200.0,
    "DEFAULT_TRAVEL_FEED": 3000.0,
    "DEFAULT_LINE_THICKNESS_MM": 0.75,
    "DEFAULT_PEN_DOWN_S": 700,
    "DEFAULT_SERVO_DWELL": 0.06,
    "DEFAULT_SERVO_RAMP_ENABLED": True,
    "DEFAULT_SERVO_RAMP_STEP": 20,
    "DEFAULT_SERVO_RAMP_DELAY_MS": 10.0,
    "DEFAULT_PEN_UP_DWELL_MS": 30.0,
    "DEFAULT_PEN_DOWN_DWELL_MS": 60.0,
    "MIN_SERVO_S": 500,
    "MAX_SERVO_S": 1000,
    "DEFAULT_SAMPLE_STEP_DEG": 1.0,
    "DEFAULT_CURVE_SAMPLES": 80,
    "DEFAULT_MARGIN_PERCENT": 4.0,
    "DEFAULT_ROTATION_DEG": 0.0,
    "DEFAULT_ENABLE_FILL": True,
    "DEFAULT_FILL_MODE": "slicer",
    "DEFAULT_PARSER_MODE": "visible_geometry",
    "DEFAULT_COLOR_MAPPING_MODE": False,
    "DEFAULT_TRACE_STROKE_ONLY_PATHS": True,
    "DEFAULT_FILL_ONLY_DARK_SVG_FILLS": True,
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
    "SVG_DARK_FILL_LUMINANCE_THRESHOLD": 0.42,
    "SVG_LIGHT_CUTOUT_LUMINANCE_THRESHOLD": 0.82,
    "SVG_MIN_PRINT_OPACITY": 0.99,
}


def make_parser() -> SvgParser:
    return SvgParser(CONFIG, MachineState(default_pen_up_s=575))


def test_black_fill_with_white_cutout():
    svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <rect x="0" y="0" width="100" height="100" fill="#111111"/>
      <rect x="30" y="30" width="40" height="40" fill="white"/>
    </svg>
    """
    result = make_parser().analyze_svg(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    assert len(result.bundle.fill_shapes) == 1
    assert abs(result.bundle.fill_shapes[0].geometry.area - 8400.0) < 0.01


def test_stroke_only_svg():
    svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10">
      <rect x="1" y="1" width="8" height="8" stroke="#000" stroke-width="0.5" fill="none"/>
    </svg>
    """
    result = make_parser().analyze_svg(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    assert result.bundle.outline_segments
    assert not result.bundle.fill_shapes


def test_inherited_group_style():
    svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <g style="fill: rgb(0,0,0); fill-opacity: 1;">
        <rect x="0" y="0" width="40" height="40"/>
      </g>
      <g style="fill: rgba(255,255,255,0.9);">
        <rect x="10" y="10" width="20" height="20"/>
      </g>
    </svg>
    """
    result = make_parser().analyze_svg(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    assert abs(result.bundle.fill_shapes[0].geometry.area - 1200.0) < 0.01


def test_compound_path_holes():
    svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <path fill="#000000" d="M 0 0 L 100 0 L 100 100 L 0 100 Z M 30 30 L 30 70 L 70 70 L 70 30 Z"/>
    </svg>
    """
    result = make_parser().analyze_svg(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    assert abs(result.bundle.fill_shapes[0].geometry.area - 8400.0) < 0.01


def test_transparent_fill_cutout():
    svg = """
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
      <rect x="0" y="0" width="100" height="100" fill="#000000"/>
      <rect x="30" y="30" width="40" height="40" fill="#000000" fill-opacity="0.2"/>
    </svg>
    """
    result = make_parser().analyze_svg(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    assert abs(result.bundle.fill_shapes[0].geometry.area - 8400.0) < 0.01


def test_no_drawable_geometry_diagnostics():
    svg = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 10"><text>Hello</text></svg>'
    result = make_parser().analyze_svg(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    assert result.print_model.diagnostics
