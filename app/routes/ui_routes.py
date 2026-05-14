from flask import Blueprint, current_app, jsonify, render_template

from app.extensions import get_state

ui_bp = Blueprint("ui", __name__)


@ui_bp.get("/")
def index():
    config = current_app.config
    return render_template(
        "index.html",
        x_steps_per_degree=f"{config['MOTOR_FULL_STEPS_PER_REV'] * config['X_MICROSTEPS'] / 360.0:.6f}",
        y_steps_per_degree=f"{config['MOTOR_FULL_STEPS_PER_REV'] * config['Y_MICROSTEPS'] / 360.0:.6f}",
        default_x_max_feed=config["DEFAULT_X_MAX_FEED"],
        default_y_max_feed=config["DEFAULT_Y_MAX_FEED"],
        default_x_acceleration=config["DEFAULT_X_ACCELERATION"],
        default_y_acceleration=config["DEFAULT_Y_ACCELERATION"],
        default_draw_feed=config["DEFAULT_DRAW_FEED"],
        default_travel_feed=config["DEFAULT_TRAVEL_FEED"],
        default_line_thickness_mm=config["DEFAULT_LINE_THICKNESS_MM"],
        default_pen_up_s=config["DEFAULT_PEN_UP_S"],
        default_pen_down_s=config["DEFAULT_PEN_DOWN_S"],
        default_servo_dwell=config["DEFAULT_SERVO_DWELL"],
        default_servo_ramp_enabled=config["DEFAULT_SERVO_RAMP_ENABLED"],
        default_servo_ramp_step=config["DEFAULT_SERVO_RAMP_STEP"],
        default_servo_ramp_delay_ms=config["DEFAULT_SERVO_RAMP_DELAY_MS"],
        default_pen_up_dwell_ms=config["DEFAULT_PEN_UP_DWELL_MS"],
        default_pen_down_dwell_ms=config["DEFAULT_PEN_DOWN_DWELL_MS"],
        default_sample_step_deg=config["DEFAULT_SAMPLE_STEP_DEG"],
        default_margin_percent=config["DEFAULT_MARGIN_PERCENT"],
        default_rotation_deg=config["DEFAULT_ROTATION_DEG"],
        default_parser_mode=config["DEFAULT_PARSER_MODE"],
        default_color_mapping_mode=config["DEFAULT_COLOR_MAPPING_MODE"],
        default_enable_fill=config["DEFAULT_ENABLE_FILL"],
        default_trace_stroke_only_paths=config["DEFAULT_TRACE_STROKE_ONLY_PATHS"],
        default_fill_only_dark_svg_fills=config["DEFAULT_FILL_ONLY_DARK_SVG_FILLS"],
        default_wall_count=config["DEFAULT_WALL_COUNT"],
        default_infill_density=config["DEFAULT_INFILL_DENSITY"],
        default_infill_spacing_mm=config["DEFAULT_INFILL_SPACING_MM"],
        default_infill_angle_deg=config["DEFAULT_INFILL_ANGLE_DEG"],
        default_min_fill_area_mm2=config["DEFAULT_MIN_FILL_AREA_MM2"],
        default_min_fill_width_mm=config["DEFAULT_MIN_FILL_WIDTH_MM"],
        default_simplify_tolerance_mm=config["DEFAULT_SIMPLIFY_TOLERANCE_MM"],
        default_outline_after_fill=config["DEFAULT_OUTLINE_AFTER_FILL"],
        default_remove_duplicate_paths=config["DEFAULT_REMOVE_DUPLICATE_PATHS"],
        default_min_segment_length_mm=config["DEFAULT_MIN_SEGMENT_LENGTH_MM"],
        default_travel_optimization=config["DEFAULT_TRAVEL_OPTIMIZATION"],
    )


@ui_bp.get("/state")
def get_machine_state():
    snapshot = get_state().snapshot()
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
    return jsonify(snapshot)
