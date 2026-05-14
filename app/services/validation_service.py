from __future__ import annotations

from typing import Any

from ._legacy import legacy


class ValidationService:
    validate_feed = staticmethod(legacy.validate_feed)
    validate_degrees = staticmethod(legacy.validate_degrees)
    validate_y_degrees = staticmethod(legacy.validate_y_degrees)
    validate_servo_s = staticmethod(legacy.validate_servo_s)
    validate_dwell = staticmethod(legacy.validate_dwell)
    validate_bool = staticmethod(legacy.validate_bool)
    validate_non_negative_float = staticmethod(legacy.validate_non_negative_float)
    validate_non_negative_int = staticmethod(legacy.validate_non_negative_int)

    def parse_generate_gcode_form(self, form, config) -> dict[str, Any]:
        options = {
            "draw_feed": self.validate_feed(form.get("draw_feed", config["DEFAULT_DRAW_FEED"])),
            "travel_feed": self.validate_feed(form.get("travel_feed", config["DEFAULT_TRAVEL_FEED"])),
            "sample_step_deg": float(form.get("sample_step_deg", config["DEFAULT_SAMPLE_STEP_DEG"])),
            "margin_percent": float(form.get("margin_percent", config["DEFAULT_MARGIN_PERCENT"])),
            "placement_scale": float(form.get("placement_scale", 100.0)),
            "placement_offset_x": float(form.get("placement_offset_x", 0.0)),
            "placement_offset_y": float(form.get("placement_offset_y", 0.0)),
            "rotation_deg": float(form.get("rotation_deg", config["DEFAULT_ROTATION_DEG"])),
            "parser_mode": form.get("parser_mode", config["DEFAULT_PARSER_MODE"]),
            "color_mapping_mode": self.validate_bool(form.get("color_mapping_mode", config["DEFAULT_COLOR_MAPPING_MODE"])),
            "line_thickness_mm": float(form.get("line_thickness_mm", config["DEFAULT_LINE_THICKNESS_MM"])),
            "enable_fill": self.validate_bool(form.get("enable_fill", config["DEFAULT_ENABLE_FILL"])),
            "trace_stroke_only_paths": self.validate_bool(form.get("trace_stroke_only_paths", config["DEFAULT_TRACE_STROKE_ONLY_PATHS"])),
            "fill_only_dark_svg_fills": self.validate_bool(
                form.get("fill_only_dark_svg_fills", config["DEFAULT_FILL_ONLY_DARK_SVG_FILLS"])
            ),
            "fill_mode": form.get("fill_mode", config["DEFAULT_FILL_MODE"]),
            "wall_count": self.validate_non_negative_int(form.get("wall_count", config["DEFAULT_WALL_COUNT"]), "Wall count", minimum=1, maximum=8),
            "infill_pattern": form.get("infill_pattern", config["DEFAULT_INFILL_PATTERN"]),
            "infill_density": self.validate_non_negative_float(form.get("infill_density", config["DEFAULT_INFILL_DENSITY"]), "Infill density", maximum=100),
            "infill_spacing_mm": self.validate_non_negative_float(form.get("infill_spacing_mm", config["DEFAULT_INFILL_SPACING_MM"]), "Infill spacing", maximum=10),
            "infill_angle_deg": float(form.get("infill_angle_deg", config["DEFAULT_INFILL_ANGLE_DEG"])),
            "outline_after_fill": self.validate_bool(form.get("outline_after_fill", config["DEFAULT_OUTLINE_AFTER_FILL"])),
            "min_fill_area_mm2": self.validate_non_negative_float(form.get("min_fill_area_mm2", config["DEFAULT_MIN_FILL_AREA_MM2"]), "Minimum fill area", maximum=10000),
            "min_fill_width_mm": self.validate_non_negative_float(form.get("min_fill_width_mm", config["DEFAULT_MIN_FILL_WIDTH_MM"]), "Minimum fill width", maximum=10),
            "simplify_tolerance_mm": self.validate_non_negative_float(form.get("simplify_tolerance_mm", config["DEFAULT_SIMPLIFY_TOLERANCE_MM"]), "Simplify tolerance", maximum=5),
            "remove_duplicate_paths": self.validate_bool(form.get("remove_duplicate_paths", config["DEFAULT_REMOVE_DUPLICATE_PATHS"])),
            "small_shape_mode": form.get("small_shape_mode", config["DEFAULT_SMALL_SHAPE_MODE"]),
            "min_segment_length_mm": self.validate_non_negative_float(form.get("min_segment_length_mm", config["DEFAULT_MIN_SEGMENT_LENGTH_MM"]), "Minimum segment length", maximum=20),
            "travel_optimization": form.get("travel_optimization", config["DEFAULT_TRAVEL_OPTIMIZATION"]),
            "fit_mode": form.get("fit_mode", "contain"),
            "invert_y": form.get("invert_y", "1") == "1",
            "include_comments": form.get("include_comments", "1") == "1",
            "pen_up_s": self.validate_servo_s(form.get("pen_up_s", config["DEFAULT_PEN_UP_S"])),
            "pen_down_s": self.validate_servo_s(form.get("pen_down_s", config["DEFAULT_PEN_DOWN_S"])),
            "servo_ramp_enabled": self.validate_bool(form.get("servo_ramp_enabled", config["DEFAULT_SERVO_RAMP_ENABLED"])),
            "servo_ramp_step": self.validate_non_negative_int(form.get("servo_ramp_step", config["DEFAULT_SERVO_RAMP_STEP"]), "Servo ramp step", minimum=1, maximum=200),
            "servo_ramp_delay_ms": self.validate_non_negative_float(form.get("servo_ramp_delay_ms", config["DEFAULT_SERVO_RAMP_DELAY_MS"]), "Servo ramp delay", maximum=1000),
            "pen_up_dwell_ms": self.validate_non_negative_float(form.get("pen_up_dwell_ms", config["DEFAULT_PEN_UP_DWELL_MS"]), "Pen up dwell", maximum=5000),
            "pen_down_dwell_ms": self.validate_non_negative_float(form.get("pen_down_dwell_ms", config["DEFAULT_PEN_DOWN_DWELL_MS"]), "Pen down dwell", maximum=5000),
            "debug_pipeline": self.validate_bool(form.get("debug_pipeline", "0")),
        }
        if options["sample_step_deg"] <= 0:
            raise ValueError("Sample step must be greater than 0")
        if options["margin_percent"] < 0 or options["margin_percent"] > 25:
            raise ValueError("Margin percent must be between 0 and 25")
        if options["line_thickness_mm"] < 0 or options["line_thickness_mm"] > 10:
            raise ValueError("Line thickness must be between 0 and 10 mm")
        if options["fit_mode"] not in {"contain", "stretch"}:
            raise ValueError("Invalid fit mode")
        if options["parser_mode"] not in {"visible_geometry", "detect_visible_print_areas"}:
            raise ValueError("Invalid parser mode")
        if options["fill_mode"] != "slicer":
            raise ValueError("Only slicer fill mode is currently supported")
        if options["infill_pattern"] not in {"zigzag", "hatch"}:
            raise ValueError("Infill pattern must be zigzag or hatch")
        if options["infill_density"] <= 0 or options["infill_density"] > 100:
            raise ValueError("Infill density must be between 0 and 100")
        if options["small_shape_mode"] not in {"single-wall", "skip", "centerline"}:
            raise ValueError("Invalid small shape mode")
        if options["travel_optimization"] not in {"nearest-neighbor"}:
            raise ValueError("Invalid travel optimization")
        return options

    def parse_analyze_svg_form(self, form, config) -> dict[str, Any]:
        options = {
            "parser_mode": form.get("parser_mode", config["DEFAULT_PARSER_MODE"]),
            "color_mapping_mode": self.validate_bool(form.get("color_mapping_mode", config["DEFAULT_COLOR_MAPPING_MODE"])),
            "trace_stroke_only_paths": self.validate_bool(
                form.get("trace_stroke_only_paths", config["DEFAULT_TRACE_STROKE_ONLY_PATHS"])
            ),
            "fill_only_dark_svg_fills": self.validate_bool(
                form.get("fill_only_dark_svg_fills", config["DEFAULT_FILL_ONLY_DARK_SVG_FILLS"])
            ),
            "debug_pipeline": self.validate_bool(form.get("debug_pipeline", "0")),
        }
        if options["parser_mode"] not in {"visible_geometry", "detect_visible_print_areas"}:
            raise ValueError("Invalid parser mode")
        return options
