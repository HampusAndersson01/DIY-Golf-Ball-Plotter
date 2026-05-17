from __future__ import annotations

from typing import Any

from . import pipeline_core
from .raster_analysis_service import RasterAnalysisService


class ValidationService:
    validate_feed = staticmethod(pipeline_core.validate_feed)
    validate_degrees = staticmethod(pipeline_core.validate_degrees)
    validate_y_degrees = staticmethod(pipeline_core.validate_y_degrees)
    validate_servo_s = staticmethod(pipeline_core.validate_servo_s)
    validate_dwell = staticmethod(pipeline_core.validate_dwell)
    validate_bool = staticmethod(pipeline_core.validate_bool)
    parse_locale_float = staticmethod(pipeline_core.parse_locale_float)
    validate_non_negative_float = staticmethod(pipeline_core.validate_non_negative_float)
    validate_non_negative_int = staticmethod(pipeline_core.validate_non_negative_int)

    @staticmethod
    def _form_value(form, key: str):
        value = form.get(key)
        if value is None:
            return None
        if isinstance(value, str) and value.strip() == "":
            return None
        return value

    def _derived_pen_defaults(self, line_thickness_mm: float) -> dict[str, float]:
        return {
            "infill_spacing_mm": line_thickness_mm,
            "min_fill_width_mm": line_thickness_mm,
            "min_fill_area_mm2": line_thickness_mm * line_thickness_mm,
        }

    def _parse_float(self, form, key: str, default: float) -> float:
        return self.parse_locale_float(form.get(key, default), default)

    def parse_generate_gcode_form(self, form, config) -> dict[str, Any]:
        line_thickness_mm = self._parse_float(form, "line_thickness_mm", config["DEFAULT_LINE_THICKNESS_MM"])
        derived_defaults = self._derived_pen_defaults(line_thickness_mm)
        custom_infill_spacing = self.validate_bool(form.get("custom_infill_spacing", "0"))
        options = {
            "draw_feed": self.validate_feed(form.get("draw_feed", config["DEFAULT_DRAW_FEED"])),
            "travel_feed": self.validate_feed(form.get("travel_feed", config["DEFAULT_TRAVEL_FEED"])),
            "sample_step_deg": self._parse_float(form, "sample_step_deg", config["DEFAULT_SAMPLE_STEP_DEG"]),
            "margin_percent": self._parse_float(form, "margin_percent", config["DEFAULT_MARGIN_PERCENT"]),
            "placement_scale": self._parse_float(form, "placement_scale", 100.0),
            "placement_offset_x": self._parse_float(form, "placement_offset_x", 0.0),
            "placement_offset_y": self._parse_float(form, "placement_offset_y", 0.0),
            "rotation_deg": self._parse_float(form, "rotation_deg", config["DEFAULT_ROTATION_DEG"]),
            "parser_mode": form.get("parser_mode", config["DEFAULT_PARSER_MODE"]),
            "color_mapping_mode": self.validate_bool(form.get("color_mapping_mode", config["DEFAULT_COLOR_MAPPING_MODE"])),
            "line_thickness_mm": line_thickness_mm,
            "enable_fill": self.validate_bool(form.get("enable_fill", config["DEFAULT_ENABLE_FILL"])),
            "trace_stroke_only_paths": self.validate_bool(form.get("trace_stroke_only_paths", config["DEFAULT_TRACE_STROKE_ONLY_PATHS"])),
            "fill_only_dark_svg_fills": self.validate_bool(
                form.get("fill_only_dark_svg_fills", config["DEFAULT_FILL_ONLY_DARK_SVG_FILLS"])
            ),
            "fill_mode": form.get("fill_mode", config["DEFAULT_FILL_MODE"]),
            "wall_count": self.validate_non_negative_int(form.get("wall_count", config["DEFAULT_WALL_COUNT"]), "Wall count", minimum=1, maximum=8),
            "custom_infill_spacing": custom_infill_spacing,
            "infill_pattern": form.get("infill_pattern", config["DEFAULT_INFILL_PATTERN"]),
            "infill_density": self.validate_non_negative_float(form.get("infill_density", config["DEFAULT_INFILL_DENSITY"]), "Infill density", maximum=100),
            "infill_spacing_mm": self.validate_non_negative_float(
                self._form_value(form, "infill_spacing_mm")
                if custom_infill_spacing and self._form_value(form, "infill_spacing_mm") is not None
                else derived_defaults["infill_spacing_mm"],
                "Infill spacing",
                maximum=10,
            ),
            "infill_angle_deg": self._parse_float(form, "infill_angle_deg", config["DEFAULT_INFILL_ANGLE_DEG"]),
            "fill_strategy": form.get("fill_strategy", config.get("DEFAULT_FILL_STRATEGY", "adaptive_angle")),
            "alternate_fill_angle_deg": self._parse_float(form, "alternate_fill_angle_deg", config.get("DEFAULT_ALTERNATE_FILL_ANGLE_DEG", -45.0)),
            "outline_after_fill": self.validate_bool(form.get("outline_after_fill", config["DEFAULT_OUTLINE_AFTER_FILL"])),
            "min_fill_area_mm2": self.validate_non_negative_float(
                self._form_value(form, "min_fill_area_mm2") if self._form_value(form, "min_fill_area_mm2") is not None else derived_defaults["min_fill_area_mm2"],
                "Minimum fill area",
                maximum=10000,
            ),
            "min_fill_width_mm": self.validate_non_negative_float(
                self._form_value(form, "min_fill_width_mm") if self._form_value(form, "min_fill_width_mm") is not None else derived_defaults["min_fill_width_mm"],
                "Minimum fill width",
                maximum=10,
            ),
            "simplify_tolerance_mm": self.validate_non_negative_float(form.get("simplify_tolerance_mm", config["DEFAULT_SIMPLIFY_TOLERANCE_MM"]), "Simplify tolerance", maximum=5),
            "remove_duplicate_paths": self.validate_bool(form.get("remove_duplicate_paths", config["DEFAULT_REMOVE_DUPLICATE_PATHS"])),
            "small_shape_mode": form.get("small_shape_mode", config["DEFAULT_SMALL_SHAPE_MODE"]),
            "thin_detail_mode": self.validate_bool(form.get("thin_detail_mode", config["DEFAULT_THIN_DETAIL_MODE"])),
            "thin_detail_min_area_mm2": self.validate_non_negative_float(
                form.get("thin_detail_min_area_mm2", config["DEFAULT_THIN_DETAIL_MIN_AREA_MM2"]),
                "Thin detail minimum area",
                maximum=10000,
            ),
            "thin_detail_simplify_mm": self.validate_non_negative_float(
                form.get("thin_detail_simplify_mm", config["DEFAULT_THIN_DETAIL_SIMPLIFY_MM"]),
                "Thin detail simplify",
                maximum=5,
            ),
            "thin_detail_overlap": self.validate_bool(form.get("thin_detail_overlap", config["DEFAULT_THIN_DETAIL_OVERLAP"])),
            "min_segment_length_mm": self.validate_non_negative_float(form.get("min_segment_length_mm", config["DEFAULT_MIN_SEGMENT_LENGTH_MM"]), "Minimum segment length", maximum=20),
            "travel_optimization": form.get("travel_optimization", config["DEFAULT_TRAVEL_OPTIMIZATION"]),
            "allow_pen_down_infill_connectors": self.validate_bool(form.get("allow_pen_down_infill_connectors", config["DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS"])),
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
            "gcode_mode": form.get("gcode_mode", config["DEFAULT_GCODE_MODE"]),
            "debug_pipeline": self.validate_bool(form.get("debug_pipeline", "0")),
        }
        options["effective_infill_spacing_mm"] = options["infill_spacing_mm"]
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
        if options["fill_strategy"] not in {"horizontal_scanline", "rotated_scanline", "adaptive_angle", "crosshatch"}:
            raise ValueError("Invalid fill strategy")
        if options["infill_density"] <= 0 or options["infill_density"] > 100:
            raise ValueError("Infill density must be between 0 and 100")
        if options["small_shape_mode"] not in {"single-wall", "skip", "centerline"}:
            raise ValueError("Invalid small shape mode")
        if options["travel_optimization"] not in {"nearest-neighbor"}:
            raise ValueError("Invalid travel optimization")
        if options["gcode_mode"] not in {"simple"}:
            raise ValueError("Invalid G-code mode")
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

    def parse_analyze_raster_form(self, form, config) -> dict[str, Any]:
        options = {
            "simplify_colors": self.validate_bool(form.get("simplify_colors", "1")),
            "max_colors": self.validate_non_negative_int(
                form.get("max_colors", config["DEFAULT_RASTER_MAX_COLORS"]),
                "Max colors",
                minimum=2,
                maximum=16,
            ),
        }
        return options

    def parse_generate_raster_form(self, form, config) -> dict[str, Any]:
        line_thickness_mm = self._parse_float(form, "line_thickness_mm", config["DEFAULT_LINE_THICKNESS_MM"])
        derived_defaults = self._derived_pen_defaults(line_thickness_mm)
        custom_infill_spacing = self.validate_bool(form.get("custom_infill_spacing", "0"))
        options = {
            "draw_feed": self.validate_feed(form.get("draw_feed", config["DEFAULT_DRAW_FEED"])),
            "travel_feed": self.validate_feed(form.get("travel_feed", config["DEFAULT_TRAVEL_FEED"])),
            "sample_step_deg": self._parse_float(form, "sample_step_deg", config["DEFAULT_SAMPLE_STEP_DEG"]),
            "margin_percent": self._parse_float(form, "margin_percent", config["DEFAULT_MARGIN_PERCENT"]),
            "placement_scale": self._parse_float(form, "placement_scale", 100.0),
            "placement_offset_x": self._parse_float(form, "placement_offset_x", 0.0),
            "placement_offset_y": self._parse_float(form, "placement_offset_y", 0.0),
            "rotation_deg": self._parse_float(form, "rotation_deg", config["DEFAULT_ROTATION_DEG"]),
            "line_thickness_mm": line_thickness_mm,
            "wall_count": self.validate_non_negative_int(form.get("wall_count", config["DEFAULT_WALL_COUNT"]), "Wall count", minimum=1, maximum=8),
            "custom_infill_spacing": custom_infill_spacing,
            "infill_pattern": form.get("infill_pattern", config["DEFAULT_INFILL_PATTERN"]),
            "infill_density": self.validate_non_negative_float(form.get("infill_density", config["DEFAULT_INFILL_DENSITY"]), "Infill density", maximum=100),
            "infill_spacing_mm": self.validate_non_negative_float(
                self._form_value(form, "infill_spacing_mm")
                if custom_infill_spacing and self._form_value(form, "infill_spacing_mm") is not None
                else derived_defaults["infill_spacing_mm"],
                "Infill spacing",
                maximum=10,
            ),
            "infill_angle_deg": self._parse_float(form, "infill_angle_deg", config["DEFAULT_INFILL_ANGLE_DEG"]),
            "fill_strategy": form.get("fill_strategy", config.get("DEFAULT_FILL_STRATEGY", "adaptive_angle")),
            "alternate_fill_angle_deg": self._parse_float(form, "alternate_fill_angle_deg", config.get("DEFAULT_ALTERNATE_FILL_ANGLE_DEG", -45.0)),
            "outline_after_fill": self.validate_bool(form.get("outline_after_fill", config["DEFAULT_OUTLINE_AFTER_FILL"])),
            "min_fill_area_mm2": self.validate_non_negative_float(
                self._form_value(form, "min_fill_area_mm2") if self._form_value(form, "min_fill_area_mm2") is not None else derived_defaults["min_fill_area_mm2"],
                "Minimum fill area",
                maximum=10000,
            ),
            "min_fill_width_mm": self.validate_non_negative_float(
                self._form_value(form, "min_fill_width_mm") if self._form_value(form, "min_fill_width_mm") is not None else derived_defaults["min_fill_width_mm"],
                "Minimum fill width",
                maximum=10,
            ),
            "simplify_tolerance_mm": self.validate_non_negative_float(form.get("simplify_tolerance_mm", config["DEFAULT_SIMPLIFY_TOLERANCE_MM"]), "Simplify tolerance", maximum=5),
            "remove_duplicate_paths": self.validate_bool(form.get("remove_duplicate_paths", config["DEFAULT_REMOVE_DUPLICATE_PATHS"])),
            "small_shape_mode": form.get("small_shape_mode", config["DEFAULT_SMALL_SHAPE_MODE"]),
            "thin_detail_mode": self.validate_bool(form.get("thin_detail_mode", config["DEFAULT_THIN_DETAIL_MODE"])),
            "thin_detail_min_area_mm2": self.validate_non_negative_float(
                form.get("thin_detail_min_area_mm2", config["DEFAULT_THIN_DETAIL_MIN_AREA_MM2"]),
                "Thin detail minimum area",
                maximum=10000,
            ),
            "thin_detail_simplify_mm": self.validate_non_negative_float(
                form.get("thin_detail_simplify_mm", config["DEFAULT_THIN_DETAIL_SIMPLIFY_MM"]),
                "Thin detail simplify",
                maximum=5,
            ),
            "thin_detail_overlap": self.validate_bool(form.get("thin_detail_overlap", config["DEFAULT_THIN_DETAIL_OVERLAP"])),
            "min_segment_length_mm": self.validate_non_negative_float(form.get("min_segment_length_mm", config["DEFAULT_MIN_SEGMENT_LENGTH_MM"]), "Minimum segment length", maximum=20),
            "travel_optimization": form.get("travel_optimization", config["DEFAULT_TRAVEL_OPTIMIZATION"]),
            "allow_pen_down_infill_connectors": self.validate_bool(form.get("allow_pen_down_infill_connectors", config["DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS"])),
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
            "gcode_mode": form.get("gcode_mode", config["DEFAULT_GCODE_MODE"]),
            "color_tolerance": self.validate_non_negative_int(form.get("color_tolerance", config["DEFAULT_RASTER_COLOR_TOLERANCE"]), "Color tolerance", minimum=0, maximum=255),
            "min_component_area_px": self.validate_non_negative_int(form.get("min_component_area_px", config["DEFAULT_RASTER_MIN_COMPONENT_AREA_PX"]), "Min component area", minimum=0, maximum=1000000),
            "mask_open_radius_px": self.validate_non_negative_int(form.get("mask_open_radius_px", config["DEFAULT_RASTER_MASK_OPEN_RADIUS_PX"]), "Mask open radius", minimum=0, maximum=50),
            "mask_close_radius_px": self.validate_non_negative_int(form.get("mask_close_radius_px", config["DEFAULT_RASTER_MASK_CLOSE_RADIUS_PX"]), "Mask close radius", minimum=0, maximum=50),
            "min_region_area_px": self.validate_non_negative_float(form.get("min_region_area_px", config["DEFAULT_RASTER_MIN_REGION_AREA_PX"]), "Min region area", maximum=1000000),
            "region_simplify_px": self.validate_non_negative_float(form.get("region_simplify_px", config["DEFAULT_RASTER_REGION_SIMPLIFY_PX"]), "Region simplify", maximum=50),
            "debug_pipeline": self.validate_bool(form.get("debug_pipeline", "0")),
        }
        options["effective_infill_spacing_mm"] = options["infill_spacing_mm"]
        options["selected_colors"] = RasterAnalysisService.parse_selected_colors(form.get("selected_colors"))
        if not options["selected_colors"]:
            raise ValueError("Select at least one color to print")
        if options["sample_step_deg"] <= 0:
            raise ValueError("Sample step must be greater than 0")
        if options["margin_percent"] < 0 or options["margin_percent"] > 25:
            raise ValueError("Margin percent must be between 0 and 25")
        if options["line_thickness_mm"] < 0 or options["line_thickness_mm"] > 10:
            raise ValueError("Line thickness must be between 0 and 10 mm")
        if options["fit_mode"] not in {"contain", "stretch"}:
            raise ValueError("Invalid fit mode")
        if options["infill_pattern"] not in {"zigzag", "hatch"}:
            raise ValueError("Invalid raster infill pattern")
        if options["fill_strategy"] not in {"horizontal_scanline", "rotated_scanline", "adaptive_angle", "crosshatch"}:
            raise ValueError("Invalid fill strategy")
        if options["infill_density"] <= 0 or options["infill_density"] > 100:
            raise ValueError("Infill density must be between 0 and 100")
        if options["small_shape_mode"] not in {"single-wall", "skip", "centerline"}:
            raise ValueError("Invalid small shape mode")
        if options["travel_optimization"] not in {"nearest-neighbor"}:
            raise ValueError("Invalid travel optimization")
        if options["gcode_mode"] not in {"simple"}:
            raise ValueError("Invalid G-code mode")
        return options
