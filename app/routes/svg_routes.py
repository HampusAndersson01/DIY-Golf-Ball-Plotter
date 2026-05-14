from dataclasses import asdict

from flask import Blueprint, current_app, request

from app.extensions import (
    get_gcode_service,
    get_geometry_service,
    get_self_test_service,
    get_state,
    get_svg_parser,
    get_toolpath_service,
    get_validation_service,
)
from app.utils.response_utils import json_error, json_ok

svg_bp = Blueprint("svg", __name__)


@svg_bp.post("/generate-gcode")
def generate_gcode_route():
    state = get_state()
    validation = get_validation_service()
    config = current_app.config
    try:
        file = request.files.get("svg")
        if file is None:
            raise ValueError("No SVG file uploaded")
        svg_text = file.read().decode("utf-8", errors="ignore")

        options = validation.parse_generate_gcode_form(request.form, config)
        debug_data = {} if options["debug_pipeline"] else None

        bundle, viewbox_bounds, print_model = get_svg_parser().extract_svg_bundle(
            svg_text,
            debug=debug_data,
            parser_mode=options["parser_mode"],
            color_mapping_mode=options["color_mapping_mode"],
            trace_stroke_only_paths=options["trace_stroke_only_paths"],
            fill_only_dark_svg_fills=options["fill_only_dark_svg_fills"],
        )
        if not bundle.outline_segments and not bundle.fill_boundary_segments and not bundle.fill_shapes:
            raise ValueError(
                "; ".join(print_model.diagnostics or ["Visible SVG content could not be normalized into drawable geometry."])
            )

        geometry = get_geometry_service()
        bounds = viewbox_bounds or geometry.bounds_from_bundle(bundle)
        mapped = geometry.map_bundle_to_angles(bundle, bounds, options["fit_mode"], options["invert_y"], options["margin_percent"])
        geometry.debug_append_bundle(debug_data, "mapped_paths", mapped)
        placed = geometry.apply_placement_transform(
            mapped,
            options["placement_scale"],
            options["rotation_deg"],
            options["placement_offset_x"],
            options["placement_offset_y"],
        )
        geometry.debug_append_bundle(debug_data, "placed_paths", placed)

        toolpaths = get_toolpath_service().generate_toolpaths(
            placed,
            enable_fill=options["enable_fill"],
            line_width_mm=options["line_thickness_mm"],
            wall_count=options["wall_count"],
            infill_density=options["infill_density"],
            infill_spacing_mm=options["infill_spacing_mm"] if options["infill_spacing_mm"] > 0 else options["line_thickness_mm"],
            infill_angle_deg=options["infill_angle_deg"],
            outline_after_fill=options["outline_after_fill"],
            min_fill_area_mm2=options["min_fill_area_mm2"],
            min_fill_width_mm=options["min_fill_width_mm"],
            simplify_tolerance_mm=options["simplify_tolerance_mm"],
            remove_duplicate_paths=options["remove_duplicate_paths"],
            small_shape_mode=options["small_shape_mode"],
            min_segment_length_mm=options["min_segment_length_mm"],
            travel_optimization=options["travel_optimization"],
            debug=debug_data,
        )
        if not toolpaths:
            raise ValueError("No toolpaths were generated from the current SVG/settings")

        gcode, preview = get_gcode_service().generate_from_toolpaths(toolpaths=toolpaths, **options)
        if debug_data is not None:
            debug_data["gcode_preview"] = preview

        point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
        state.update(
            last_svg_name=file.filename,
            last_gcode=gcode,
            last_preview=preview,
            progress_total=0,
            progress_done=0,
            status="G-code generated - calibrate before run",
            last_error=None,
        )

        return json_ok(
            gcode=gcode,
            preview=preview,
            toolpath_count=len(toolpaths),
            point_count=point_count,
            bounds=asdict(bounds),
            viewbox_bounds=asdict(viewbox_bounds) if viewbox_bounds else None,
            print_model=asdict(print_model),
            debug=debug_data,
        )
    except Exception as exc:
        state.update(last_error=str(exc), status=f"Generate error: {exc}")
        return json_error(str(exc), status=500)


@svg_bp.post("/analyze-svg")
def analyze_svg_route():
    try:
        file = request.files.get("svg")
        if file is None:
            raise ValueError("No SVG file uploaded")
        svg_text = file.read().decode("utf-8", errors="ignore")
        options = get_validation_service().parse_analyze_svg_form(request.form, current_app.config)
        debug_data = {} if options["debug_pipeline"] else None
        result = get_svg_parser().analyze_svg(svg_text, debug=debug_data, **options)
        return json_ok(
            print_model=asdict(result.print_model),
            viewbox_bounds=asdict(result.viewbox_bounds) if result.viewbox_bounds else None,
            debug=debug_data,
        )
    except Exception as exc:
        return json_error(str(exc), status=500)


@svg_bp.post("/self-test-svg-pipeline")
def self_test_svg_pipeline_route():
    try:
        summary = get_self_test_service().run()
        return json_ok(summary=summary)
    except Exception as exc:
        return json_error(str(exc), status=500)
