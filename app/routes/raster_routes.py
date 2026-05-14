from __future__ import annotations

from dataclasses import asdict

from flask import Blueprint, current_app, request

from app.extensions import (
    get_gcode_service,
    get_geometry_service,
    get_raster_analysis_service,
    get_state,
    get_toolpath_service,
    get_validation_service,
)
from app.utils.response_utils import json_error, json_ok

raster_bp = Blueprint("raster", __name__)


@raster_bp.post("/analyze-image")
def analyze_image_route():
    try:
        file = request.files.get("image")
        if file is None:
            raise ValueError("No PNG or JPG image uploaded")

        options = get_validation_service().parse_analyze_raster_form(request.form, current_app.config)
        result = get_raster_analysis_service().analyze_image(
            file.read(),
            simplify_colors=options["simplify_colors"],
            max_colors=options["max_colors"],
        )
        return json_ok(analysis=get_raster_analysis_service().serialize_analysis(result))
    except Exception as exc:
        return json_error(str(exc), status=500)


@raster_bp.post("/generate-image-gcode")
def generate_image_gcode_route():
    state = get_state()
    validation = get_validation_service()
    config = current_app.config
    try:
        file = request.files.get("image")
        if file is None:
            raise ValueError("No PNG or JPG image uploaded")

        image_bytes = file.read()
        options = validation.parse_generate_raster_form(request.form, config)
        debug_data = {} if options["debug_pipeline"] else None
        raster = get_raster_analysis_service()
        mask_result = raster.build_mask(
            image_bytes,
            options["selected_colors"],
            tolerance=options["color_tolerance"],
            min_component_area_px=options["min_component_area_px"],
            open_radius_px=options["mask_open_radius_px"],
            close_radius_px=options["mask_close_radius_px"],
        )
        region_result = raster.extract_regions(
            mask_result,
            min_region_area_px=options["min_region_area_px"],
            simplify_tolerance_px=options["region_simplify_px"],
        )
        if region_result.bundle.printable_geometry is None or region_result.bundle.printable_geometry.is_empty:
            raise ValueError("No printable regions were found for the selected colors")

        geometry = get_geometry_service()
        mapped = geometry.map_bundle_to_angles(
            region_result.bundle,
            region_result.bounds,
            options["fit_mode"],
            options["invert_y"],
            options["margin_percent"],
        )
        geometry.debug_append_bundle(debug_data, "mapped_paths", mapped)
        placed = geometry.apply_placement_transform(
            mapped,
            options["placement_scale"],
            options["rotation_deg"],
            options["placement_offset_x"],
            options["placement_offset_y"],
        )
        geometry.debug_append_bundle(debug_data, "placed_paths", placed)

        toolpaths = get_toolpath_service().generate_from_regions(
            placed,
            pen_width_mm=options["line_thickness_mm"],
            wall_count=options["wall_count"],
            infill_pattern=options["infill_pattern"],
            infill_spacing_mm=options["infill_spacing_mm"] if options["infill_spacing_mm"] > 0 else options["line_thickness_mm"],
            infill_density=options["infill_density"],
            infill_angle_deg=options["infill_angle_deg"],
            outline_after_fill=options["outline_after_fill"],
            min_region_area=options["min_fill_area_mm2"],
            min_fill_width_mm=options["min_fill_width_mm"],
            simplify_tolerance_mm=options["simplify_tolerance_mm"],
            remove_duplicate_paths=options["remove_duplicate_paths"],
            small_shape_mode=options["small_shape_mode"],
            min_segment_length_mm=options["min_segment_length_mm"],
            travel_optimization=options["travel_optimization"],
            debug=debug_data,
        )
        if not toolpaths:
            raise ValueError("No toolpaths were generated from the selected image regions")

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
            status="Raster G-code generated - calibrate before run",
            last_error=None,
        )

        return json_ok(
            gcode=gcode,
            preview=preview,
            toolpath_count=len(toolpaths),
            point_count=point_count,
            source_bounds=asdict(region_result.bounds),
            mask=raster.serialize_mask(mask_result),
            regions=raster.serialize_regions(region_result),
            selected_colors=options["selected_colors"],
            debug=debug_data,
        )
    except Exception as exc:
        state.update(last_error=str(exc), status=f"Generate error: {exc}")
        return json_error(str(exc), status=500)
