from __future__ import annotations

from dataclasses import asdict

from flask import Blueprint, current_app, jsonify, request

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


def build_generate_debug_payload(*, selected_colors=None, mask_pixel_count=0):
    return {
        "received_files": sorted(list(request.files.keys())),
        "received_form_keys": sorted(list(request.form.keys())),
        "selected_colors": list(selected_colors or []),
        "mask_pixel_count": int(mask_pixel_count or 0),
    }


def estimate_runtime_seconds(preview: list[dict], *, draw_feed: float, travel_feed: float) -> float:
    total_minutes = 0.0
    draw_feed = max(float(draw_feed or 0.0), 1e-6)
    travel_feed = max(float(travel_feed or 0.0), 1e-6)
    for entry in preview or []:
        points = entry.get("points") or []
        if len(points) < 2:
            continue
        feed = travel_feed if entry.get("kind") == "travel" else draw_feed
        path_distance = 0.0
        for index in range(1, len(points)):
            dx = float(points[index]["x"]) - float(points[index - 1]["x"])
            dy = float(points[index]["y"]) - float(points[index - 1]["y"])
            path_distance += (dx * dx + dy * dy) ** 0.5
        total_minutes += path_distance / feed
    return max(0.0, total_minutes * 60.0)


@raster_bp.post("/analyze-image")
@raster_bp.post("/analyze-image-colors")
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
            min_region_area_px=0.0 if options["thin_detail_mode"] else options["min_region_area_px"],
            simplify_tolerance_px=options["region_simplify_px"],
        )
        has_area_geometry = region_result.bundle.printable_geometry is not None and not region_result.bundle.printable_geometry.is_empty
        has_detail_geometry = bool(region_result.bundle.detail_segments)
        if not has_area_geometry and not has_detail_geometry:
            raise ValueError("No printable regions were found for the selected colors")

        geometry = get_geometry_service()
        mapped = geometry.map_bundle_to_surface_mm(
            region_result.bundle,
            region_result.bounds,
            options["fit_mode"],
            options["invert_y"],
            options["margin_percent"],
        )
        geometry.debug_append_bundle(debug_data, "mapped_paths", mapped)
        placed = geometry.apply_surface_placement_transform(
            mapped,
            options["placement_scale"],
            options["rotation_deg"],
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
            thin_detail_mode=options["thin_detail_mode"],
            thin_detail_min_area_mm2=options["thin_detail_min_area_mm2"],
            thin_detail_simplify_mm=options["thin_detail_simplify_mm"],
            thin_detail_overlap=options["thin_detail_overlap"],
            min_segment_length_mm=options["min_segment_length_mm"],
            travel_optimization=options["travel_optimization"],
            allow_pen_down_infill_connectors=options["allow_pen_down_infill_connectors"],
            debug=debug_data,
        )
        if not toolpaths:
            raise ValueError("No toolpaths were generated from the selected image regions")

        toolpath_diagnostics = get_toolpath_service().summarize_toolpaths(toolpaths)
        gcode, preview = get_gcode_service().generate_from_toolpaths(toolpaths=toolpaths, **options)
        if debug_data is not None:
            debug_data["gcode_preview"] = preview

        stage_counts = {
            "selected_mask_pixel_count": mask_result.printable_pixel_count,
            "connected_component_count": mask_result.connected_component_count,
            "selected_component_count": region_result.selected_component_count,
            "detail_trace_component_count": region_result.detail_trace_component_count,
            "detail_trace_path_count": region_result.detail_trace_path_count,
            "skeleton_pixel_count": region_result.skeleton_pixel_count,
            "contour_count": region_result.contour_count,
            "polygon_count": region_result.polygon_count,
            "final_toolpath_count": len(toolpaths),
            "one_move_toolpaths": toolpath_diagnostics["one_move_toolpaths"],
            "generated_fill_walls": sum(1 for path in toolpaths if path.kind == "fill-wall"),
            "generated_infill_paths": sum(1 for path in toolpaths if path.kind == "fill-infill"),
            "generated_thin_detail_paths": sum(1 for path in toolpaths if path.kind == "detail-trace"),
            "generated_detail_trace_paths": sum(1 for path in toolpaths if path.kind == "detail-trace"),
            "generated_outline_paths": sum(1 for path in toolpaths if path.kind == "outline"),
            "final_toolpaths_by_kind": {
                "fill-wall": sum(1 for path in toolpaths if path.kind == "fill-wall"),
                "fill-infill": sum(1 for path in toolpaths if path.kind == "fill-infill"),
                "detail-trace": sum(1 for path in toolpaths if path.kind == "detail-trace"),
                "outline": sum(1 for path in toolpaths if path.kind == "outline"),
                "travel": sum(1 for path in toolpaths if path.kind == "travel"),
            },
        }
        if debug_data is not None:
            stage_counts.update(debug_data.get("slicer_counts", {}))
            stage_counts.update(debug_data.get("toolpath_counts", {}))

        point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
        estimated_runtime_seconds = estimate_runtime_seconds(
            preview,
            draw_feed=options["draw_feed"],
            travel_feed=options["travel_feed"],
        )
        summary = {
            "image_size": f"{mask_result.width}x{mask_result.height}",
            "selected_colors": options["selected_colors"],
            "mask_pixel_count": mask_result.printable_pixel_count,
            "component_count": mask_result.connected_component_count,
            "toolpath_counts": stage_counts["final_toolpaths_by_kind"],
            "wall_path_count": stage_counts["generated_fill_walls"],
            "infill_path_count": stage_counts["generated_infill_paths"],
            "detail_trace_path_count": stage_counts["generated_detail_trace_paths"],
            "travel_path_count": stage_counts["final_toolpaths_by_kind"]["travel"],
            "gcode_line_count": len(gcode),
            "point_count": point_count,
            "estimated_runtime_seconds": estimated_runtime_seconds,
            "pen_lift_count": len(toolpaths),
        }
        state.update(
            last_svg_name=file.filename,
            last_gcode=gcode,
            last_preview=preview,
            last_summary=summary,
            progress_total=0,
            progress_done=0,
            current_gcode_line=0,
            current_path_id=None,
            current_preview_point_index=0,
            status="Raster G-code generated - calibrate before run",
            last_error=None,
        )

        return json_ok(
            gcode=gcode,
            preview=preview,
            toolpath_count=len(toolpaths),
            toolpath_diagnostics=toolpath_diagnostics,
            point_count=point_count,
            mask_pixel_count=mask_result.printable_pixel_count,
            component_count=mask_result.connected_component_count,
            source_bounds=asdict(region_result.bounds),
            mask=raster.serialize_mask(mask_result),
            mask_preview=mask_result.mask_preview_url,
            regions=raster.serialize_regions(region_result),
            selected_colors=options["selected_colors"],
            summary=summary,
            stage_counts=stage_counts,
            debug=debug_data,
        )
    except Exception as exc:
        state.update(last_error=str(exc), status=f"Generate error: {exc}")
        selected_colors = []
        try:
            selected_colors = validation.parse_generate_raster_form(request.form, config).get("selected_colors", [])
        except Exception:
            selected_colors = []
        return jsonify({
            "ok": False,
            "error": str(exc),
            "debug": build_generate_debug_payload(selected_colors=selected_colors),
        }), 500
