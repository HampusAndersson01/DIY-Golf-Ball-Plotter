from __future__ import annotations

from dataclasses import asdict
import math
import time

from flask import Blueprint, current_app, jsonify, request

from app.extensions import (
    get_gcode_service,
    get_geometry_service,
    get_raster_analysis_service,
    get_state,
    get_toolpath_service,
    get_validation_service,
)
from app.services import pipeline_core
from app.services.runtime_estimation_service import estimate_gcode_runtime
from app.utils.response_utils import json_error, json_ok, log_exception

raster_bp = Blueprint("raster", __name__)


def build_generate_debug_payload(*, selected_colors=None, mask_pixel_count=0):
    return {
        "received_files": sorted(list(request.files.keys())),
        "received_form_keys": sorted(list(request.form.keys())),
        "selected_colors": list(selected_colors or []),
        "mask_pixel_count": int(mask_pixel_count or 0),
    }


def build_setting_debug(error: Exception, config) -> dict | None:
    message = str(error)
    if "line_thickness_mm" not in message and "Line thickness" not in message:
        return None
    return {
        "missing_path": "pen.line_thickness_mm",
        "used_default": True,
        "default_value": config["DEFAULT_LINE_THICKNESS_MM"],
    }


def build_fill_header_settings(options: dict, design_bounds) -> dict:
    return {
        "artworkScalePercent": f'{options["artwork_scale_percent"]:.4f}',
        "originAnchor": options["origin_anchor"],
        "originOffsetXmm": f'{options["origin_offset_x_mm"]:.4f}',
        "originOffsetYmm": f'{options["origin_offset_y_mm"]:.4f}',
        "lineWidthMm": f'{options["line_thickness_mm"]:.4f}',
        "infillSpacingMm": f'{options["effective_infill_spacing_mm"]:.4f}',
        "wallCount": options["wall_count"],
        "infillAngle": f'{options["infill_angle_deg"]:.4f}',
        "rotationDeg": f'{options["rotation_deg"]:.4f}',
        "designWidthMm": f"{design_bounds.width:.4f}",
        "designHeightMm": f"{design_bounds.height:.4f}",
        "coordinateSpaceUsedForFill": "surface-mm-on-ball",
    }


def estimate_runtime_seconds(gcode: list[str], *, draw_feed: float, travel_feed: float, pen_up_s: int, pen_down_s: int) -> dict[str, object]:
    return estimate_gcode_runtime(
        gcode,
        draw_feed=draw_feed,
        travel_feed=travel_feed,
        pen_up_s=pen_up_s,
        pen_down_s=pen_down_s,
    ).as_dict()


def build_effective_settings(options: dict) -> dict:
    return {
        "artwork_scale_percent": options["artwork_scale_percent"],
        "line_thickness_mm": options["line_thickness_mm"],
        "infill_spacing_mm": options["effective_infill_spacing_mm"],
        "custom_infill_spacing": options["custom_infill_spacing"],
        "wall_count": options["wall_count"],
        "fill_density": options["infill_density"],
    }


def build_mask_projection_quad(
    *,
    options: dict,
    bounds,
    resolved_center_lat_deg: float,
    auto_y_band_fit: dict[str, object] | None = None,
    artwork_scale_center_x_mm: float | None = None,
    artwork_scale_center_y_mm: float | None = None,
    origin_translation_x_mm: float | None = None,
    origin_translation_y_mm: float | None = None,
) -> dict[str, dict[str, float]]:
    corner_segment = pipeline_core.Segment(
        points=[
            pipeline_core.Point(float(bounds.min_x), float(bounds.min_y)),
            pipeline_core.Point(float(bounds.max_x), float(bounds.min_y)),
            pipeline_core.Point(float(bounds.max_x), float(bounds.max_y)),
            pipeline_core.Point(float(bounds.min_x), float(bounds.max_y)),
            pipeline_core.Point(float(bounds.min_x), float(bounds.min_y)),
        ],
        closed=True,
    )
    frame_bundle = pipeline_core.GeometryBundle(outline_segments=[corner_segment])
    geometry = get_geometry_service()
    mapped = geometry.map_bundle_to_surface_mm(
        frame_bundle,
        bounds,
        options["fit_mode"],
        options["invert_y"],
        options["margin_percent"],
    )
    if not mapped.outline_segments or len(mapped.outline_segments[0].points) < 4:
        return {}

    corners_surface = list(mapped.outline_segments[0].points[:4])
    scale_factor = float(options["artwork_scale_percent"]) / 100.0
    if artwork_scale_center_x_mm is not None and artwork_scale_center_y_mm is not None and abs(scale_factor - 1.0) > 1e-12:
        scaled_corners = []
        for point in corners_surface:
            scaled_corners.append(
                pipeline_core.Point(
                    float(artwork_scale_center_x_mm) + ((point.x - float(artwork_scale_center_x_mm)) * scale_factor),
                    float(artwork_scale_center_y_mm) + ((point.y - float(artwork_scale_center_y_mm)) * scale_factor),
                )
            )
        corners_surface = scaled_corners

    placement_scale = float(options["placement_scale"]) / 100.0
    rotation_rad = math.radians(float(options["rotation_deg"]))
    cos_a = math.cos(rotation_rad)
    sin_a = math.sin(rotation_rad)
    transformed_corners = []
    for point in corners_surface:
        sx = point.x * placement_scale
        sy = point.y * placement_scale
        transformed_corners.append(
            pipeline_core.Point(
                (sx * cos_a) - (sy * sin_a),
                (sx * sin_a) + (sy * cos_a),
            )
        )
    corners_surface = transformed_corners

    if not corners_surface:
        return {}

    dx = float(origin_translation_x_mm) if origin_translation_x_mm is not None else 0.0
    dy = float(origin_translation_y_mm) if origin_translation_y_mm is not None else 0.0
    corners_surface = [pipeline_core.Point(point.x + dx, point.y + dy) for point in corners_surface]

    if auto_y_band_fit and bool(auto_y_band_fit.get("auto_scaled", False)):
        y_band_scale_factor = float(auto_y_band_fit.get("scale_factor", 1.0))
        origin_x = float(auto_y_band_fit.get("origin_x_mm", 0.0))
        origin_y = float(auto_y_band_fit.get("origin_y_mm", 0.0))
        scaled_corners = []
        for point in corners_surface:
            scaled_corners.append(
                pipeline_core.Point(
                    origin_x + ((point.x - origin_x) * y_band_scale_factor),
                    origin_y + ((point.y - origin_y) * y_band_scale_factor),
                )
            )
        corners_surface = scaled_corners

    projected = [
        pipeline_core.surface_mm_to_ball_angles(
            point,
            center_lon_deg=options["placement_offset_x"],
            center_lat_deg=resolved_center_lat_deg,
            ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
        )
        for point in corners_surface
    ]
    return {
        "top_left": {"x": projected[0].x, "y": projected[0].y},
        "top_right": {"x": projected[1].x, "y": projected[1].y},
        "bottom_right": {"x": projected[2].x, "y": projected[2].y},
        "bottom_left": {"x": projected[3].x, "y": projected[3].y},
    }


def build_projected_mask_preview_paths(
    *,
    placed_bundle,
    center_lon_deg: float,
    center_lat_deg: float,
    ball_diameter_mm: float,
    pen_width_mm: float,
) -> list[dict[str, object]]:
    geometry = placed_bundle.printable_geometry
    if geometry is None or geometry.is_empty:
        return []
    surface_paths: list[pipeline_core.Toolpath] = []
    for polygon in pipeline_core.normalize_geometry(geometry):
        exterior_points = [pipeline_core.Point(float(x), float(y)) for x, y in polygon.exterior.coords]
        if len(exterior_points) >= 2:
            surface_paths.append(
                pipeline_core.Toolpath(
                    points=exterior_points,
                    kind="mask-overlay",
                    closed=True,
                    coordinate_space="surface_mm",
                    source="mask_projection",
                )
            )
        for ring in polygon.interiors:
            hole_points = [pipeline_core.Point(float(x), float(y)) for x, y in ring.coords]
            if len(hole_points) >= 2:
                surface_paths.append(
                    pipeline_core.Toolpath(
                        points=hole_points,
                        kind="mask-overlay-hole",
                        closed=True,
                        coordinate_space="surface_mm",
                        source="mask_projection",
                    )
                )
    if not surface_paths:
        return []
    prepared = pipeline_core.prepare_toolpaths_for_projection(surface_paths, default_pen_width_mm=max(0.01, float(pen_width_mm)))
    projected = pipeline_core.project_toolpaths_to_ball_angles(
        prepared,
        center_lon_deg=float(center_lon_deg),
        center_lat_deg=float(center_lat_deg),
        ball_diameter_mm=float(ball_diameter_mm),
    )
    return [
        {
            "id": f"mask_overlay_{index:03d}",
            "kind": path.kind,
            "closed": bool(path.closed),
            "points": [{"x": float(point.x), "y": float(point.y)} for point in path.points],
            "gcode_start_line": None,
            "gcode_end_line": None,
            "pen_down": False,
            "source": "mask_projection",
        }
        for index, path in enumerate(projected)
        if len(path.points) >= 2
    ]


def project_surface_toolpaths(toolpaths, options: dict):
    cleaned_toolpaths, cleanup_stats = pipeline_core.cleanup_surface_toolpaths(
        toolpaths,
        tolerance_mm=options["simplify_tolerance_mm"],
        min_segment_length_mm=options["min_segment_length_mm"],
    )
    cleaned_toolpaths = pipeline_core.assign_stable_path_ids(cleaned_toolpaths)
    pipeline_core.validate_toolpaths_finite(cleaned_toolpaths, coordinate_space="surface_mm")
    cleaned_toolpaths = pipeline_core.prepare_toolpaths_for_projection(
        cleaned_toolpaths,
        default_pen_width_mm=options["line_thickness_mm"],
    )
    y_band_scale_debug: dict[str, object] = {}
    try:
        _resolved_lat, projection_center_debug = pipeline_core.resolve_safe_projection_center_lat(
            cleaned_toolpaths,
            requested_center_lat_deg=options["placement_offset_y"],
            ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
            y_draw_min_deg=current_app.config["Y_DRAW_MIN"],
            y_draw_max_deg=current_app.config["Y_DRAW_MAX"],
        )
    except ValueError:
        cleaned_toolpaths, y_band_scale_debug = pipeline_core.fit_surface_toolpaths_to_y_band(
            cleaned_toolpaths,
            ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
            y_draw_min_deg=current_app.config["Y_DRAW_MIN"],
            y_draw_max_deg=current_app.config["Y_DRAW_MAX"],
        )
        if bool(y_band_scale_debug.get("auto_scaled", False)):
            current_app.logger.warning(
                "Auto-scaled artwork by %.4f to fit Y drawing band (span %.4fmm -> %.4fmm)",
                float(y_band_scale_debug.get("scale_factor", 1.0)),
                float(y_band_scale_debug.get("current_span_mm", 0.0)),
                float(y_band_scale_debug.get("allowed_span_mm", 0.0)),
            )
    resolved_center_lat_deg, projection_center_debug = pipeline_core.resolve_safe_projection_center_lat(
        cleaned_toolpaths,
        requested_center_lat_deg=options["placement_offset_y"],
        ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
        y_draw_min_deg=current_app.config["Y_DRAW_MIN"],
        y_draw_max_deg=current_app.config["Y_DRAW_MAX"],
    )
    if bool(projection_center_debug.get("auto_clamped", False)):
        current_app.logger.warning(
            "Auto-clamped projection center latitude from %.4f to %.4f to keep projected toolpaths inside Y draw limits",
            float(projection_center_debug["requested_center_lat_deg"]),
            float(projection_center_debug["resolved_center_lat_deg"]),
        )
    projected_toolpaths = pipeline_core.project_toolpaths_to_ball_angles(
        cleaned_toolpaths,
        center_lon_deg=options["placement_offset_x"],
        center_lat_deg=resolved_center_lat_deg,
        ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
    )
    pipeline_core.validate_toolpaths_finite(projected_toolpaths, coordinate_space="machine_deg")
    lifecycle_logs, outline_pipeline_debug = pipeline_core.build_toolpath_lifecycle_debug(cleaned_toolpaths, projected_toolpaths)
    pipeline_core.log_physical_outline_mismatch_check(cleaned_toolpaths, projected_toolpaths)
    coordinate_debug = {
        "unit_model": "surface_mm_then_project_once_to_machine_deg",
        "auto_y_band_fit": y_band_scale_debug,
        "projection_center_latitude": projection_center_debug,
        "toolpath_kinds": lifecycle_logs,
        "projection_applied_to": {
            kind: True for kind in ("outline", "fill-wall", "fill-infill", "detail-trace")
        },
        "projection_count_by_kind": {
            kind: 1 for kind in ("outline", "fill-wall", "fill-infill", "detail-trace")
        },
    }
    return (
        cleaned_toolpaths,
        projected_toolpaths,
        cleanup_stats,
        coordinate_debug,
        outline_pipeline_debug,
        pipeline_core.build_region_alignment_debug(cleaned_toolpaths, projected_toolpaths),
        pipeline_core.build_sampling_debug(cleaned_toolpaths, projected_toolpaths),
        pipeline_core.build_outline_vs_infill_alignment_audit(cleaned_toolpaths, projected_toolpaths),
    )


def build_gcode_stats(gcode: list[str], cleanup_stats: dict[str, object], *, preview_path_count: int = 0, debug: dict | None = None) -> dict[str, object]:
    comment_lines = sum(1 for line in gcode if line.strip().startswith("(") and line.strip().endswith(")"))
    blank_lines = sum(1 for line in gcode if not line.strip())
    motion_lines = sum(1 for line in gcode if line.strip().startswith("G1"))
    dwell_count = sum(1 for line in gcode if line.strip().startswith("G4"))
    pen_up_count = sum(1 for line in gcode if line.strip().startswith("M3 S") and "S575" in line)
    pen_down_count = sum(1 for line in gcode if line.strip().startswith("M3 S") and "S700" in line)
    estimated_bytes_after = sum(len((line + "\n").encode("ascii", errors="ignore")) for line in gcode)
    estimated_bytes_before = estimated_bytes_after
    for line in gcode:
        stripped = line.strip()
        if stripped.startswith("G1 ") and " F" not in stripped:
            estimated_bytes_before += len(" F1200.000")
    return {
        "total_lines": len(gcode),
        "preview_path_count": preview_path_count,
        "motion_lines": motion_lines,
        "comment_lines": comment_lines,
        "blank_lines": blank_lines,
        "dwell_count": dwell_count,
        "pen_up_count": pen_up_count,
        "pen_down_count": pen_down_count,
        "modal_feedrate_optimized": True,
        "duplicate_points_removed": cleanup_stats["duplicate_points_removed"],
        "short_segments_removed": cleanup_stats["short_segments_removed"],
        "simplification_tolerance_mm": cleanup_stats["simplification_tolerance_mm"],
        "estimated_serial_bytes_before": estimated_bytes_before,
        "estimated_serial_bytes_after": estimated_bytes_after,
        "pen_state_summary": (debug or {}).get("pen_state_summary"),
    }


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
        log_exception("Analyze image failed", exc)
        return json_error(str(exc), status=500)


@raster_bp.post("/generate-image-gcode")
def generate_image_gcode_route():
    state = get_state()
    validation = get_validation_service()
    config = current_app.config
    try:
        stage_t0 = time.perf_counter()
        stage_marks: dict[str, float] = {}

        def _mark(stage_name: str) -> None:
            stage_marks[stage_name] = time.perf_counter() - stage_t0

        def _dur(start_stage: str, end_stage: str) -> float:
            return max(0.0, float(stage_marks.get(end_stage, 0.0) - stage_marks.get(start_stage, 0.0)))

        current_app.logger.info(
            "Raster G-code generation requested: files=%s form_keys=%s",
            sorted(list(request.files.keys())),
            sorted(list(request.form.keys())),
        )
        file = request.files.get("image")
        if file is None:
            raise ValueError("No PNG or JPG image uploaded")

        image_bytes = file.read()
        _mark("input_loading")
        options = validation.parse_generate_raster_form(request.form, config)
        debug_data = {} if options["debug_pipeline"] else None
        raster = get_raster_analysis_service()
        mask_result = raster.build_mask(
            image_bytes,
            options["selected_colors"],
            simplify_colors=options["simplify_colors"],
            max_colors=options["max_colors"],
            tolerance=options["color_tolerance"],
            min_component_area_px=options["min_component_area_px"],
            open_radius_px=options["mask_open_radius_px"],
            close_radius_px=options["mask_close_radius_px"],
        )
        _mark("mask_extraction")
        region_result = raster.extract_regions(
            mask_result,
            min_region_area_px=0.0 if options["thin_detail_mode"] else options["min_region_area_px"],
            simplify_tolerance_px=options["region_simplify_px"],
        )
        _mark("vector_extraction")
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
        mapped_selected_bounds = geometry.bounds_from_bundle(mapped)
        geometry.debug_append_bundle(debug_data, "mapped_paths", mapped)
        artwork_scaled = geometry.apply_surface_artwork_scale(
            mapped,
            options["artwork_scale_percent"],
        )
        geometry.debug_append_bundle(debug_data, "artwork_scaled_paths", artwork_scaled)
        transformed = geometry.apply_surface_placement_transform(
            artwork_scaled,
            options["placement_scale"],
            options["rotation_deg"],
        )
        geometry.debug_append_bundle(debug_data, "transformed_paths", transformed)
        placed = geometry.apply_origin_anchor_placement(
            transformed,
            origin_anchor=options["origin_anchor"],
            origin_offset_x_mm=options["origin_offset_x_mm"],
            origin_offset_y_mm=options["origin_offset_y_mm"],
        )
        origin_translation = placed.metadata.get("origin_translation_mm") if isinstance(placed.metadata, dict) else None
        geometry.debug_append_bundle(debug_data, "placed_paths", placed)
        x_span_debug = pipeline_core.validate_bundle_x_span(
            placed,
            max_x_span_deg=current_app.config["DEFAULT_MAX_PRINT_X_SPAN_DEG"],
            ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
            allow_overflow=options["ignore_printable_x_span_limit"],
        )
        if x_span_debug.get("limit_overridden"):
            current_app.logger.warning(
                "Printable X-span limit override enabled: width_deg=%.2f limit_deg=%.2f",
                float(x_span_debug["width_deg"]),
                float(x_span_debug["max_width_deg"]),
            )
        _mark("polygon_cleanup")
        effective_settings = build_effective_settings(options)
        design_bounds = geometry.bounds_from_bundle(placed)
        final_polygon_count = len(pipeline_core.normalize_geometry(placed.printable_geometry)) if placed.printable_geometry is not None and not placed.printable_geometry.is_empty else 0
        current_app.logger.debug(
            "Received fill settings: line_width_mm=%.4f infill_spacing_mm=%.4f custom_infill_spacing=%s min_fill_width_mm=%.4f min_fill_area_mm2=%.4f min_segment_length_mm=%.4f wall_count=%d infill_angle_deg=%.4f rotation_deg=%.4f",
            options["line_thickness_mm"],
            options["effective_infill_spacing_mm"],
            options["custom_infill_spacing"],
            options["min_fill_width_mm"],
            options["min_fill_area_mm2"],
            options["min_segment_length_mm"],
            options["wall_count"],
            options["infill_angle_deg"],
            options["rotation_deg"],
        )
        current_app.logger.debug(
            "Final design bounds in mm: min_x=%.4f min_y=%.4f max_x=%.4f max_y=%.4f width=%.4f height=%.4f filled_polygons=%d",
            design_bounds.min_x,
            design_bounds.min_y,
            design_bounds.max_x,
            design_bounds.max_y,
            design_bounds.width,
            design_bounds.height,
            final_polygon_count,
        )

        toolpaths = get_toolpath_service().generate_from_regions(
            placed,
            pen_width_mm=options["line_thickness_mm"],
            wall_count=options["wall_count"],
            infill_pattern=options["infill_pattern"],
            infill_spacing_mm=options["effective_infill_spacing_mm"],
            infill_density=options["infill_density"],
            infill_angle_deg=options["infill_angle_deg"],
            fill_strategy=options["fill_strategy"],
            alternate_fill_angle_deg=options["alternate_fill_angle_deg"],
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
            infill_path_mode=options["infill_path_mode"],
            expensive_coverage_repair=False,
            debug=debug_data,
        )
        _mark("fill_path_generation")
        if not toolpaths:
            raise ValueError("No toolpaths were generated from the selected image regions")

        toolpath_diagnostics = get_toolpath_service().summarize_toolpaths(toolpaths)
        current_app.logger.debug(
            "Generated toolpath counts: wall_paths=%d infill_paths=%d infill_segments=%d",
            sum(1 for path in toolpaths if path.kind == "fill-wall"),
            sum(1 for path in toolpaths if path.kind == "fill-infill"),
            sum(max(0, len(path.points) - 1) for path in toolpaths if path.kind == "fill-infill"),
        )
        cleaned_toolpaths, projected_toolpaths, cleanup_stats, coordinate_debug, outline_pipeline_debug, region_alignment_debug, sampling_debug, outline_vs_infill_alignment_debug = project_surface_toolpaths(toolpaths, options)
        _mark("path_ordering")
        gcode, preview = get_gcode_service().generate_from_toolpaths(
            toolpaths=projected_toolpaths,
            header_comment_settings=build_fill_header_settings(options, design_bounds),
            draw_feed=options["draw_feed"],
            travel_feed=options["travel_feed"],
            sample_step_deg=options["sample_step_deg"],
            placement_offset_x=options["placement_offset_x"],
            placement_offset_y=options["placement_offset_y"],
            pen_up_s=options["pen_up_s"],
            pen_down_s=options["pen_down_s"],
            servo_ramp_enabled=options["servo_ramp_enabled"],
            servo_ramp_step=options["servo_ramp_step"],
            servo_ramp_delay_ms=options["servo_ramp_delay_ms"],
            pen_up_dwell_ms=options["pen_up_dwell_ms"],
            pen_down_dwell_ms=options["pen_down_dwell_ms"],
            gcode_mode=options["gcode_mode"],
            include_comments=options["include_comments"],
            debug=debug_data,
        )
        _mark("gcode_writing")
        machine_motion_debug = pipeline_core.build_machine_motion_debug(
            cleaned_toolpaths,
            projected_toolpaths,
            preview,
            gcode,
            pen_up_s=options["pen_up_s"],
            pen_down_s=options["pen_down_s"],
        )
        if debug_data is not None:
            pipeline_core.debug_append_toolpaths(debug_data, "surface_mm_toolpaths", cleaned_toolpaths)
            pipeline_core.debug_append_toolpaths(debug_data, "projected_machine_deg_toolpaths", projected_toolpaths)
            projected_path_debug = pipeline_core.build_projected_path_debug(cleaned_toolpaths, projected_toolpaths, preview)
            outline_fill_alignment_debug = dict((debug_data.get("outline_fill_alignment_debug") or {}))
            outline_fill_alignment_debug.update({
                "outline_projected_once": projected_path_debug["projection_count_by_kind"].get("outline", 0) == 1,
                "infill_projected_once": projected_path_debug["projection_count_by_kind"].get("fill-infill", 0) == 1,
                "preview_and_gcode_same_paths": projected_path_debug["preview_and_gcode_share_same_projected_paths"],
            })
            debug_data["gcode_preview"] = preview
            debug_data["coordinate_debug"] = coordinate_debug
            debug_data["printable_x_span_debug"] = x_span_debug
            debug_data["outline_pipeline_debug"] = {
                **outline_pipeline_debug,
                **projected_path_debug,
            }
            debug_data["outline_fill_alignment_debug"] = outline_fill_alignment_debug
            debug_data["region_alignment_debug"] = region_alignment_debug
            debug_data["sampling_debug"] = sampling_debug
            debug_data["machine_motion_debug"] = machine_motion_debug
            debug_data["outline_vs_infill_alignment"] = outline_vs_infill_alignment_debug
            debug_data["visual_debug_layers"] = [
                {"key": "detected_printable_polygons", "label": "final printable polygon", "color": "gray"},
                {"key": "residual_detail_target", "label": "residual area needing detail", "color": "red"},
                {"key": "outer_walls", "label": "outline centerline before projection", "color": "orange-dashed"},
                {"key": "clipped_infill_lines", "label": "infill centerlines", "color": "cyan"},
                {"key": "detail_traces_accepted", "label": "accepted detail traces", "color": "magenta"},
                {"key": "detail_traces_rejected", "label": "rejected detail traces", "color": "red-dashed"},
                {"key": "projected_machine_deg_toolpaths", "label": "outline centerline after projection", "color": "orange-solid"},
                {"key": "gcode_preview", "label": "reconstructed G-code path", "color": "purple"},
            ]
            outline_pipeline_debug = debug_data["outline_pipeline_debug"]

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

        gcode_stats = build_gcode_stats(gcode, cleanup_stats, preview_path_count=len(preview), debug=debug_data)
        current_app.logger.info(
            "Raster G-code generated: file=%s toolpaths=%d preview_paths=%d gcode_lines=%d selected_colors=%d",
            file.filename,
            len(toolpaths),
            len(preview),
            len(gcode),
            len(options["selected_colors"]),
        )
        point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
        runtime_estimate = estimate_runtime_seconds(
            gcode,
            draw_feed=options["draw_feed"],
            travel_feed=options["travel_feed"],
            pen_up_s=options["pen_up_s"],
            pen_down_s=options["pen_down_s"],
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
            "gcode_line_count": runtime_estimate["rawGcodeLines"],
            "streamable_gcode_line_count": runtime_estimate["streamableGcodeLines"],
            "point_count": point_count,
            "estimated_runtime_seconds": runtime_estimate["estimatedRuntimeSeconds"],
            "estimated_runtime_breakdown": runtime_estimate,
            "pen_lift_count": runtime_estimate["penLifts"],
        }
        state.update(
            last_svg_name=file.filename,
            last_gcode=gcode,
            last_preview=preview,
            last_summary=summary,
            progress_total=0,
            progress_done=0,
            run_started_at=None,
            run_finished_at=None,
            job_started_at=None,
            job_finished_at=None,
            job_elapsed_seconds=0.0,
            job_estimated_total_seconds=float(runtime_estimate["estimatedRuntimeSeconds"]),
            job_estimated_remaining_seconds=0.0,
            job_state="idle",
            runtime_estimate_multiplier=1.0,
            job_estimate_profile={
                "cumulative_seconds_by_stream_line": runtime_estimate.get("cumulativeSecondsByStreamLine") or [],
            },
            current_gcode_line=0,
            current_path_id=None,
            current_preview_point_index=0,
            status="Raster G-code generated - calibrate before run",
            last_error=None,
            last_timeout_debug=None,
        )
        projection_center_debug = coordinate_debug.get("projection_center_latitude") or {}
        resolved_center_lat_deg = float(projection_center_debug.get("resolved_center_lat_deg", options["placement_offset_y"]))
        auto_y_band_fit = coordinate_debug.get("auto_y_band_fit") if isinstance(coordinate_debug, dict) else None
        mask_projection_quad = build_mask_projection_quad(
            options=options,
            bounds=region_result.bounds,
            resolved_center_lat_deg=resolved_center_lat_deg,
            auto_y_band_fit=auto_y_band_fit if isinstance(auto_y_band_fit, dict) else None,
            artwork_scale_center_x_mm=(mapped_selected_bounds.min_x + mapped_selected_bounds.max_x) * 0.5,
            artwork_scale_center_y_mm=(mapped_selected_bounds.min_y + mapped_selected_bounds.max_y) * 0.5,
            origin_translation_x_mm=float((origin_translation or {}).get("x", 0.0)) if isinstance(origin_translation, dict) else 0.0,
            origin_translation_y_mm=float((origin_translation or {}).get("y", 0.0)) if isinstance(origin_translation, dict) else 0.0,
        )
        mask_projected_preview = build_projected_mask_preview_paths(
            placed_bundle=placed,
            center_lon_deg=float(options["placement_offset_x"]),
            center_lat_deg=float(resolved_center_lat_deg),
            ball_diameter_mm=float(current_app.config["BALL_DIAMETER_MM"]),
            pen_width_mm=float(options["line_thickness_mm"]),
        )
        _mark("preview_rendering")

        stage_durations = {
            "input_loading_s": round(float(stage_marks.get("input_loading", 0.0)), 4),
            "vector_extraction_tracing_s": round(_dur("mask_extraction", "vector_extraction"), 4),
            "polygon_cleanup_s": round(_dur("vector_extraction", "polygon_cleanup"), 4),
            "fill_path_generation_s": round(_dur("polygon_cleanup", "fill_path_generation"), 4),
            "path_ordering_s": round(_dur("fill_path_generation", "path_ordering"), 4),
            "gcode_writing_s": round(_dur("path_ordering", "gcode_writing"), 4),
            "preview_rendering_s": round(_dur("gcode_writing", "preview_rendering"), 4),
            "total_s": round(float(stage_marks.get("preview_rendering", 0.0)), 4),
        }
        slow_stages = {name: value for name, value in stage_durations.items() if name.endswith("_s") and name != "total_s" and value > 2.0}
        current_app.logger.info("Raster generation timings: %s", stage_durations)
        if slow_stages:
            current_app.logger.warning("Raster generation slow stages (>2s): %s", slow_stages)

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
            mask_projection_quad=mask_projection_quad,
            mask_projected_preview=mask_projected_preview,
            regions=raster.serialize_regions(region_result),
            selected_colors=options["selected_colors"],
            summary=summary,
            stage_counts=stage_counts,
            effective_settings=effective_settings,
            coordinate_debug=coordinate_debug,
            printable_x_span_debug=x_span_debug,
            outline_pipeline_debug=outline_pipeline_debug,
            outline_fill_alignment_debug=(debug_data or {}).get("outline_fill_alignment_debug"),
            region_alignment_debug=region_alignment_debug,
            sampling_debug=sampling_debug,
            machine_motion_debug=machine_motion_debug,
            outline_vs_infill_alignment=outline_vs_infill_alignment_debug,
            infill_debug=(debug_data or {}).get("infill_debug"),
            gcode_stats=gcode_stats,
            stage_timings=stage_durations,
            debug=debug_data,
        )
    except Exception as exc:
        log_exception("Generate raster G-code failed", exc)
        state.update(last_error=str(exc), status=f"Generate error: {exc}")
        selected_colors = []
        try:
            selected_colors = validation.parse_generate_raster_form(request.form, config).get("selected_colors", [])
        except Exception:
            selected_colors = []
        return jsonify({
            "ok": False,
            "error": str(exc),
            "setting_debug": build_setting_debug(exc, config),
            "debug": build_generate_debug_payload(selected_colors=selected_colors),
        }), 500


@raster_bp.post("/generate-diagnostic-gcode")
def generate_diagnostic_gcode_route():
    state = get_state()
    config = current_app.config
    pattern = str(request.form.get("pattern", "diagnostic_suite"))
    mode = str(request.form.get("mode", "fill_then_cleanup"))
    line_width_mm = float(request.form.get("line_thickness_mm", config["DEFAULT_LINE_THICKNESS_MM"]))
    draw_feed = float(request.form.get("draw_feed", config["DEFAULT_DRAW_FEED"]))
    travel_feed = float(request.form.get("travel_feed", config["DEFAULT_TRAVEL_FEED"]))
    wall_count = int(request.form.get("wall_count", 1))
    debug_data: dict[str, object] = {}
    try:
        if pattern == "x_axis_rotation_ticks":
            machine_toolpaths, tick_specs = pipeline_core.build_x_axis_rotation_calibration_toolpaths()
            gcode, preview = get_gcode_service().generate_from_toolpaths(
                toolpaths=machine_toolpaths,
                draw_feed=draw_feed,
                travel_feed=travel_feed,
                sample_step_deg=config["DEFAULT_SAMPLE_STEP_DEG"],
                placement_offset_x=0.0,
                placement_offset_y=0.0,
                pen_up_s=config["DEFAULT_PEN_UP_S"],
                pen_down_s=config["DEFAULT_PEN_DOWN_S"],
                servo_ramp_enabled=config["DEFAULT_SERVO_RAMP_ENABLED"],
                servo_ramp_step=config["DEFAULT_SERVO_RAMP_STEP"],
                servo_ramp_delay_ms=config["DEFAULT_SERVO_RAMP_DELAY_MS"],
                pen_up_dwell_ms=config["DEFAULT_PEN_UP_DWELL_MS"],
                pen_down_dwell_ms=config["DEFAULT_PEN_DOWN_DWELL_MS"],
                gcode_mode=config["DEFAULT_GCODE_MODE"],
                include_comments=True,
                debug=debug_data,
            )
            machine_motion_debug = pipeline_core.build_machine_motion_debug(
                [],
                machine_toolpaths,
                preview,
                gcode,
                pen_up_s=config["DEFAULT_PEN_UP_S"],
                pen_down_s=config["DEFAULT_PEN_DOWN_S"],
            )
            x_axis_calibration_pattern = pipeline_core.build_x_axis_rotation_calibration_metadata(
                tick_specs,
                machine_toolpaths,
                gcode,
                ball_diameter_mm=config["BALL_DIAMETER_MM"],
                pen_up_s=config["DEFAULT_PEN_UP_S"],
                pen_down_s=config["DEFAULT_PEN_DOWN_S"],
            )
            point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
            runtime_estimate = estimate_runtime_seconds(
                gcode,
                draw_feed=draw_feed,
                travel_feed=travel_feed,
                pen_up_s=config["DEFAULT_PEN_UP_S"],
                pen_down_s=config["DEFAULT_PEN_DOWN_S"],
            )
            toolpath_counts = {
                "fill-wall": 0,
                "fill-infill": 0,
                "detail-trace": 0,
                "outline": len(machine_toolpaths),
                "travel": sum(1 for path in preview if path["kind"] == "travel"),
            }
            summary = {
                "image_size": f"diagnostic:{pattern}",
                "selected_colors": [],
                "mask_pixel_count": 0,
                "component_count": len(machine_toolpaths),
                "toolpath_counts": toolpath_counts,
                "wall_path_count": 0,
                "infill_path_count": 0,
                "detail_trace_path_count": 0,
                "travel_path_count": toolpath_counts["travel"],
                "gcode_line_count": runtime_estimate["rawGcodeLines"],
                "streamable_gcode_line_count": runtime_estimate["streamableGcodeLines"],
                "point_count": point_count,
                "estimated_runtime_seconds": runtime_estimate["estimatedRuntimeSeconds"],
                "estimated_runtime_breakdown": runtime_estimate,
                "pen_lift_count": runtime_estimate["penLifts"],
            }
            state.update(
                last_svg_name=f"diagnostic:{pattern}",
                last_gcode=gcode,
                last_preview=preview,
                last_summary=summary,
                progress_total=0,
                progress_done=0,
                run_started_at=None,
                run_finished_at=None,
                job_started_at=None,
                job_finished_at=None,
                job_elapsed_seconds=0.0,
                job_estimated_total_seconds=float(runtime_estimate["estimatedRuntimeSeconds"]),
                job_estimated_remaining_seconds=0.0,
                job_state="idle",
                runtime_estimate_multiplier=1.0,
                job_estimate_profile={
                    "cumulative_seconds_by_stream_line": runtime_estimate.get("cumulativeSecondsByStreamLine") or [],
                },
                current_gcode_line=0,
                current_path_id=None,
                current_preview_point_index=0,
                status="X rotary calibration G-code generated - calibrate before run",
                last_error=None,
                last_timeout_debug=None,
            )
            return json_ok(
                pattern=pattern,
                mode=mode,
                gcode=gcode,
                preview=preview,
                mask_preview=None,
                selected_colors=[],
                summary=summary,
                stage_counts={},
                effective_settings={
                    "line_thickness_mm": line_width_mm,
                    "infill_spacing_mm": line_width_mm,
                    "custom_infill_spacing": False,
                    "wall_count": wall_count,
                    "fill_density": 0.0,
                },
                calibrationPattern=None,
                xAxisCalibrationPattern=x_axis_calibration_pattern,
                coordinate_debug={"unit_model": "machine_deg_direct_calibration"},
                outline_pipeline_debug={"preview_and_gcode_share_same_projected_paths": True},
                region_alignment_debug={},
                sampling_debug={},
                machine_motion_debug=machine_motion_debug,
                outline_vs_infill_alignment={},
                gcode_stats=build_gcode_stats(gcode, {"duplicate_points_removed": 0, "short_segments_removed": 0, "simplification_tolerance_mm": 0.0}, preview_path_count=len(preview), debug=debug_data),
                debug=debug_data,
            )

        bundle = pipeline_core.build_diagnostic_geometry_bundle(pattern)
        toolpaths = pipeline_core.generate_toolpaths(
            bundle,
            enable_fill=True,
            line_width_mm=line_width_mm,
            wall_count=wall_count,
            infill_density=100.0,
            infill_spacing_mm=line_width_mm,
            infill_angle_deg=0.0,
            outline_after_fill=(mode != "outline_only"),
            min_fill_area_mm2=0.0,
            min_fill_width_mm=0.0,
            simplify_tolerance_mm=0.0,
            remove_duplicate_paths=False,
            small_shape_mode="single-wall",
            min_segment_length_mm=0.0,
            travel_optimization="nearest-neighbor",
            debug=debug_data,
        )
        if mode == "infill_only":
            toolpaths = [path for path in toolpaths if path.kind == "fill-infill"]
        elif mode == "outline_only":
            toolpaths = [path for path in toolpaths if path.kind == "outline"]
        options = {
            "simplify_tolerance_mm": 0.0,
            "min_segment_length_mm": 0.0,
            "line_thickness_mm": line_width_mm,
            "placement_offset_x": 0.0,
            "placement_offset_y": 0.0,
        }
        cleaned_toolpaths, projected_toolpaths, cleanup_stats, coordinate_debug, outline_pipeline_debug, region_alignment_debug, sampling_debug, outline_vs_infill_alignment_debug = project_surface_toolpaths(toolpaths, options)
        gcode, preview = get_gcode_service().generate_from_toolpaths(
            toolpaths=projected_toolpaths,
            draw_feed=draw_feed,
            travel_feed=travel_feed,
            sample_step_deg=config["DEFAULT_SAMPLE_STEP_DEG"],
            placement_offset_x=0.0,
            placement_offset_y=0.0,
            pen_up_s=config["DEFAULT_PEN_UP_S"],
            pen_down_s=config["DEFAULT_PEN_DOWN_S"],
            servo_ramp_enabled=config["DEFAULT_SERVO_RAMP_ENABLED"],
            servo_ramp_step=config["DEFAULT_SERVO_RAMP_STEP"],
            servo_ramp_delay_ms=config["DEFAULT_SERVO_RAMP_DELAY_MS"],
            pen_up_dwell_ms=config["DEFAULT_PEN_UP_DWELL_MS"],
            pen_down_dwell_ms=config["DEFAULT_PEN_DOWN_DWELL_MS"],
            gcode_mode=config["DEFAULT_GCODE_MODE"],
            include_comments=True,
            debug=debug_data,
        )
        machine_motion_debug = pipeline_core.build_machine_motion_debug(
            cleaned_toolpaths,
            projected_toolpaths,
            preview,
            gcode,
            pen_up_s=config["DEFAULT_PEN_UP_S"],
            pen_down_s=config["DEFAULT_PEN_DOWN_S"],
        )
        calibration_pattern = pipeline_core.build_calibration_pattern_metadata(
            pattern,
            bundle,
            cleaned_toolpaths,
            projected_toolpaths,
            gcode,
            ball_diameter_mm=config["BALL_DIAMETER_MM"],
            pen_up_s=config["DEFAULT_PEN_UP_S"],
            pen_down_s=config["DEFAULT_PEN_DOWN_S"],
        )
        point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
        runtime_estimate = estimate_runtime_seconds(
            gcode,
            draw_feed=draw_feed,
            travel_feed=travel_feed,
            pen_up_s=config["DEFAULT_PEN_UP_S"],
            pen_down_s=config["DEFAULT_PEN_DOWN_S"],
        )
        toolpath_counts = {
            "fill-wall": sum(1 for path in toolpaths if path.kind == "fill-wall"),
            "fill-infill": sum(1 for path in toolpaths if path.kind == "fill-infill"),
            "detail-trace": sum(1 for path in toolpaths if path.kind == "detail-trace"),
            "outline": sum(1 for path in toolpaths if path.kind == "outline"),
            "travel": 0,
        }
        summary = {
            "image_size": f"diagnostic:{pattern}",
            "selected_colors": [],
            "mask_pixel_count": 0,
            "component_count": len(pipeline_core.normalize_geometry(bundle.printable_geometry)),
            "toolpath_counts": toolpath_counts,
            "wall_path_count": toolpath_counts["fill-wall"],
            "infill_path_count": toolpath_counts["fill-infill"],
            "detail_trace_path_count": toolpath_counts["detail-trace"],
            "travel_path_count": sum(1 for path in preview if path["kind"] == "travel"),
            "gcode_line_count": runtime_estimate["rawGcodeLines"],
            "streamable_gcode_line_count": runtime_estimate["streamableGcodeLines"],
            "point_count": point_count,
            "estimated_runtime_seconds": runtime_estimate["estimatedRuntimeSeconds"],
            "estimated_runtime_breakdown": runtime_estimate,
            "pen_lift_count": runtime_estimate["penLifts"],
        }
        state.update(
            last_svg_name=f"diagnostic:{pattern}",
            last_gcode=gcode,
            last_preview=preview,
            last_summary=summary,
            progress_total=0,
            progress_done=0,
            run_started_at=None,
            run_finished_at=None,
            job_started_at=None,
            job_finished_at=None,
            job_elapsed_seconds=0.0,
            job_estimated_total_seconds=float(runtime_estimate["estimatedRuntimeSeconds"]),
            job_estimated_remaining_seconds=0.0,
            job_state="idle",
            runtime_estimate_multiplier=1.0,
            job_estimate_profile={
                "cumulative_seconds_by_stream_line": runtime_estimate.get("cumulativeSecondsByStreamLine") or [],
            },
            current_gcode_line=0,
            current_path_id=None,
            current_preview_point_index=0,
            status="Diagnostic G-code generated - calibrate before run",
            last_error=None,
            last_timeout_debug=None,
        )
        return json_ok(
            pattern=pattern,
            mode=mode,
            gcode=gcode,
            preview=preview,
            mask_preview=None,
            selected_colors=[],
            summary=summary,
            stage_counts={},
            effective_settings={
                "line_thickness_mm": line_width_mm,
                "infill_spacing_mm": line_width_mm,
                "custom_infill_spacing": False,
                "wall_count": wall_count,
                "fill_density": 100.0,
            },
            calibrationPattern=calibration_pattern,
            coordinate_debug=coordinate_debug,
            outline_pipeline_debug=outline_pipeline_debug,
            region_alignment_debug=region_alignment_debug,
            sampling_debug=sampling_debug,
            machine_motion_debug=machine_motion_debug,
            outline_vs_infill_alignment=outline_vs_infill_alignment_debug,
            gcode_stats=build_gcode_stats(gcode, cleanup_stats, preview_path_count=len(preview), debug=debug_data),
            debug=debug_data,
        )
    except Exception as exc:
        log_exception("Generate diagnostic G-code failed", exc)
        return json_error(str(exc), status=500)
