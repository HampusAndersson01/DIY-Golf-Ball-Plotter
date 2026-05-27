from dataclasses import asdict

from flask import Blueprint, current_app, request

from app.extensions import (
    get_gcode_service,
    get_geometry_service,
    get_raster_analysis_service,
    get_self_test_service,
    get_state,
    get_svg_parser,
    get_toolpath_service,
    get_validation_service,
)
from app.services import pipeline_core
from app.utils.response_utils import json_error, json_ok, log_exception

svg_bp = Blueprint("svg", __name__)


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


def build_effective_settings(options: dict) -> dict:
    return {
        "artwork_scale_percent": options["artwork_scale_percent"],
        "line_thickness_mm": options["line_thickness_mm"],
        "infill_spacing_mm": options["effective_infill_spacing_mm"],
        "custom_infill_spacing": options["custom_infill_spacing"],
        "wall_count": options["wall_count"],
        "fill_density": options["infill_density"],
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
    projected_toolpaths = pipeline_core.project_toolpaths_to_ball_angles(
        cleaned_toolpaths,
        center_lon_deg=options["placement_offset_x"],
        center_lat_deg=options["placement_offset_y"],
        ball_diameter_mm=current_app.config["BALL_DIAMETER_MM"],
    )
    pipeline_core.validate_toolpaths_finite(projected_toolpaths, coordinate_space="machine_deg")
    lifecycle_logs, outline_pipeline_debug = pipeline_core.build_toolpath_lifecycle_debug(cleaned_toolpaths, projected_toolpaths)
    pipeline_core.log_physical_outline_mismatch_check(cleaned_toolpaths, projected_toolpaths)
    coordinate_debug = {
        "unit_model": "surface_mm_then_project_once_to_machine_deg",
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


@svg_bp.post("/generate-gcode")
def generate_gcode_route():
    state = get_state()
    validation = get_validation_service()
    config = current_app.config
    try:
        current_app.logger.info(
            "SVG G-code generation requested: files=%s form_keys=%s",
            sorted(list(request.files.keys())),
            sorted(list(request.form.keys())),
        )
        raster_file = request.files.get("image") or request.files.get("raster")
        if raster_file is not None:
            image_bytes = raster_file.read()
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
                debug=debug_data,
            )
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
                outline_pipeline_debug = debug_data["outline_pipeline_debug"]
            state.update(
                last_svg_name=raster_file.filename,
                last_gcode=gcode,
                last_preview=preview,
                progress_total=0,
                progress_done=0,
                status="Raster G-code generated - calibrate before run",
                last_error=None,
                last_timeout_debug=None,
            )
            current_app.logger.info(
                "Raster-in-SVG endpoint generated G-code: file=%s toolpaths=%d preview_paths=%d gcode_lines=%d",
                raster_file.filename,
                len(toolpaths),
                len(preview),
                len(gcode),
            )
            return json_ok(
                gcode=gcode,
                preview=preview,
                toolpath_count=len(toolpaths),
                toolpath_diagnostics=toolpath_diagnostics,
                point_count=sum(len(path["points"]) for path in preview if path["kind"] != "travel"),
                mask_pixel_count=mask_result.printable_pixel_count,
                component_count=mask_result.connected_component_count,
                mask=raster.serialize_mask(mask_result),
                mask_preview=mask_result.mask_preview_url,
                regions=raster.serialize_regions(region_result),
                selected_colors=options["selected_colors"],
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
                gcode_stats=build_gcode_stats(gcode, cleanup_stats, preview_path_count=len(preview), debug=debug_data),
                debug=debug_data,
            )

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
        mapped = geometry.map_bundle_to_surface_mm(bundle, bounds, options["fit_mode"], options["invert_y"], options["margin_percent"])
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

        toolpaths = get_toolpath_service().generate_toolpaths(
            placed,
            enable_fill=options["enable_fill"],
            line_width_mm=options["line_thickness_mm"],
            wall_count=options["wall_count"],
            infill_density=options["infill_density"],
            infill_spacing_mm=options["effective_infill_spacing_mm"],
            infill_angle_deg=options["infill_angle_deg"],
            outline_after_fill=options["outline_after_fill"],
            min_fill_area_mm2=options["min_fill_area_mm2"],
            min_fill_width_mm=options["min_fill_width_mm"],
            simplify_tolerance_mm=options["simplify_tolerance_mm"],
            remove_duplicate_paths=options["remove_duplicate_paths"],
            small_shape_mode=options["small_shape_mode"],
            fill_strategy=options["fill_strategy"],
            alternate_fill_angle_deg=options["alternate_fill_angle_deg"],
            thin_detail_mode=options["thin_detail_mode"],
            thin_detail_min_area_mm2=options["thin_detail_min_area_mm2"],
            thin_detail_simplify_mm=options["thin_detail_simplify_mm"],
            thin_detail_overlap=options["thin_detail_overlap"],
            min_segment_length_mm=options["min_segment_length_mm"],
            travel_optimization=options["travel_optimization"],
            allow_pen_down_infill_connectors=options["allow_pen_down_infill_connectors"],
            infill_path_mode=options["infill_path_mode"],
            debug=debug_data,
        )
        if not toolpaths:
            raise ValueError("No toolpaths were generated from the current SVG/settings")

        toolpath_diagnostics = get_toolpath_service().summarize_toolpaths(toolpaths)
        current_app.logger.debug(
            "Generated toolpath counts: wall_paths=%d infill_paths=%d infill_segments=%d",
            sum(1 for path in toolpaths if path.kind == "fill-wall"),
            sum(1 for path in toolpaths if path.kind == "fill-infill"),
            sum(max(0, len(path.points) - 1) for path in toolpaths if path.kind == "fill-infill"),
        )
        cleaned_toolpaths, projected_toolpaths, cleanup_stats, coordinate_debug, outline_pipeline_debug, region_alignment_debug, sampling_debug, outline_vs_infill_alignment_debug = project_surface_toolpaths(toolpaths, options)
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
            outline_pipeline_debug = debug_data["outline_pipeline_debug"]

        point_count = sum(len(path["points"]) for path in preview if path["kind"] != "travel")
        state.update(
            last_svg_name=file.filename,
            last_gcode=gcode,
            last_preview=preview,
            progress_total=0,
            progress_done=0,
            status="G-code generated - calibrate before run",
            last_error=None,
            last_timeout_debug=None,
        )
        current_app.logger.info(
            "SVG G-code generated: file=%s toolpaths=%d preview_paths=%d gcode_lines=%d",
            file.filename,
            len(toolpaths),
            len(preview),
            len(gcode),
        )

        return json_ok(
            gcode=gcode,
            preview=preview,
            toolpath_count=len(toolpaths),
            toolpath_diagnostics=toolpath_diagnostics,
            point_count=point_count,
            bounds=asdict(bounds),
            viewbox_bounds=asdict(viewbox_bounds) if viewbox_bounds else None,
            print_model=asdict(print_model),
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
            gcode_stats=build_gcode_stats(gcode, cleanup_stats, preview_path_count=len(preview), debug=debug_data),
            debug=debug_data,
        )
    except Exception as exc:
        log_exception("Generate SVG G-code failed", exc)
        state.update(last_error=str(exc), status=f"Generate error: {exc}")
        return json_error(str(exc), status=500, setting_debug=build_setting_debug(exc, config))


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
        log_exception("Analyze SVG failed", exc)
        return json_error(str(exc), status=500)


@svg_bp.post("/self-test-svg-pipeline")
def self_test_svg_pipeline_route():
    try:
        summary = get_self_test_service().run()
        return json_ok(summary=summary)
    except Exception as exc:
        log_exception("SVG pipeline self-test failed", exc)
        return json_error(str(exc), status=500)
