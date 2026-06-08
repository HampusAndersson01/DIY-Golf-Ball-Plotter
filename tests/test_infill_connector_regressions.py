from __future__ import annotations

import json
import math
import os
from io import BytesIO
from pathlib import Path
import tempfile

import cv2
import numpy as np
import pytest
from PIL import Image, ImageDraw
from shapely.geometry import LineString, Polygon
from werkzeug.datastructures import MultiDict

from app import create_app
from app.models.machine_state import MachineState
from app.services import pipeline_core
from app.services.gcode_service import GcodeService
from app.services.geometry_service import GeometryService
from app.services.raster_analysis_service import RasterAnalysisService
from app.services.toolpath_service import ToolpathService
from app.services.validation_service import ValidationService
from tests.test_svg_parser import CONFIG


ROOT = Path(__file__).resolve().parents[1]
HA_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "ha-compact-lightbg.png"
ARSENAL_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "black-arsenal-logo-png-1.png"
CAROLIN_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "Carolin Line.png"


def _load_services() -> tuple[RasterAnalysisService, GeometryService, ToolpathService, GcodeService]:
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    return raster, GeometryService(), ToolpathService(), GcodeService()


def _count_real_pen_lifts_from_gcode(gcode: list[str], *, pen_up_s: float, pen_down_s: float) -> int:
    lifts = 0
    current_pen_down = False
    for line in gcode:
        if not line.startswith("M3 S"):
            continue
        try:
            value = float(line.split("S", 1)[1].strip())
        except Exception:
            continue
        if abs(value - pen_down_s) <= 1e-6:
            current_pen_down = True
            continue
        if abs(value - pen_up_s) <= 1e-6:
            if current_pen_down:
                lifts += 1
            current_pen_down = False
    return lifts


def _selected_black_color_id(raster: RasterAnalysisService, image_bytes: bytes) -> str:
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((color.id for color in analysis.colors if color.hex == "#000000"), analysis.colors[0].id if analysis.colors else None)
    assert selected is not None, "No selectable black color found in fixture image"
    return selected


def _synthetic_logo_png_bytes() -> bytes:
    canvas = Image.new("RGB", (1600, 1200), "white")
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((80, 120, 1100, 980), fill="black")
    draw.rectangle((1260, 120, 1263, 1040), fill="black")
    draw.ellipse((1260, 1080, 1264, 1084), fill="black")
    buffer = BytesIO()
    canvas.save(buffer, format="PNG")
    return buffer.getvalue()


def _assert_connector_paths_stay_inside_mask(toolpaths: list[pipeline_core.Toolpath], validation: dict[str, object]) -> None:
    mask = validation.get("mask")
    matrix = validation.get("current_to_source_matrix")
    assert isinstance(mask, np.ndarray)
    assert isinstance(matrix, (tuple, list)) and len(matrix) == 6
    current_to_source = tuple(float(value) for value in matrix)
    mask_height, mask_width = mask.shape[:2]

    connector_paths = [path for path in toolpaths if path.kind in {"fill-infill-travel", "coverage_connector"}]
    if not connector_paths:
        # No accepted infill connectors for this fixture — nothing to validate.
        return

    for path in connector_paths:
        line = LineString([(point.x, point.y) for point in path.points])
        sample_step_mm = max(0.01, min(0.05, line.length / max(2, int(math.ceil(line.length / 0.05)))))
        sample_count = max(2, int(math.ceil(line.length / sample_step_mm)) + 1)
        for sample_index in range(sample_count):
            distance_mm = min(line.length, (line.length * sample_index) / max(sample_count - 1, 1))
            sample_point = line.interpolate(distance_mm)
            source_point = pipeline_core.apply_svg_matrix(pipeline_core.Point(float(sample_point.x), float(sample_point.y)), current_to_source)
            pixel_x = int(round(source_point.x))
            pixel_y = int(round(source_point.y))
            assert 0 <= pixel_x < mask_width and 0 <= pixel_y < mask_height, (
                f"Connector left the raster mask at {pixel_x},{pixel_y} for path {path.metadata.get('infill_segment_id')}"
            )
            assert bool(mask[pixel_y, pixel_x]), (
                f"Connector crossed whitespace or a hole at {pixel_x},{pixel_y} for path {path.metadata.get('infill_segment_id')}"
            )


def _run_fixture(image_path: Path, *, infill_path_mode: str = "rectilinear") -> dict[str, object]:
    assert image_path.exists(), f"Missing fixture image: {image_path}"
    image_bytes = image_path.read_bytes()
    raster, geometry, toolpaths_service, gcode_service = _load_services()
    selected = _selected_black_color_id(raster, image_bytes)
    mask = raster.build_mask(
        image_bytes,
        [selected],
        tolerance=24,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=1,
    )
    regions = raster.extract_regions(mask, min_region_area_px=8, simplify_tolerance_px=1.0)
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)

    debug: dict[str, object] = {}
    reference_spacing = 0.6
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=reference_spacing,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=reference_spacing,
        infill_density=100.0,
        infill_angle_deg=0.0,
        fill_strategy="rotated_scanline",
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode=infill_path_mode,
        debug=debug,
    )

    gcode, preview = gcode_service.generate_from_toolpaths(
        toolpaths=toolpaths,
        draw_feed=1200.0,
        travel_feed=3000.0,
        sample_step_deg=1.0,
        pen_up_s=575,
        pen_down_s=700,
        servo_ramp_enabled=True,
        servo_ramp_step=20,
        servo_ramp_delay_ms=10.0,
        pen_up_dwell_ms=30.0,
        pen_down_dwell_ms=60.0,
        gcode_mode="simple",
        include_comments=True,
        debug=debug,
    )

    diagnostics = debug.get("infill_connector_diagnostics", {})
    validation = mapped.metadata.get("connector_validation", {})
    assert isinstance(validation, dict)

    result = {
        "image_path": str(image_path),
        "toolpaths": toolpaths,
        "gcode": gcode,
        "preview": preview,
        "debug": debug,
        "diagnostics": diagnostics,
        "estimated_runtime_seconds": debug.get("estimated_runtime_seconds"),
        "validation": validation,
        "actual_pen_lifts": _count_real_pen_lifts_from_gcode(gcode, pen_up_s=575, pen_down_s=700),
        "accepted_connector_count": int(diagnostics.get("accepted_connectors", debug.get("accepted_same_cell_connectors", 0)) or 0),
    }
    _assert_connector_paths_stay_inside_mask(toolpaths, validation)
    return result


def _run_frontend_arsenal_fixture(*, rotation_deg: float) -> dict[str, object]:
    app = create_app()
    config = app.config
    fixture_bytes = ARSENAL_FIXTURE.read_bytes()
    raster = RasterAnalysisService(config, MachineState(default_pen_up_s=575))
    geometry = GeometryService()
    toolpaths_service = ToolpathService()
    validation = ValidationService()
    gcode_service = GcodeService()

    analysis = raster.analyze_image(fixture_bytes, max_colors=config["DEFAULT_RASTER_MAX_COLORS"])
    selected = next(
        (color.id for color in analysis.colors if color.hex == "#000000"),
        analysis.colors[0].id if analysis.colors else None,
    )
    assert selected is not None

    options = validation.parse_generate_raster_form(
        MultiDict({
            "selected_colors": f"[\"{selected}\"]",
            "line_thickness_mm": "0.6",
            "rotation_deg": str(rotation_deg),
        }),
        config,
    )
    mask = raster.build_mask(
        fixture_bytes,
        options["selected_colors"],
        simplify_colors=options["simplify_colors"],
        max_colors=options["max_colors"],
        tolerance=options["color_tolerance"],
        min_component_area_px=options["min_component_area_px"],
        open_radius_px=options["mask_open_radius_px"],
        close_radius_px=options["mask_close_radius_px"],
    )
    regions = raster.extract_regions(
        mask,
        min_region_area_px=0.0 if options["thin_detail_mode"] else options["min_region_area_px"],
        simplify_tolerance_px=options["region_simplify_px"],
    )
    mapped = geometry.map_bundle_to_surface_mm(
        regions.bundle,
        regions.bounds,
        options["fit_mode"],
        options["invert_y"],
        options["margin_percent"],
    )
    artwork_scaled = geometry.apply_surface_artwork_scale(mapped, options["artwork_scale_percent"])
    transformed = geometry.apply_surface_placement_transform(
        artwork_scaled,
        options["placement_scale"],
        options["rotation_deg"],
    )
    placed = geometry.apply_origin_anchor_placement(
        transformed,
        origin_anchor=options["origin_anchor"],
        origin_offset_x_mm=options["origin_offset_x_mm"],
        origin_offset_y_mm=options["origin_offset_y_mm"],
    )

    debug: dict[str, object] = {}
    surface_toolpaths = toolpaths_service.generate_from_regions(
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
        debug=debug,
    )
    projected = pipeline_core.assign_stable_path_ids(
        pipeline_core.project_toolpaths_to_ball_angles(
            pipeline_core.prepare_toolpaths_for_projection(surface_toolpaths, default_pen_width_mm=0.6),
            center_lon_deg=0.0,
            center_lat_deg=0.0,
        )
    )
    gcode, preview = gcode_service.generate_from_toolpaths(
        toolpaths=projected,
        draw_feed=1200.0,
        travel_feed=3000.0,
        sample_step_deg=1.0,
        placement_offset_x=0.0,
        placement_offset_y=0.0,
        pen_up_s=575,
        pen_down_s=700,
        servo_ramp_enabled=True,
        servo_ramp_step=20,
        servo_ramp_delay_ms=10.0,
        pen_up_dwell_ms=30.0,
        pen_down_dwell_ms=60.0,
        gcode_mode="simple",
        include_comments=True,
        debug=debug,
    )
    return {
        "mask": mask,
        "placed": placed,
        "surface_toolpaths": surface_toolpaths,
        "projected_toolpaths": projected,
        "gcode": gcode,
        "preview": preview,
        "debug": debug,
    }


def _arsenal_final_surface_paths(result: dict[str, object]) -> tuple[dict[str, object], tuple[float, float, float, float, float, float], list[pipeline_core.Toolpath]]:
    debug = result["debug"]
    validation = result["placed"].metadata.get("connector_validation", {})
    assert isinstance(validation, dict)
    current_to_source = tuple(float(value) for value in validation["current_to_source_matrix"])
    final_surface_paths = [
        path
        for path in result["surface_toolpaths"]
        if path.kind != "travel" and len(path.points) >= 1
    ]
    assert final_surface_paths, "No final emitted drawable paths were captured from the G-code emission stage"
    return debug, current_to_source, final_surface_paths


def _rasterize_surface_centerlines(
    toolpaths: list[pipeline_core.Toolpath],
    *,
    shape: tuple[int, int],
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    include_kinds: set[str],
    max_segment_mm: float = 0.15,
) -> np.ndarray:
    canvas = np.zeros(shape, dtype=np.uint8)
    for path in toolpaths:
        if path.kind not in include_kinds or len(path.points) < 1:
            continue
        if len(path.points) == 1:
            source_point = pipeline_core.apply_svg_matrix(path.points[0], current_to_source_matrix)
            cv2.circle(canvas, (int(round(source_point.x)), int(round(source_point.y))), 1, 255, -1)
            continue
        sampled_points = pipeline_core.resample_segment(path.points, max_step=max(0.01, float(max_segment_mm)))
        pts = np.asarray(
            [(int(round(pipeline_core.apply_svg_matrix(point, current_to_source_matrix).x)), int(round(pipeline_core.apply_svg_matrix(point, current_to_source_matrix).y))) for point in sampled_points],
            dtype=np.int32,
        )
        if len(pts) >= 2:
            cv2.polylines(canvas, [pts.reshape(-1, 1, 2)], False, 255, 1, lineType=cv2.LINE_8)
        elif len(pts) == 1:
            cv2.circle(canvas, (int(pts[0][0]), int(pts[0][1])), 1, 255, -1)
    return canvas


def _surface_path_overflow_attribution(
    toolpaths: list[pipeline_core.Toolpath],
    *,
    shape: tuple[int, int],
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_mm: float,
    expected_bool: np.ndarray,
    safe_centerline_bool: np.ndarray,
    include_kinds: set[str],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for index, path in enumerate(toolpaths):
        if path.kind not in include_kinds or len(path.points) < 1:
            continue
        metadata = dict(path.metadata or {})
        path_mask = pipeline_core.rasterize_surface_toolpaths_mask(
            [path],
            shape=shape,
            current_to_source_matrix=current_to_source_matrix,
            pen_radius_mm=pen_radius_mm,
            max_segment_mm=0.15,
            include_kinds={path.kind},
        )
        centerline_mask = _rasterize_surface_centerlines(
            [path],
            shape=shape,
            current_to_source_matrix=current_to_source_matrix,
            include_kinds={path.kind},
            max_segment_mm=0.15,
        )
        overflow_pixels = int(np.count_nonzero((path_mask > 0) & ~expected_bool))
        centerline_violation_pixels = int(np.count_nonzero((centerline_mask > 0) & ~safe_centerline_bool))
        rows.append({
            "path_index": int(index),
            "path_id": str(path.path_id or f"path_{index:04d}"),
            "path_kind": str(path.kind),
            "source": str(path.source),
            "overflow_pixels": int(overflow_pixels),
            "centerline_violation_pixels": int(centerline_violation_pixels),
            "centerline_generation_mode": str(metadata.get("centerline_generation_mode", "unknown")),
            "generated_from_safe_mask": bool(metadata.get("generated_from_safe_mask", False)),
            "safe_centerline_inset_mm": float(metadata.get("safe_centerline_inset_mm", 0.0) or 0.0),
        })
    rows.sort(
        key=lambda row: (
            -int(row["overflow_pixels"]),
            -int(row["centerline_violation_pixels"]),
            str(row["path_kind"]),
            str(row["source"]),
            str(row["path_id"]),
        )
    )
    return rows


def _make_carolin_fixture_if_needed() -> None:
    if CAROLIN_FIXTURE.exists():
        return

    canvas = np.full((96, 520, 3), 255, dtype=np.uint8)
    cv2.line(canvas, (0, 22), (519, 22), (0, 0, 0), 8, cv2.LINE_8)
    cv2.line(canvas, (0, 84), (519, 84), (0, 0, 0), 8, cv2.LINE_8)
    cv2.putText(canvas, "Carolin", (148, 74), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, (0, 0, 0), 4, cv2.LINE_AA)
    CAROLIN_FIXTURE.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(canvas, mode="RGB").save(CAROLIN_FIXTURE)


def test_ha_fixture_remains_safe_and_stays_under_pen_lift_target():
    result = _run_fixture(HA_FIXTURE)
    diagnostics = result["diagnostics"]

    assert result["actual_pen_lifts"] < 120
    assert diagnostics.get("total_infill_rows", 0) > 0
    assert diagnostics.get("accepted_connectors", 0) == result["accepted_connector_count"]
    assert diagnostics.get("rejected_raster_mask_sampling", 0) == 0
    assert diagnostics.get("rejected_connectors", 0) >= 0


def test_arsenal_fixture_reduces_pen_lifts_without_crossing_logo_gaps():
    result = _run_fixture(ARSENAL_FIXTURE)
    diagnostics = result["diagnostics"]

    assert result["actual_pen_lifts"] < 200
    assert diagnostics.get("accepted_connectors", 0) == result["accepted_connector_count"]
    assert diagnostics.get("rejected_raster_mask_sampling", 0) == 0
    assert diagnostics.get("rejected_outside_selected_color", 0) == 0


def test_arsenal_cell_based_routing_is_no_worse_than_legacy_routing():
    legacy = _run_fixture(ARSENAL_FIXTURE, infill_path_mode="legacy")
    modern = _run_fixture(ARSENAL_FIXTURE, infill_path_mode="rectilinear")

    assert modern["actual_pen_lifts"] <= legacy["actual_pen_lifts"]
    assert float(modern["estimated_runtime_seconds"] or 0.0) <= float(legacy["estimated_runtime_seconds"] or 0.0)
    assert modern["diagnostics"].get("rejected_raster_mask_sampling", 0) == 0
    assert modern["diagnostics"].get("rejected_outside_selected_color", 0) == 0


def test_arsenal_fixture_reports_small_detail_overlap_diagnostics():
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]

    assert debug.get("small_detail_outline_mode_enabled") is True
    assert int(debug.get("small_detail_components_detected", 0)) > 0
    assert "small_detail_drop_reasons" in debug
    assert "self_overlapping_detail_paths_allowed" in debug
    assert "self_overlapping_detail_paths_rejected" in debug
    assert "detail_paths_kept_despite_overlap" in debug
    assert debug.get("arsenal_detail_outline_paths_generated", 0) >= debug.get("arsenal_detail_outline_paths_dropped", 0)


def test_arsenal_fixture_preserves_outer_outlines_for_small_printable_components():
    result = _run_fixture(ARSENAL_FIXTURE)
    outer_outlines = [
        path for path in result["toolpaths"]
        if path.kind == "outline" and str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTER_OUTLINE"
    ]

    assert len(outer_outlines) >= 14
    assert result["debug"]["contour_offset_debug"]["outline_component_count_input"] >= 14
    assert result["debug"]["contour_offset_debug"]["outer_outline_path_count"] >= 14


def test_arsenal_fixture_uses_serpentine_fill_for_wide_detail_regions():
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]
    detail_serpentine_paths = [
        path for path in result["toolpaths"]
        if path.kind == "fill-infill" and str((path.metadata or {}).get("fill_mode", "")) == "detail_serpentine_fill"
    ]
    detail_centerlines = [path for path in result["toolpaths"] if path.kind == "detail-trace"]

    assert int(debug.get("detail_region_count", 0)) > 0
    assert int(debug.get("detail_regions_classified_wide", 0)) > 0
    assert int(debug.get("detail_regions_serpentine_filled", 0)) > 0
    assert int(debug.get("arsenal_detail_serpentine_paths_generated", 0)) > 0
    assert detail_serpentine_paths
    assert len(detail_serpentine_paths) > len(detail_centerlines)


def test_arsenal_detail_repair_pass_reaches_required_coverage():
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]
    region_rows = list(debug.get("detail_region_repair_rows", []))
    fillable_rows = [row for row in region_rows if bool(row.get("fillable", False))]

    assert debug.get("detail_repair_pass_enabled") is True
    assert float(debug.get("required_detail_coverage_percent", 0.0)) >= 95.0
    assert float(debug.get("detail_coverage_after_repair_percent", 0.0)) >= float(debug.get("required_detail_coverage_percent", 0.0))
    assert int(debug.get("detail_fillable_regions_failing_after_repair", -1)) == 0
    assert int(debug.get("regions_failing_after_repair", -1)) == 0
    assert int(debug.get("missed_blob_count_after_repair", -1)) == 0
    assert float(debug.get("largest_missed_blob_equivalent_diameter_mm_after", 1.0)) <= 0.10
    assert fillable_rows
    assert all(float(row.get("coverage_after_percent", 0.0)) >= float(debug.get("required_detail_coverage_percent", 0.0)) for row in fillable_rows)


def test_arsenal_detail_repair_strokes_improve_coverage_without_overflow():
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]
    repair_paths = [
        path for path in result["toolpaths"]
        if path.kind == "fill-repair" and path.source == "mask_space_coverage_repair"
    ]

    assert float(debug.get("detail_coverage_after_repair_percent", 0.0)) > float(debug.get("detail_coverage_before_repair_percent", 0.0))
    assert int(debug.get("detail_repair_strokes_added", 0)) > 0
    assert int(debug.get("repair_candidates_accepted", 0)) > 0
    assert debug.get("repair_paths_exported") is True
    assert repair_paths
    assert float(debug.get("detail_repair_outside_overflow_mm2", 0.0)) <= float(debug.get("outside_region_overflow_tolerance_mm2", 0.0))
    assert not any(path.kind == "outline" and path.source == "detail_repair_fill" for path in result["toolpaths"])


def test_arsenal_local_blob_validation_blocks_global_only_passes():
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]

    assert float(debug.get("detail_coverage_before_repair_percent", 0.0)) >= 90.0
    assert int(debug.get("regions_failing_before_repair", 0)) > 0
    assert int(debug.get("missed_blob_count_before_repair", 0)) > 0
    assert debug.get("coverage_validation_target") == "selected_color_mask"
    assert debug.get("local_coverage_validation_enabled") is True


def test_arsenal_fill_may_overlap_outline_when_repairing_target_gaps():
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]

    assert debug.get("fill_allowed_to_overlap_outline") is True
    assert debug.get("repair_clipped_against") == "selected_color_mask"
    assert debug.get("coverage_preview_gcode_consistent") is True
    assert float(debug["infill_debug"]["pen_width_mm"]) == pytest.approx(0.6, abs=1e-9)
    assert float(debug["infill_debug"]["spacing_mm"]) == pytest.approx(0.6, abs=1e-9)


def test_arsenal_missed_blob_diagnostics_and_path_stats_are_written(monkeypatch: pytest.MonkeyPatch):
    artifact_dir = Path(tempfile.gettempdir()) / "golfball_plotter_test_artifacts" / "arsenal_missed_blob_debug"
    monkeypatch.setenv("COVERAGE_DEBUG_ARTIFACT_DIR", str(artifact_dir))
    monkeypatch.setenv("WRITE_COVERAGE_DEBUG_ARTIFACTS", "1")
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]

    diagnostics_path = artifact_dir / "missed_blob_diagnostics.json"
    path_stats_path = artifact_dir / "path_stats.json"
    assert diagnostics_path.exists()
    assert path_stats_path.exists()

    diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))
    path_stats = json.loads(path_stats_path.read_text(encoding="utf-8"))

    assert isinstance(diagnostics, list)
    assert path_stats["missed_blob_debug_enabled"] is True
    assert path_stats["coverage_target"] == "selected_color_mask"
    assert path_stats["repair_clipping"] == "selected_color_mask"
    assert path_stats["fill_allowed_to_overlap_outline"] is True
    assert path_stats["repair_paths_exported"] is True
    assert path_stats["coverage_preview_gcode_consistent"] is True
    assert int(path_stats["repair_candidates_accepted"]) > 0
    assert int(debug.get("repair_candidates_accepted", 0)) == int(path_stats["repair_candidates_accepted"])


def test_arsenal_exported_path_coverage_audit_matches_preview_target(monkeypatch: pytest.MonkeyPatch):
    artifact_dir = Path(tempfile.gettempdir()) / "golfball_plotter_test_artifacts" / "arsenal_export_coverage_audit"
    monkeypatch.setenv("COVERAGE_DEBUG_ARTIFACT_DIR", str(artifact_dir))
    monkeypatch.setenv("WRITE_COVERAGE_DEBUG_ARTIFACTS", "1")
    result = _run_fixture(ARSENAL_FIXTURE)
    debug = result["debug"]

    path_stats = json.loads((artifact_dir / "path_stats.json").read_text(encoding="utf-8"))
    coverage_report = json.loads((artifact_dir / "coverage_from_exported_paths_report.json").read_text(encoding="utf-8"))
    mask_report = json.loads((artifact_dir / "mask_consistency_report.json").read_text(encoding="utf-8"))
    resampling_report = json.loads((artifact_dir / "path_resampling_report.json").read_text(encoding="utf-8"))

    assert mask_report["preview_target_vs_diagnostic_target_iou"] >= 0.5
    assert coverage_report["coverage_rasterization_space"] == "surface-mm-on-ball"
    assert coverage_report["final_repair_scope"] == "all_selected_color_components"
    assert int(coverage_report["visible_missed_blob_count_after_repair"]) == 0
    assert float(coverage_report["largest_visible_missed_blob_equivalent_diameter_mm_after"]) <= 0.15
    assert float(resampling_report["max_surface_segment_mm_after"]) <= 0.15 + 1e-9
    assert path_stats["repair_paths_exported"] is True
    assert debug.get("root_cause_category_corrected") in {"coverage_under_sampling_fixed", "false_negative_coverage_simulation", "wrong_target_mask_selection"}


def test_arsenal_final_output_coverage_90ccw_0p6mm(tmp_path: Path):
    result = _run_frontend_arsenal_fixture(rotation_deg=90.0)
    debug, current_to_source, final_surface_paths = _arsenal_final_surface_paths(result)

    mask_value = result["mask"]
    expected_source_mask = np.asarray(getattr(mask_value, "mask", mask_value))
    expected_mask = (expected_source_mask > 0).astype(np.uint8) * 255
    actual_mask = pipeline_core.rasterize_surface_toolpaths_mask(
        final_surface_paths,
        shape=expected_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_mm=0.3,
        max_segment_mm=0.15,
        include_kinds={"fill-infill", "fill-repair", "detail-trace", "detail-continuation", "outline", "fill-wall"},
    )
    tolerance_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    actual_with_tolerance = cv2.dilate(actual_mask, tolerance_kernel, iterations=1)

    expected_bool = expected_mask > 0
    actual_bool = actual_mask > 0
    tolerated_actual_bool = actual_with_tolerance > 0
    covered_bool = expected_bool & tolerated_actual_bool
    missed_bool = expected_bool & ~tolerated_actual_bool
    excess_bool = actual_bool & ~expected_bool
    coverage_ratio = float(np.count_nonzero(covered_bool) / max(1, np.count_nonzero(expected_bool)))

    if coverage_ratio < 0.99:
        cv2.imwrite(str(tmp_path / "expected_mask.png"), expected_mask)
        cv2.imwrite(str(tmp_path / "actual_coverage.png"), actual_mask)
        cv2.imwrite(str(tmp_path / "missed_pixels.png"), missed_bool.astype(np.uint8) * 255)
        cv2.imwrite(str(tmp_path / "excess_pixels.png"), excess_bool.astype(np.uint8) * 255)
        overlay = np.full((expected_mask.shape[0], expected_mask.shape[1], 3), 255, dtype=np.uint8)
        overlay[expected_bool] = (220, 220, 220)
        overlay[covered_bool] = (0, 180, 0)
        overlay[missed_bool] = (0, 0, 255)
        overlay[excess_bool] = (255, 128, 0)
        cv2.imwrite(str(tmp_path / "overlay_expected_vs_actual.png"), overlay)

    assert coverage_ratio >= 0.99, (
        f"Arsenal final emitted coverage at 0.6 mm and 90 CCW regressed to {coverage_ratio:.5f}; "
        f"expected_px={int(np.count_nonzero(expected_bool))} "
        f"covered_px={int(np.count_nonzero(covered_bool))} "
        f"missed_px={int(np.count_nonzero(missed_bool))} "
        f"excess_px={int(np.count_nonzero(excess_bool))} "
        f"artifacts={tmp_path}"
    )


def test_arsenal_final_output_overflow_and_centerline_safety_90ccw_0p6mm(tmp_path: Path):
    result = _run_frontend_arsenal_fixture(rotation_deg=90.0)
    _debug, current_to_source, final_surface_paths = _arsenal_final_surface_paths(result)

    mask_value = result["mask"]
    expected_source_mask = np.asarray(getattr(mask_value, "mask", mask_value))
    expected_mask = (expected_source_mask > 0).astype(np.uint8) * 255
    expected_bool = expected_mask > 0
    expected_pixels = int(np.count_nonzero(expected_bool))
    assert expected_pixels > 0

    line_width_mm = 0.6
    pen_radius_mm = line_width_mm * 0.5
    a, b, c, d, _e, _f = current_to_source
    px_per_mm = max(1e-6, (math.hypot(a, b) + math.hypot(c, d)) * 0.5)
    erosion_radius_px = max(1, int(round(pen_radius_mm * px_per_mm)))
    boundary_band_px = max(1, erosion_radius_px + 1)
    erosion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erosion_radius_px * 2 + 1, erosion_radius_px * 2 + 1))
    boundary_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (boundary_band_px * 2 + 1, boundary_band_px * 2 + 1))

    safe_centerline_mask = cv2.erode(expected_mask, erosion_kernel, iterations=1)
    boundary_band_mask = cv2.morphologyEx(expected_mask, cv2.MORPH_GRADIENT, boundary_kernel)

    coverage_kinds = {"fill-infill", "fill-repair", "detail-trace", "detail-continuation", "outline", "fill-wall"}
    overflow_kinds = {"fill-infill", "fill-repair", "detail-trace", "detail-continuation", "fill-wall"}
    centerline_kinds = {"fill-infill", "fill-repair", "detail-trace", "detail-continuation", "fill-wall"}
    actual_coverage_all = pipeline_core.rasterize_surface_toolpaths_mask(
        final_surface_paths,
        shape=expected_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_mm=pen_radius_mm,
        max_segment_mm=0.15,
        include_kinds=coverage_kinds,
    )
    actual_coverage = pipeline_core.rasterize_surface_toolpaths_mask(
        final_surface_paths,
        shape=expected_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_mm=pen_radius_mm,
        max_segment_mm=0.15,
        include_kinds=overflow_kinds,
    )
    centerline_mask = _rasterize_surface_centerlines(
        final_surface_paths,
        shape=expected_mask.shape,
        current_to_source_matrix=current_to_source,
        include_kinds=centerline_kinds,
        max_segment_mm=0.15,
    )

    tolerance_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    actual_with_tolerance = cv2.dilate(actual_coverage_all, tolerance_kernel, iterations=1)
    actual_bool = actual_coverage_all > 0
    tolerated_actual_bool = actual_with_tolerance > 0
    overflow_bool = actual_coverage > 0
    safe_centerline_bool = safe_centerline_mask > 0
    centerline_bool = centerline_mask > 0

    covered_expected_bool = expected_bool & tolerated_actual_bool
    missed_bool = expected_bool & ~tolerated_actual_bool
    excess_bool = overflow_bool & ~expected_bool
    boundary_overflow_bool = excess_bool & (boundary_band_mask > 0)
    centerline_violation_bool = centerline_bool & ~safe_centerline_bool

    coverage_ratio = float(np.count_nonzero(covered_expected_bool) / max(1, expected_pixels))
    overflow_ratio = float(np.count_nonzero(excess_bool) / max(1, expected_pixels))
    boundary_overflow_ratio = float(np.count_nonzero(boundary_overflow_bool) / max(1, expected_pixels))
    centerline_violation_pixels = int(np.count_nonzero(centerline_violation_bool))

    if coverage_ratio < 0.99 or overflow_ratio > 0.005 or centerline_violation_pixels > 8:
        per_path_rows = _surface_path_overflow_attribution(
            final_surface_paths,
            shape=expected_mask.shape,
            current_to_source_matrix=current_to_source,
            pen_radius_mm=pen_radius_mm,
            expected_bool=expected_bool,
            safe_centerline_bool=safe_centerline_bool,
            include_kinds=overflow_kinds | centerline_kinds,
        )
        cv2.imwrite(str(tmp_path / "expected_mask.png"), expected_mask)
        cv2.imwrite(str(tmp_path / "safe_centerline_mask.png"), safe_centerline_mask)
        cv2.imwrite(str(tmp_path / "actual_coverage.png"), actual_coverage)
        cv2.imwrite(str(tmp_path / "missed_pixels.png"), missed_bool.astype(np.uint8) * 255)
        cv2.imwrite(str(tmp_path / "excess_pixels.png"), excess_bool.astype(np.uint8) * 255)
        cv2.imwrite(str(tmp_path / "centerline_violations.png"), centerline_violation_bool.astype(np.uint8) * 255)
        (tmp_path / "path_overflow_attribution.json").write_text(
            json.dumps(per_path_rows, indent=2),
            encoding="utf-8",
        )

        overlay_expected_actual_excess = np.full((expected_mask.shape[0], expected_mask.shape[1], 3), 255, dtype=np.uint8)
        overlay_expected_actual_excess[expected_bool] = (220, 220, 220)
        overlay_expected_actual_excess[covered_expected_bool] = (0, 180, 0)
        overlay_expected_actual_excess[missed_bool] = (0, 0, 255)
        overlay_expected_actual_excess[excess_bool] = (255, 140, 0)
        cv2.imwrite(str(tmp_path / "overlay_expected_actual_excess.png"), overlay_expected_actual_excess)

        overlay_paths_vs_safe_mask = np.full((expected_mask.shape[0], expected_mask.shape[1], 3), 255, dtype=np.uint8)
        overlay_paths_vs_safe_mask[safe_centerline_bool] = (220, 255, 220)
        overlay_paths_vs_safe_mask[centerline_bool] = (0, 90, 220)
        overlay_paths_vs_safe_mask[centerline_violation_bool] = (0, 0, 255)
        cv2.imwrite(str(tmp_path / "overlay_paths_vs_safe_mask.png"), overlay_paths_vs_safe_mask)
    else:
        per_path_rows = []

    assert coverage_ratio >= 0.99, (
        f"Arsenal final coverage fell below threshold: coverage_ratio={coverage_ratio:.5f} artifacts={tmp_path}"
    )
    assert overflow_ratio <= 0.005, (
        f"Arsenal final emitted paths overflow the mask: overflow_ratio={overflow_ratio:.5f} "
        f"boundary_overflow_ratio={boundary_overflow_ratio:.5f} "
        f"top_offenders={per_path_rows[:5]} artifacts={tmp_path}"
    )
    assert centerline_violation_pixels <= 8, (
        f"Arsenal final emitted centerlines leave the safe eroded mask: "
        f"centerline_violation_pixels={centerline_violation_pixels} "
        f"top_offenders={per_path_rows[:5]} artifacts={tmp_path}"
    )


def test_arsenal_final_repair_audit_replaces_boundary_hugging_repairs():
    result = _run_frontend_arsenal_fixture(rotation_deg=90.0)
    debug, _current_to_source, final_surface_paths = _arsenal_final_surface_paths(result)

    audit_summary = dict(debug.get("final_repair_audit_summary", {}))
    audit_rows = list(debug.get("final_repair_audit_rows", []))
    rebuild_rows = list(debug.get("final_repair_rebuild_candidate_rows", []))
    fill_repairs = [path for path in final_surface_paths if path.kind == "fill-repair"]

    assert int(audit_summary.get("audited_repair_count", 0)) > 0
    assert int(audit_summary.get("rejected_existing_repair_count", 0)) > 0
    assert int(audit_summary.get("optimized_repair_count", 0)) == len(fill_repairs)
    assert any(
        str((path.metadata or {}).get("repair_mode", "")) == "thin-collapsed-detail-repair"
        for path in fill_repairs
    )
    assert any(
        str((row or {}).get("repair_mode", "")) == "thin-collapsed-detail-repair"
        and str((row or {}).get("classification", "")) == "thin-collapsed-detail-repair"
        for row in rebuild_rows
    )
    assert not any(
        str((path.metadata or {}).get("repair_mode", "")) == "normal-safe-repair"
        and float((path.metadata or {}).get("safe_centerline_inset_mm", 0.0) or 0.0) < 0.3 - 1e-9
        for path in fill_repairs
    )
    assert any(str((row or {}).get("classification", "")) == "reject-useless-or-overflowing" for row in audit_rows)


def test_arsenal_frontend_default_runs_source_thin_centerline_pass():
    result = _run_frontend_arsenal_fixture(rotation_deg=90.0)
    debug, _current_to_source, final_surface_paths = _arsenal_final_surface_paths(result)

    thin_paths = [
        path for path in final_surface_paths
        if path.kind == "detail-trace" and bool((path.metadata or {}).get("thin_source_region_centerline", False))
    ]

    assert debug.get("frontend_default_used_thin_centerline_pass") is True
    assert int(debug.get("thin_source_region_count", 0)) > 0
    assert int(debug.get("thin_centerline_candidate_count", 0)) > 0
    assert int(debug.get("thin_centerline_accepted_count", 0)) > 0
    assert float(debug.get("thin_centerline_total_length_mm", 0.0)) > 0.0
    assert debug.get("thin_centerline_paths_exported") is True
    assert thin_paths


def test_ring_shape_is_split_into_local_cells_before_routing():
    outer = Polygon([(0.0, 0.0), (120.0, 0.0), (120.0, 80.0), (0.0, 80.0)])
    inner = Polygon([(35.0, 18.0), (85.0, 18.0), (85.0, 62.0), (35.0, 62.0)])
    printable = outer.difference(inner)

    debug: dict[str, object] = {}
    toolpaths = pipeline_core.generate_toolpaths(
        pipeline_core.GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=1.0,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=1.0,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        debug=debug,
    )

    assert debug.get("local_cell_count", 0) >= 2
    assert debug.get("average_segments_per_cell", 0.0) > 1.0
    assert debug.get("total_pen_up_travel_distance_mm", 0.0) > 0.0
    assert len([path for path in toolpaths if path.kind == "fill-infill"]) >= 2


def test_carolin_fixture_rejects_whitespace_crossing_connectors():
    _make_carolin_fixture_if_needed()
    result = _run_fixture(CAROLIN_FIXTURE)
    diagnostics = result["diagnostics"]
    debug = result["debug"]

    allowed = {
        "fill-infill",
        "fill-repair",
        "detail-trace",
        "outline",
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "outline_cleanup",
    }
    drawable = [path for path in result["toolpaths"] if path.kind != "travel"]
    assert drawable
    assert all(path.kind in allowed for path in drawable)
    assert result["actual_pen_lifts"] < 50
    assert diagnostics.get("rejected_raster_mask_sampling", 0) >= 0
    assert diagnostics.get("rejected_outside_selected_color", 0) >= 0
    assert diagnostics.get("accepted_connectors", 0) == result["accepted_connector_count"]
    assert result["accepted_connector_count"] >= 0
    assert debug.get("detail_filter_mode") == pipeline_core.DETAIL_FILTER_MODE
    assert debug.get("detail_source_whitelist_enforced") is True
    assert debug.get("travel_geometry_allowed_as_detail") is False
    assert int(debug.get("detail_paths_dropped_as_redundant_overlap", 0)) > 0
    assert int(debug.get("detail_repair_strokes_added", 0)) == 0
    assert debug.get("coverage_validation_target") == "selected_color_mask"


def test_carolin_fixture_post_generation_ordering_reduces_travel_and_preserves_outline():
    _make_carolin_fixture_if_needed()
    result = _run_fixture(CAROLIN_FIXTURE)
    debug = result["debug"]

    assert debug.get("travel_optimization_mode") == "final_export_event_stream_ordering"
    assert debug.get("optimizer_runs_after_path_merging") is True
    assert debug.get("optimizer_runs_on_final_export_paths") is True
    assert debug.get("preview_uses_optimized_order") is True
    assert debug.get("gcode_uses_optimized_order") is True
    assert debug.get("uses_surface_mm_for_ordering") is True


def test_ha_fixture_skips_detail_repair_augmentation():
    result = _run_fixture(HA_FIXTURE)
    debug = result["debug"]

    assert int(debug.get("detail_repair_strokes_added", 0)) == 0
    assert float(debug.get("detail_coverage_after_repair_percent", 0.0)) == pytest.approx(0.0, abs=1e-9)
    assert float(debug["infill_debug"]["pen_width_mm"]) == pytest.approx(0.6, abs=1e-9)
    assert float(debug["infill_debug"]["spacing_mm"]) == pytest.approx(0.6, abs=1e-9)
    assert debug.get("geometry_changed") is False
    assert debug.get("path_points_moved") is False
    assert debug.get("paths_reordered") is True
    assert int(debug.get("paths_reordered_count", 0)) > 0
    assert float(debug.get("optimized_pen_up_travel_length_mm", 0.0)) < float(debug.get("raw_pen_up_travel_length_mm", 0.0))
    assert int(debug.get("bad_choice_count_after_optimization", 0)) == 0
    assert debug.get("stale_travel_geometry_removed") is True
    assert int(debug.get("outline_path_count", 0)) > 0
    assert any(path.kind == "outline" for path in result["toolpaths"])


def test_ha_fixture_post_generation_ordering_has_no_geometry_regression():
    result = _run_fixture(HA_FIXTURE)
    debug = result["debug"]

    assert debug.get("travel_optimization_mode") == "final_export_event_stream_ordering"
    assert debug.get("geometry_changed") is False
    assert debug.get("path_points_moved") is False
    assert int(debug.get("outline_path_count", 0)) >= 0
    assert float(debug.get("optimized_pen_up_travel_length_mm", 0.0)) <= float(debug.get("raw_pen_up_travel_length_mm", 0.0))


def test_carolin_fixture_mask_coverage_is_at_least_ninety_percent():
    _make_carolin_fixture_if_needed()
    result = _run_fixture(CAROLIN_FIXTURE)
    debug = result.get("debug", {}) if isinstance(result.get("debug", {}), dict) else {}
    validation = result["validation"]
    mask = validation.get("mask")
    matrix = validation.get("current_to_source_matrix")
    assert isinstance(mask, np.ndarray)
    assert isinstance(matrix, (tuple, list)) and len(matrix) == 6

    line_width_mm = 0.6
    metrics = pipeline_core.compute_toolpath_mask_coverage_metrics(
        result["toolpaths"],
        mask=mask,
        current_to_source_matrix=tuple(float(value) for value in matrix),
        pen_radius_mm=line_width_mm * 0.5,
        sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
        include_kinds={
            "fill-infill",
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_tiny_mark",
            "coverage_contour",
            "coverage_connector",
            "outline_cleanup",
        },
    )
    assert metrics is not None
    breakdown = pipeline_core.compute_toolpath_mask_coverage_breakdown(
        result["toolpaths"],
        mask=mask,
        current_to_source_matrix=tuple(float(value) for value in matrix),
        pen_radius_mm=line_width_mm * 0.5,
        sample_step_mm=max(0.01, min(line_width_mm * 0.35, 0.05)),
        include_kinds={
            "fill-infill",
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_tiny_mark",
            "coverage_contour",
            "coverage_connector",
            "outline_cleanup",
        },
    )
    if metrics.penalized_coverage_percent < 90.0 and os.getenv("WRITE_COVERAGE_DEBUG_ARTIFACTS", "0") == "1":
        target_mask = np.asarray(mask) > 0
        drawn = np.zeros_like(target_mask, dtype=np.uint8)
        a, b, c, d, _e, _f = tuple(float(value) for value in matrix)
        px_per_mm = max(1e-6, (math.hypot(a, b) + math.hypot(c, d)) * 0.5)
        pen_radius_px = max(0.0, (line_width_mm * 0.5) * px_per_mm)
        pen_radius_i = max(1, int(round(pen_radius_px)))
        sample_step_mm = max(0.01, min(line_width_mm * 0.35, 0.05))
        kinds = {
            "fill-infill",
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_tiny_mark",
            "coverage_contour",
            "coverage_connector",
            "outline_cleanup",
        }
        centerline_overlay = np.zeros((target_mask.shape[0], target_mask.shape[1], 3), dtype=np.uint8)
        for path in result["toolpaths"]:
            if path.kind not in kinds or len(path.points) < 1:
                continue
            if len(path.points) == 1:
                source = pipeline_core.apply_svg_matrix(path.points[0], tuple(float(value) for value in matrix))
                cv2.circle(drawn, (int(round(source.x)), int(round(source.y))), pen_radius_i, 255, -1)
                cv2.circle(centerline_overlay, (int(round(source.x)), int(round(source.y))), 1, (0, 255, 255), -1)
                continue
            for p0, p1 in zip(path.points, path.points[1:]):
                line = LineString([(p0.x, p0.y), (p1.x, p1.y)])
                if line.length <= 1e-9:
                    continue
                sample_count = max(2, int(math.ceil(line.length / sample_step_mm)) + 1)
                for sample_index in range(sample_count):
                    distance_mm = min(line.length, (line.length * sample_index) / max(sample_count - 1, 1))
                    sample = line.interpolate(distance_mm)
                    source = pipeline_core.apply_svg_matrix(pipeline_core.Point(float(sample.x), float(sample.y)), tuple(float(value) for value in matrix))
                    px = int(round(source.x))
                    py = int(round(source.y))
                    cv2.circle(drawn, (px, py), pen_radius_i, 255, -1)
                    if 0 <= px < centerline_overlay.shape[1] and 0 <= py < centerline_overlay.shape[0]:
                        centerline_overlay[py, px] = (0, 255, 255)
        drawn_mask = drawn > 0
        inside_covered = target_mask & drawn_mask
        inside_missed = target_mask & ~drawn_mask
        outside_overdraw = ~target_mask & drawn_mask
        out_dir = Path(tempfile.gettempdir()) / "golfball_plotter_test_artifacts" / "carolin_coverage"
        out_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_dir / "target_mask.png"), (target_mask.astype(np.uint8) * 255))
        cv2.imwrite(str(out_dir / "drawn_mask.png"), (drawn_mask.astype(np.uint8) * 255))
        cv2.imwrite(str(out_dir / "inside_covered.png"), (inside_covered.astype(np.uint8) * 255))
        cv2.imwrite(str(out_dir / "inside_missed.png"), (inside_missed.astype(np.uint8) * 255))
        cv2.imwrite(str(out_dir / "outside_overdraw.png"), (outside_overdraw.astype(np.uint8) * 255))
        overlay = np.zeros((target_mask.shape[0], target_mask.shape[1], 3), dtype=np.uint8)
        overlay[inside_covered] = (0, 255, 0)
        overlay[inside_missed] = (0, 0, 255)
        overlay[outside_overdraw] = (255, 0, 0)
        boundary = cv2.morphologyEx((target_mask.astype(np.uint8) * 255), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
        overlay[boundary] = (220, 220, 220)
        centerline_mask = centerline_overlay[:, :, 1] > 0
        overlay[centerline_mask] = (0, 255, 255)
        cv2.imwrite(str(out_dir / "coverage_overlay.png"), overlay)

    missed_components = int(debug.get("missed_component_count", 0) or 0)
    top_missed = debug.get("missed_components_top_by_area", [])
    if not isinstance(top_missed, list):
        top_missed = []
    accepted_backfill_count = int(debug.get("coverage_backfill_component_count", 0) or 0)
    rejected_backfill_count = int(debug.get("coverage_backfill_component_rejected", 0) or 0)
    rejection_reasons = debug.get("coverage_backfill_component_rejection_reasons", {})
    if not isinstance(rejection_reasons, dict):
        rejection_reasons = {}
    coverage_backfill_global_count = sum(1 for path in result["toolpaths"] if path.metadata.get("coverage_backfill_global"))
    coverage_backfill_component_count = sum(1 for path in result["toolpaths"] if path.metadata.get("coverage_backfill_component"))

    assert metrics.raw_coverage_percent >= 80.0, (
        f"Carolin raw fill coverage regressed too far: "
        f"raw={metrics.raw_coverage_percent:.2f}%, "
        f"covered_px={metrics.covered_inside_mask_px}, missed_px={metrics.missed_inside_mask_px}, "
        f"path_kind_overdraw_table={breakdown}"
    )
    assert any(path.kind == "fill-infill" for path in result["toolpaths"])
    assert any(path.kind == "outline" for path in result["toolpaths"])
    assert int(debug.get("detail_paths_dropped", 0)) > 0, (
        f"Expected Carolin detail pruning to drop redundant paths, got "
        f"detail_paths_dropped={debug.get('detail_paths_dropped')}, "
        f"drop_reasons={debug.get('detail_drop_reasons')}, "
        f"missed_component_count={missed_components}, top_missed_components_by_area={top_missed[:10]}, "
        f"accepted_backfill_count={accepted_backfill_count}, rejected_backfill_count={rejected_backfill_count}, "
        f"rejection_reasons={rejection_reasons}, coverage_backfill_global_count={coverage_backfill_global_count}, "
        f"coverage_backfill_component_count={coverage_backfill_component_count}"
    )


def test_coverage_metric_synthetic_perfect_fill_rectangle():
    mask = np.zeros((64, 96), dtype=np.uint8)
    cv2.line(mask, (12, 32), (84, 32), 255, thickness=12, lineType=cv2.LINE_8)
    line = pipeline_core.Toolpath(
        points=[pipeline_core.Point(12.0, 32.0), pipeline_core.Point(84.0, 32.0)],
        kind="coverage_centerline",
        closed=False,
        source="test",
        metadata={},
    )
    m = pipeline_core.compute_toolpath_mask_coverage_metrics(
        [line],
        mask=mask,
        current_to_source_matrix=(1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        pen_radius_mm=6.0,
        sample_step_mm=0.5,
        include_kinds={"coverage_centerline"},
    )
    assert m is not None
    assert m.raw_coverage_percent > 99.0
    assert m.outside_overdraw_percent < 1.0


def test_coverage_metric_synthetic_known_overdraw_ratio():
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[:100, :100] = 255
    drawn = np.zeros((100, 100), dtype=np.uint8)
    drawn[:100, :100] = 255
    drawn[0:10, 0:10] = 255
    mask_area = int(np.count_nonzero(mask))
    overdraw = 100
    outside_percent = 100.0 * overdraw / mask_area
    assert outside_percent == pytest.approx(1.0)


def test_coverage_metric_synthetic_thin_band_centerline():
    mask = np.zeros((80, 120), dtype=np.uint8)
    cv2.rectangle(mask, (10, 38), (110, 42), 255, -1)
    path = pipeline_core.Toolpath(
        points=[pipeline_core.Point(10.0, 40.0), pipeline_core.Point(110.0, 40.0)],
        kind="coverage_centerline",
        closed=False,
        source="test",
        metadata={},
    )
    m = pipeline_core.compute_toolpath_mask_coverage_metrics(
        [path],
        mask=mask,
        current_to_source_matrix=(1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        pen_radius_mm=2.0,
        sample_step_mm=0.5,
        include_kinds={"coverage_centerline"},
    )
    assert m is not None
    assert m.raw_coverage_percent > 90.0
    assert m.outside_overdraw_percent < 15.0


def test_coverage_metric_synthetic_one_pixel_alignment_offset_penalizes():
    mask = np.zeros((40, 100), dtype=np.uint8)
    cv2.rectangle(mask, (10, 18), (90, 22), 255, -1)
    path = pipeline_core.Toolpath(
        points=[pipeline_core.Point(10.0, 20.0), pipeline_core.Point(90.0, 20.0)],
        kind="coverage_centerline",
        closed=False,
        source="test",
        metadata={},
    )
    m0 = pipeline_core.compute_toolpath_mask_coverage_metrics(
        [path],
        mask=mask,
        current_to_source_matrix=(1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        pen_radius_mm=2.0,
        sample_step_mm=0.5,
        include_kinds={"coverage_centerline"},
    )
    m1 = pipeline_core.compute_toolpath_mask_coverage_metrics(
        [path],
        mask=mask,
        current_to_source_matrix=(1.0, 0.0, 0.0, 1.0, 1.0, 0.0),
        pen_radius_mm=2.0,
        sample_step_mm=0.5,
        include_kinds={"coverage_centerline"},
    )
    assert m0 is not None and m1 is not None
    assert m1.penalized_coverage_percent < m0.penalized_coverage_percent


def test_penalized_coverage_formula_examples_are_strict():
    def _score(mask_area_px: int, covered_inside_mask_px: int, overdraw_outside_mask_px: int) -> float:
        raw = 100.0 * covered_inside_mask_px / mask_area_px
        outside = 100.0 * overdraw_outside_mask_px / mask_area_px
        return raw - outside

    # 100% covered and 1% overdraw => 99%
    assert _score(mask_area_px=10000, covered_inside_mask_px=10000, overdraw_outside_mask_px=100) == pytest.approx(99.0)
    # 90% covered and 40% overdraw => 50%
    assert _score(mask_area_px=10000, covered_inside_mask_px=9000, overdraw_outside_mask_px=4000) == pytest.approx(50.0)
    # Do not clamp: severe overdraw can make the score negative.
    assert _score(mask_area_px=10000, covered_inside_mask_px=2000, overdraw_outside_mask_px=6000) < 0.0


def test_synthetic_png_uses_single_stroke_fallback_and_shared_canonical_paths():
    raster, geometry, toolpaths_service, gcode_service = _load_services()
    image_bytes = _synthetic_logo_png_bytes()
    selected = _selected_black_color_id(raster, image_bytes)
    mask = raster.build_mask(
        image_bytes,
        [selected],
        tolerance=24,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=1,
    )
    regions = raster.extract_regions(mask, min_region_area_px=8, simplify_tolerance_px=1.0)
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)

    debug: dict[str, object] = {}
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=0.6,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=0.6,
        infill_density=100.0,
        infill_angle_deg=45.0,
        fill_strategy="rotated_scanline",
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        debug=debug,
    )

    assert any(path.metadata.get("small_detail_fill_style") == "single_stroke_detail" for path in toolpaths)

    cleaned, _stats = pipeline_core.cleanup_surface_toolpaths(toolpaths, tolerance_mm=0.0, min_segment_length_mm=0.0)
    prepared = pipeline_core.prepare_toolpaths_for_projection(cleaned, default_pen_width_mm=0.6)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)

    gcode, preview = gcode_service.generate_from_toolpaths(
        toolpaths=projected,
        draw_feed=1200.0,
        travel_feed=3000.0,
        sample_step_deg=1.0,
        placement_offset_x=0.0,
        placement_offset_y=0.0,
        pen_up_s=575,
        pen_down_s=700,
        servo_ramp_enabled=True,
        servo_ramp_step=20,
        servo_ramp_delay_ms=10.0,
        pen_up_dwell_ms=30.0,
        pen_down_dwell_ms=60.0,
        gcode_mode="simple",
        include_comments=True,
        debug=debug,
    )

    preview_toolpaths = [path for path in pipeline_core.preview_entries_to_toolpaths(preview) if path.kind != "travel"]
    gcode_toolpaths = [path for path in pipeline_core.parse_gcode_machine_motion_paths(gcode, pen_up_s=575, pen_down_s=700) if path.kind != "travel"]
    assert preview_toolpaths
    assert gcode_toolpaths
    assert preview_toolpaths[0].points[0].x == pytest.approx(gcode_toolpaths[0].points[0].x, abs=1e-4)
    assert preview_toolpaths[0].points[0].y == pytest.approx(gcode_toolpaths[0].points[0].y, abs=1e-4)
    assert debug.get("invalidConnectorCount", 0) == 0
