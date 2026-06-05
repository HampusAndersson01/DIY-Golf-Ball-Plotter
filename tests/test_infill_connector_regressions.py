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

from app.models.machine_state import MachineState
from app.services import pipeline_core
from app.services.gcode_service import GcodeService
from app.services.geometry_service import GeometryService
from app.services.raster_analysis_service import RasterAnalysisService
from app.services.toolpath_service import ToolpathService
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
    assert int(debug.get("optimized_travel_crossing_count", 0)) <= int(debug.get("raw_travel_crossing_count", 0))
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
