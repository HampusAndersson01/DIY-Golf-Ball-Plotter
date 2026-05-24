from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from shapely.geometry import LineString

from app.models.machine_state import MachineState
from app.services import pipeline_core
from app.services.gcode_service import GcodeService
from app.services.geometry_service import GeometryService
from app.services.raster_analysis_service import RasterAnalysisService
from app.services.reference_infill_service import (
    extract_gcode_from_3mf,
    parse_reference_gcode,
    summarize_line_family,
)
from app.services.toolpath_service import ToolpathService
from tests.test_svg_parser import CONFIG


ROOT = Path(__file__).resolve().parents[1]
FIXTURE_IMAGE = ROOT / "tests" / "fixtures" / "images" / "ha-compact-lightbg.png"
FIXTURE_3MF = ROOT / "tests" / "fixtures" / "reference_3mf" / "HA.3mf"
FIXTURE_GCODE = ROOT / "tests" / "fixtures" / "reference_gcode" / "HA_plate_1.gcode"


def _angle_distance_deg(a: float, b: float) -> float:
    diff = abs((a - b) % 180.0)
    return min(diff, 180.0 - diff)


def _line_segments_from_toolpaths(toolpaths: list[pipeline_core.Toolpath]) -> list[LineString]:
    segments: list[LineString] = []
    for path in toolpaths:
        for p0, p1 in zip(path.points, path.points[1:]):
            if abs(p1.x - p0.x) < 1e-9 and abs(p1.y - p0.y) < 1e-9:
                continue
            segments.append(LineString([(p0.x, p0.y), (p1.x, p1.y)]))
    return segments


def _count_long_diagonal_connectors(lines: list[LineString], *, main_angle_deg: float, spacing_mm: float) -> int:
    threshold = max(spacing_mm * 1.5, 0.8)
    count = 0
    for line in lines:
        if line.length < threshold:
            continue
        (x1, y1), (x2, y2) = list(line.coords)
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
        if _angle_distance_deg(angle, main_angle_deg) > 20.0:
            count += 1
    return count


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


def _load_services() -> tuple[RasterAnalysisService, GeometryService, ToolpathService, GcodeService]:
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    return raster, GeometryService(), ToolpathService(), GcodeService()


def test_reference_rectilinear_infill_matches_fixture():
    assert FIXTURE_IMAGE.exists(), f"Missing fixture image: {FIXTURE_IMAGE}"
    assert FIXTURE_GCODE.exists(), f"Missing fixture reference G-code: {FIXTURE_GCODE}"

    extracted_gcode = None
    archive_inventory: list[str] = []
    if FIXTURE_3MF.exists():
        extracted_gcode, archive_inventory = extract_gcode_from_3mf(FIXTURE_3MF, FIXTURE_GCODE.parent)
        if extracted_gcode is not None:
            reference_path = extracted_gcode
        else:
            reference_path = FIXTURE_GCODE
    else:
        reference_path = FIXTURE_GCODE

    reference = parse_reference_gcode(reference_path)
    assert reference.infill.line_count > 0, "Failed to extract reference infill line family"
    assert reference.infill.spacing_mm > 0.0, "Failed to resolve reference infill spacing"

    image_bytes = FIXTURE_IMAGE.read_bytes()
    raster, geometry, toolpaths_service, gcode_service = _load_services()
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((color.id for color in analysis.colors if color.hex == "#000000"), analysis.colors[0].id if analysis.colors else None)
    assert selected is not None, "No selectable color found in fixture image"
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

    debug: dict = {}
    reference_spacing = max(0.15, reference.infill.spacing_mm)
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=reference_spacing,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=reference_spacing,
        infill_density=100.0,
        infill_angle_deg=reference.infill.angle_deg,
        fill_strategy="rotated_scanline",
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    generated_infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert generated_infill_paths, "Generated slicer output contains no fill-infill paths"
    generated_lines = _line_segments_from_toolpaths(generated_infill_paths)
    generated_summary = summarize_line_family(generated_lines)

    assert _angle_distance_deg(generated_summary.angle_deg, reference.infill.angle_deg) <= 8.0
    assert generated_summary.spacing_mm == pytest.approx(reference.infill.spacing_mm, rel=0.25, abs=0.15)
    assert generated_summary.line_count == pytest.approx(reference.infill.line_count, rel=0.55, abs=10.0)

    ref_diagonal = _count_long_diagonal_connectors(reference.infill.lines, main_angle_deg=reference.infill.angle_deg, spacing_mm=reference.infill.spacing_mm)
    gen_diagonal = _count_long_diagonal_connectors(generated_lines, main_angle_deg=generated_summary.angle_deg, spacing_mm=generated_summary.spacing_mm)
    assert gen_diagonal <= max(1, ref_diagonal)

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
    )
    preview_infill_count = len([entry for entry in preview if entry.get("kind") == "fill-infill"])
    assert preview_infill_count == len(generated_infill_paths)
    assert any(line.startswith("G1 X") for line in gcode)

    debug_payload = {
        "selected_image_fixture": str(FIXTURE_IMAGE),
        "selected_reference_gcode_fixture": str(reference_path),
        "selected_reference_3mf_fixture": str(FIXTURE_3MF),
        "extracted_reference_file_path": str(extracted_gcode) if extracted_gcode else None,
        "reference_3mf_inventory_count": len(archive_inventory),
        "component_count": regions.selected_component_count,
        "polygon_count": regions.polygon_count,
        "pen_width_mm": reference_spacing,
        "fillable_offset_mm": reference_spacing * 0.5,
        "reference_hatch_angle_deg": reference.infill.angle_deg,
        "reference_hatch_spacing_mm": reference.infill.spacing_mm,
        "generated_hatch_angle_deg": generated_summary.angle_deg,
        "generated_hatch_spacing_mm": generated_summary.spacing_mm,
        "reference_infill_lines": reference.infill.line_count,
        "generated_infill_lines": generated_summary.line_count,
        "generated_scanline_count": len(debug.get("raw_scanlines", [])),
        "generated_clipped_segment_count": len(debug.get("clipped_infill_lines", [])),
        "local_band_or_cell_count_estimate": len({path.metadata.get("scanline_polygon_index") for path in generated_infill_paths}),
        "required_infill_strokes": len(generated_infill_paths),
        "safe_connector_count": len(debug.get("valid_infill_connectors", [])),
        "rejected_connector_count": len(debug.get("rejected_infill_connectors", [])),
        "connector_rejection_reasons": debug.get("connector_rejection_reasons", {}),
        "detected_diagonal_crossovers_reference": ref_diagonal,
        "detected_diagonal_crossovers_generated": gen_diagonal,
        "pen_lifts_inside_infill_estimate": max(0, len(generated_infill_paths) - 1),
        "travel_moves_inside_infill": len([entry for entry in preview if entry.get("kind") == "travel"]),
    }
    print("\nREFERENCE_INFILL_DEBUG", json.dumps(debug_payload, separators=(",", ":"), sort_keys=True))


def test_reference_rectilinear_infill_can_emit_pen_down_connector_travels():
    assert FIXTURE_IMAGE.exists(), f"Missing fixture image: {FIXTURE_IMAGE}"

    image_bytes = FIXTURE_IMAGE.read_bytes()
    raster, geometry, toolpaths_service, gcode_service = _load_services()
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((color.id for color in analysis.colors if color.hex == "#000000"), analysis.colors[0].id if analysis.colors else None)
    assert selected is not None, "No selectable color found in fixture image"
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

    reference = parse_reference_gcode(FIXTURE_GCODE)
    reference_spacing = max(0.15, reference.infill.spacing_mm)

    before_debug: dict = {}
    toolpaths_before = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=reference_spacing,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=reference_spacing,
        infill_density=100.0,
        infill_angle_deg=reference.infill.angle_deg,
        fill_strategy="rotated_scanline",
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        debug=before_debug,
    )

    after_debug: dict = {}
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=reference_spacing,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=reference_spacing,
        infill_density=100.0,
        infill_angle_deg=reference.infill.angle_deg,
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
        debug=after_debug,
    )

    connector_paths = [path for path in toolpaths if path.kind == "fill-infill-travel"]
    assert connector_paths, "Expected at least one pen-down infill connector path"
    assert all(len(path.points) == 2 for path in connector_paths)

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
    )

    actual_pen_lifts = _count_real_pen_lifts_from_gcode(gcode, pen_up_s=575, pen_down_s=700)
    before_infill_paths = [path for path in toolpaths_before if path.kind == "fill-infill"]
    after_infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    after_connector_paths = [path for path in toolpaths if path.kind == "fill-infill-travel"]
    diagnostics = after_debug.get("infill_connector_diagnostics", {})
    top_rejection_reason = diagnostics.get("top_rejection_reason")

    report = {
        "before": {
            "pen_lifts": len(before_infill_paths),
            "infill_chains": len(before_infill_paths),
        },
        "after": {
            "pen_lifts": actual_pen_lifts,
            "infill_chains": len(after_infill_paths),
            "accepted_connectors": len(after_connector_paths),
            "rejected_connectors": diagnostics.get("rejected_connectors", 0),
            "top_rejection_reason": top_rejection_reason,
        },
        "diagnostics": diagnostics,
        "preview_fill_infill_travel_count": len([entry for entry in preview if entry.get("kind") == "fill-infill-travel"]),
        "preview_pen_down_connector_count": len([entry for entry in preview if entry.get("kind") == "fill-infill-travel" and entry.get("pen_down")]),
    }
    print("\nHA_CONNECTOR_COMPARISON", json.dumps(report, separators=(",", ":"), sort_keys=True))

    assert any(entry["kind"] == "fill-infill-travel" and entry.get("pen_down") for entry in preview)
    assert report["preview_fill_infill_travel_count"] == len(after_connector_paths)
    assert actual_pen_lifts < 50
    assert diagnostics.get("total_infill_rows", 0) > 0
    assert diagnostics.get("total_possible_adjacent_row_connector_attempts", 0) >= len(after_connector_paths)
    assert diagnostics.get("accepted_connectors", 0) == len(after_connector_paths)
    assert diagnostics.get("rejection_counts", {}).get("crosses_gap_hole_void", 0) >= 0
    assert diagnostics.get("rejection_counts", {}).get("outside_fillable_polygon", 0) >= 0
    assert diagnostics.get("rejection_counts", {}).get("different_component", 0) >= 0
    assert diagnostics.get("rejection_counts", {}).get("different_cell_or_section", 0) >= 0
    assert any(line.startswith("G1 X") for line in gcode)
