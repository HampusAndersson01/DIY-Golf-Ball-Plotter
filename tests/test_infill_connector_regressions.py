from __future__ import annotations

import math
from io import BytesIO
from pathlib import Path

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

    connector_paths = [path for path in toolpaths if path.kind == "fill-infill-travel"]
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
    reference_spacing = max(0.15, 0.75)
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

    assert result["actual_pen_lifts"] < 120
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

    assert diagnostics.get("rejected_raster_mask_sampling", 0) >= 0
    assert diagnostics.get("rejected_outside_selected_color", 0) >= 0
    assert diagnostics.get("accepted_connectors", 0) == result["accepted_connector_count"]
    assert result["accepted_connector_count"] >= 0


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
    assert len(preview_toolpaths) == len(projected)
    assert len(gcode_toolpaths) == len(preview_toolpaths)
    assert preview_toolpaths[0].points[0].x == pytest.approx(gcode_toolpaths[0].points[0].x, abs=1e-4)
    assert preview_toolpaths[0].points[0].y == pytest.approx(gcode_toolpaths[0].points[0].y, abs=1e-4)
    assert debug.get("invalidConnectorCount", 0) == 0