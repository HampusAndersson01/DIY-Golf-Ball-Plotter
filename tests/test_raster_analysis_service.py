import io

import cv2
import numpy as np
from PIL import Image, ImageDraw
from shapely.geometry import Point as ShapelyPoint, Polygon

from app.models.machine_state import MachineState
from app.services.gcode_service import GcodeService
from app.services.geometry_service import GeometryService
from app.services.raster_analysis_service import RasterAnalysisService
from app.services.toolpath_service import ToolpathService

from tests.test_svg_parser import CONFIG


def make_service() -> RasterAnalysisService:
    return RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))


def make_logo_bytes() -> bytes:
    image = Image.new("RGB", (120, 120), "white")
    draw = ImageDraw.Draw(image)
    draw.rectangle((15, 15, 105, 105), fill="black")
    draw.ellipse((40, 40, 80, 80), fill="white")
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def make_text_logo_bytes() -> bytes:
    canvas = np.full((120, 420, 3), 255, dtype=np.uint8)
    cv2.putText(canvas, "Arsenal", (8, 82), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 3, cv2.LINE_8)
    buffer = io.BytesIO()
    Image.fromarray(canvas, mode="RGB").save(buffer, format="PNG")
    return buffer.getvalue()


def test_detects_black_and_white_logo_colors():
    result = make_service().analyze_image(make_logo_bytes(), max_colors=4)
    colors = {entry.hex for entry in result.colors}
    assert "#000000" in colors
    assert "#FFFFFF" in colors


def test_black_selection_preserves_white_hole_and_generates_fill():
    service = make_service()
    geometry = GeometryService()
    toolpaths_service = ToolpathService()

    mask = service.build_mask(
        make_logo_bytes(),
        ["#000000"],
        tolerance=8,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=0,
    )
    regions = service.extract_regions(mask, min_region_area_px=10, simplify_tolerance_px=0)

    assert regions.region_count == 1
    assert regions.hole_count == 1
    polygon = regions.bundle.fill_shapes[0].geometry
    assert len(polygon.interiors) == 1

    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=0.75,
        wall_count=1,
        infill_pattern="zigzag",
        infill_spacing_mm=0.75,
        infill_density=100.0,
        infill_angle_deg=0.0,
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        debug={},
    )

    assert any(path.kind == "fill-wall" for path in toolpaths)
    assert any(path.kind == "fill-infill" for path in toolpaths)

    hole_polygon = Polygon(polygon.interiors[0])
    hole_polygon = geometry.map_bundle_to_angles(
        type(regions.bundle)(
            outline_segments=[],
            fill_boundary_segments=[],
            fill_shapes=[],
            printable_geometry=hole_polygon,
            cutout_geometry=None,
        ),
        regions.bounds,
        "contain",
        True,
        4.0,
    ).printable_geometry

    infill_points = [point for path in toolpaths if path.kind == "fill-infill" for point in path.points]
    assert not any(hole_polygon.buffer(-0.01).contains(ShapelyPoint(point.x, point.y)) for point in infill_points)


def test_generated_gcode_contains_motion_for_black_regions():
    service = make_service()
    geometry = GeometryService()
    toolpaths_service = ToolpathService()
    gcode_service = GcodeService()

    regions = service.extract_regions(service.build_mask(make_logo_bytes(), ["#000000"], tolerance=8))
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=0.75,
        wall_count=1,
        infill_pattern="zigzag",
        infill_spacing_mm=0.75,
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
    )
    assert any(line.startswith("G1 X") for line in gcode)
    assert any(entry["kind"] == "fill-infill" for entry in preview)


def test_text_mask_generates_thin_detail_paths():
    service = make_service()
    geometry = GeometryService()
    toolpaths_service = ToolpathService()

    mask = service.build_mask(
        make_text_logo_bytes(),
        ["#000000"],
        tolerance=8,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=0,
    )
    regions = service.extract_regions(mask, min_region_area_px=0, simplify_tolerance_px=0)
    assert regions.detail_trace_component_count > 0
    assert regions.detail_trace_path_count > 0
    assert regions.skeleton_pixel_count > 0
    assert regions.bundle.detail_segments
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    debug = {}
    toolpaths = toolpaths_service.generate_from_regions(
        mapped,
        pen_width_mm=1.2,
        wall_count=1,
        infill_pattern="zigzag",
        infill_spacing_mm=1.2,
        infill_density=100.0,
        infill_angle_deg=0.0,
        min_region_area=0.0,
        min_fill_width_mm=1.2,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.0,
        thin_detail_simplify_mm=0.0,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        debug=debug,
    )

    assert debug["slicer_counts"]["thin_detail_fallback_region_count"] > 0
    assert debug["toolpath_counts"]["generated_detail_trace_paths"] > 0
    assert any(path.kind == "detail-trace" for path in toolpaths)
