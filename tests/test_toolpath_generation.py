import pytest
from shapely.geometry import Polygon

from app.services.pipeline_core import GeometryBundle, generate_toolpaths, mm_to_ball_degrees


def _rect(width_deg: float, height_deg: float) -> Polygon:
    return Polygon([
        (0.0, 0.0),
        (width_deg, 0.0),
        (width_deg, height_deg),
        (0.0, height_deg),
    ])


def test_fill_wall_is_inset_by_half_pen_width():
    line_width_mm = 1.0
    line_width_deg = mm_to_ball_degrees(line_width_mm)
    printable = _rect(width_deg=line_width_deg * 10.0, height_deg=line_width_deg * 6.0)

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    wall_paths = [path for path in toolpaths if path.kind == "fill-wall"]
    assert wall_paths

    min_x = min(point.x for point in wall_paths[0].points)
    max_x = max(point.x for point in wall_paths[0].points)
    assert min_x == pytest.approx(line_width_deg * 0.5, abs=1e-6)
    assert max_x == pytest.approx((line_width_deg * 10.0) - (line_width_deg * 0.5), abs=1e-6)


def test_single_pass_regions_use_detail_fill_by_default():
    line_width_mm = 1.0
    line_width_deg = mm_to_ball_degrees(line_width_mm)
    printable = _rect(width_deg=line_width_deg * 6.0, height_deg=line_width_deg * 2.8)
    bundle = GeometryBundle(printable_geometry=printable)

    single_wall_paths = generate_toolpaths(
        bundle,
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )
    assert any(path.kind == "detail-trace" for path in single_wall_paths)

    centerline_paths = generate_toolpaths(
        bundle,
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        thin_detail_mode=False,
        small_shape_mode="centerline",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )
    assert any(path.kind == "detail-trace" for path in centerline_paths)


def test_regions_without_outline_clearance_fall_back_to_detail_fill():
    line_width_mm = 1.0
    line_width_deg = mm_to_ball_degrees(line_width_mm)
    printable = _rect(width_deg=line_width_deg * 0.8, height_deg=line_width_deg * 4.0)

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    assert any(path.kind == "detail-trace" for path in toolpaths)
    assert not any(path.kind == "fill-wall" for path in toolpaths)


def test_simple_rectangle_infill_becomes_single_zigzag_path():
    line_width_mm = 1.0
    line_width_deg = mm_to_ball_degrees(line_width_mm)
    printable = _rect(width_deg=line_width_deg * 10.0, height_deg=line_width_deg * 10.0)

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert len(infill_paths) == 1
    assert len(infill_paths[0].points) > 4

    y_values = [point.y for point in infill_paths[0].points]
    assert max(y_values) > min(y_values)


def test_trapezoid_infill_follows_angled_walls_without_fragmenting():
    line_width_mm = 1.0
    line_width_deg = mm_to_ball_degrees(line_width_mm)
    printable = Polygon([
        (0.0, 0.0),
        (line_width_deg * 16.0, 0.0),
        (line_width_deg * 12.0, line_width_deg * 18.0),
        (line_width_deg * 4.0, line_width_deg * 18.0),
    ])

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        outline_after_fill=False,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert len(infill_paths) == 1
    assert len(infill_paths[0].points) > 10
