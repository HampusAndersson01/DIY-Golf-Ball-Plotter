import pytest
from shapely.geometry import LineString, MultiPolygon, Polygon

from app.models.geometry import Point, Segment
from app.services.pipeline_core import GeometryBundle, generate_toolpaths
from app.services.toolpath_service import ToolpathService


def _rect(width_mm: float, height_mm: float) -> Polygon:
    return Polygon([
        (0.0, 0.0),
        (width_mm, 0.0),
        (width_mm, height_mm),
        (0.0, height_mm),
    ])


def _generate_fill_toolpaths(printable_geometry, **overrides):
    params = {
        "enable_fill": True,
        "line_width_mm": 1.0,
        "wall_count": 1,
        "infill_density": 100.0,
        "infill_spacing_mm": 1.0,
        "infill_angle_deg": 0.0,
        "outline_after_fill": False,
        "min_fill_area_mm2": 0.0,
        "min_fill_width_mm": 0.0,
        "simplify_tolerance_mm": 0.0,
        "remove_duplicate_paths": False,
        "small_shape_mode": "single-wall",
        "min_segment_length_mm": 0.0,
        "travel_optimization": "nearest-neighbor",
        "allow_pen_down_infill_connectors": True,
    }
    params.update(overrides)
    return generate_toolpaths(GeometryBundle(printable_geometry=printable_geometry), **params)


def _infill_region(printable_geometry, line_width_mm=1.0, wall_count=1):
    return printable_geometry.buffer(-(line_width_mm * max(1, wall_count)), join_style=1)


def _assert_infill_segments_stay_inside_region(toolpaths, region, epsilon=1e-6):
    cover_region = region.buffer(epsilon, join_style=1)
    for path in toolpaths:
        if path.kind != "fill-infill":
            continue
        for start, end in zip(path.points, path.points[1:]):
            assert cover_region.covers(LineString([(start.x, start.y), (end.x, end.y)]))


def test_fill_wall_is_inset_by_half_pen_width():
    line_width_mm = 1.0
    printable = _rect(width_mm=10.0, height_mm=6.0)

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
    assert min_x == pytest.approx(line_width_mm * 0.5, abs=1e-6)
    assert max_x == pytest.approx(10.0 - (line_width_mm * 0.5), abs=1e-6)


def test_single_pass_regions_use_detail_fill_by_default():
    line_width_mm = 1.0
    printable = _rect(width_mm=6.0, height_mm=2.8)
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
    printable = _rect(width_mm=0.8, height_mm=4.0)

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
    printable = _rect(width_mm=10.0, height_mm=10.0)

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
    printable = Polygon([
        (0.0, 0.0),
        (16.0, 0.0),
        (12.0, 18.0),
        (4.0, 18.0),
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


def test_concave_c_shape_does_not_connect_across_open_gap():
    outer = _rect(24.0, 24.0)
    gap = _rect(16.0, 8.0)
    gap = Polygon([(point[0] + 8.0, point[1] + 8.0) for point in gap.exterior.coords[:-1]])
    printable = outer.difference(gap)

    toolpaths = _generate_fill_toolpaths(printable)
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable))


def test_rectangle_with_hole_does_not_connect_across_hole():
    outer = _rect(24.0, 24.0)
    hole = _rect(8.0, 8.0)
    hole = Polygon([(point[0] + 8.0, point[1] + 8.0) for point in hole.exterior.coords[:-1]])
    printable = Polygon(outer.exterior.coords, [hole.exterior.coords])

    toolpaths = _generate_fill_toolpaths(printable)
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable))


def test_multi_island_shape_does_not_connect_between_islands():
    left = _rect(10.0, 16.0)
    right = Polygon([(x + 14.0, y) for x, y in _rect(10.0, 16.0).exterior.coords[:-1]])
    printable = MultiPolygon([left, right])

    toolpaths = _generate_fill_toolpaths(printable)
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert len(infill_paths) >= 2
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable))


def test_disabling_pen_down_infill_connectors_outputs_separate_spans():
    printable = Polygon([
        (0.0, 0.0),
        (16.0, 0.0),
        (12.0, 18.0),
        (4.0, 18.0),
    ])

    toolpaths = _generate_fill_toolpaths(printable, allow_pen_down_infill_connectors=False)
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert len(infill_paths) > 1
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable))


def test_raster_area_fill_suppresses_injected_detail_segments():
    printable = _rect(20.0, 20.0)
    bundle = GeometryBundle(
        printable_geometry=printable,
        detail_segments=[
            Segment(
                points=[
                    Point(10.0, 0.0),
                    Point(10.0, 20.0),
                ],
                closed=False,
            )
        ],
    )

    toolpaths = ToolpathService().generate_from_regions(
        bundle,
        pen_width_mm=1.0,
        wall_count=1,
        infill_pattern="zigzag",
        infill_spacing_mm=1.0,
        infill_density=100.0,
        infill_angle_deg=0.0,
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.0,
        thin_detail_simplify_mm=0.0,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    assert not any(path.kind == "detail-trace" for path in toolpaths)


def test_smaller_mm_infill_spacing_generates_many_more_rows():
    printable = _rect(30.0, 30.0)

    sparse = _generate_fill_toolpaths(printable, line_width_mm=0.75, infill_spacing_mm=0.75)
    dense = _generate_fill_toolpaths(printable, line_width_mm=0.15, infill_spacing_mm=0.15)

    sparse_segments = sum(max(0, len(path.points) - 1) for path in sparse if path.kind == "fill-infill")
    dense_segments = sum(max(0, len(path.points) - 1) for path in dense if path.kind == "fill-infill")

    assert dense_segments > sparse_segments * 4.5
