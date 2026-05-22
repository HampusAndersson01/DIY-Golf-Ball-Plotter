import math

import pytest
from shapely.geometry import LineString, MultiPolygon, Polygon

from app.models.geometry import Point, Segment, Toolpath
from app.services import pipeline_core
from app.services.gcode_service import GcodeService
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
    return printable_geometry.buffer(-(line_width_mm * 0.5), join_style=1)


def _cleanup_outline_region(printable_geometry, line_width_mm=1.0):
    return printable_geometry.buffer(-(line_width_mm * 0.25), join_style=1)


def _line_for_path(path: Toolpath) -> LineString:
    return LineString([(point.x, point.y) for point in path.points])


def _path_lengths(paths: list[Toolpath]) -> list[float]:
    return [pipeline_core.segment_length(path.points) for path in paths if len(path.points) >= 2]


def _path_segments(paths: list[Toolpath]) -> list[tuple[Point, Point]]:
    segments: list[tuple[Point, Point]] = []
    for path in paths:
        for start, end in zip(path.points, path.points[1:]):
            segments.append((start, end))
    return segments


def _fill_modes(paths: list[Toolpath]) -> set[str]:
    return {
        str(path.metadata.get("fill_mode"))
        for path in paths
        if path.kind == "fill-infill" and path.metadata.get("fill_mode") is not None
    }


def _assert_infill_segments_stay_inside_region(toolpaths, region, epsilon=1e-6):
    cover_region = region.buffer(epsilon, join_style=1)
    for path in toolpaths:
        if path.kind != "fill-infill":
            continue
        for start, end in zip(path.points, path.points[1:]):
            assert cover_region.covers(LineString([(start.x, start.y), (end.x, end.y)]))


def test_fill_wall_is_inset_inside_outer_cleanup_outline():
    line_width_mm = 1.0
    printable = _rect(width_mm=10.0, height_mm=6.0)

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=line_width_mm,
        wall_count=2,
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
    assert min_x == pytest.approx(line_width_mm * 1.25, abs=1e-6)
    assert max_x == pytest.approx(10.0 - (line_width_mm * 1.25), abs=1e-6)


@pytest.mark.parametrize("outline_after_fill", [False, True])
def test_cleanup_outline_tracks_visible_fill_edge(outline_after_fill):
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
        outline_after_fill=outline_after_fill,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    outline_path = next(path for path in toolpaths if path.kind == "outline")
    cleanup_region = _cleanup_outline_region(printable, line_width_mm=line_width_mm)
    expected_min_x = min(point[0] for point in cleanup_region.exterior.coords)
    expected_max_x = max(point[0] for point in cleanup_region.exterior.coords)

    assert min(point.x for point in outline_path.points) == pytest.approx(expected_min_x, abs=1e-6)
    assert max(point.x for point in outline_path.points) == pytest.approx(expected_max_x, abs=1e-6)
    assert outline_path.metadata["source_polygon_matches_infill_clip_polygon"] is True
    assert outline_path.metadata["outline_uses_infill_clip_polygon"] is True
    assert outline_path.metadata["generated_from"] == "final_fill_clip_polygon"
    assert outline_path.metadata["source_region_id"] == "component_001"


def test_small_regions_use_fill_or_detail_without_losing_coverage():
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
    assert any(path.kind in {"fill-infill", "detail-trace"} for path in single_wall_paths)

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
    assert any(path.kind in {"fill-infill", "detail-trace"} for path in centerline_paths)


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
    assert _fill_modes(infill_paths) == {"large_open"}


def test_long_horizontal_rectangle_prefers_horizontal_long_axis_infill():
    printable = _rect(width_mm=50.0, height_mm=2.0)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert _fill_modes(infill_paths) == {"long_thin"}
    assert all(path.metadata["resolved_infill_angle_deg"] == pytest.approx(0.0, abs=1e-6) for path in infill_paths)
    assert all(path.metadata["long_thin_fast_path_used"] is True for path in infill_paths)

    lengths = _path_lengths(infill_paths)
    assert lengths
    assert sum(lengths) / len(lengths) > 30.0

    horizontal_motion = sum(abs(end.x - start.x) for start, end in _path_segments(infill_paths))
    vertical_motion = sum(abs(end.y - start.y) for start, end in _path_segments(infill_paths))
    assert horizontal_motion > vertical_motion * 8.0


def test_long_vertical_rectangle_prefers_vertical_long_axis_infill():
    printable = _rect(width_mm=2.0, height_mm=50.0)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert _fill_modes(infill_paths) == {"long_thin"}
    assert all(path.metadata["resolved_infill_angle_deg"] == pytest.approx(90.0, abs=1e-6) for path in infill_paths)
    assert all(path.metadata["long_thin_fast_path_used"] is True for path in infill_paths)

    horizontal_motion = sum(abs(end.x - start.x) for start, end in _path_segments(infill_paths))
    vertical_motion = sum(abs(end.y - start.y) for start, end in _path_segments(infill_paths))
    assert vertical_motion > horizontal_motion * 8.0


def test_near_square_shape_uses_candidate_scoring_without_long_thin_fast_path():
    slicer = pipeline_core.SlicerService()
    region = _infill_region(_rect(12.0, 12.0), line_width_mm=0.5)

    resolved_angle, debug = slicer._resolve_infill_angle(
        region,
        spacing_mm=0.5,
        angle_deg=45.0,
        alternate_angle_deg=-45.0,
        fill_strategy="adaptive_angle",
        min_segment_length_mm=0.0,
        line_width_mm=0.5,
        region_index=0,
    )

    assert debug["long_thin_fast_path_used"] is False
    scored_angles = {round(metric["angle_deg"], 6) for metric in debug["candidate_metrics"]}
    assert round(resolved_angle, 6) in scored_angles
    assert round(debug["dominant_axis_angle_deg"], 6) in scored_angles


def test_candidate_angle_scoring_prefers_fewer_long_segments():
    slicer = pipeline_core.SlicerService()
    region = _infill_region(_rect(50.0, 2.0), line_width_mm=0.5)

    horizontal = slicer._score_infill_candidate(
        slicer._collect_scanline_rows(region, spacing_mm=0.5, angle_deg=0.0, min_segment_length_mm=0.0),
        spacing_mm=0.5,
        angle_deg=0.0,
    )
    vertical = slicer._score_infill_candidate(
        slicer._collect_scanline_rows(region, spacing_mm=0.5, angle_deg=90.0, min_segment_length_mm=0.0),
        spacing_mm=0.5,
        angle_deg=90.0,
    )

    assert horizontal["score"] > vertical["score"]
    assert horizontal["average_segment_length_mm"] > vertical["average_segment_length_mm"] * 8.0
    assert horizontal["segments"] < vertical["segments"]


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


def test_short_clipped_infill_fragments_are_skipped():
    slicer = pipeline_core.SlicerService()
    triangle = Polygon([
        (0.0, 0.0),
        (10.0, 0.0),
        (0.0, 10.0),
    ])

    paths = slicer._generate_scanline_infill(
        triangle,
        spacing_mm=1.0,
        angle_deg=0.0,
        min_segment_length_mm=2.0,
        tolerance_mm=0.0,
        allow_pen_down_infill_connectors=False,
    )

    assert paths
    assert all(pipeline_core.segment_length(path.points) >= 2.0 for path in paths)


def test_long_thin_infill_ordering_is_deterministic():
    printable = _rect(width_mm=50.0, height_mm=2.0)

    first = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )
    second = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    first_infill = [path.points for path in first if path.kind == "fill-infill"]
    second_infill = [path.points for path in second if path.kind == "fill-infill"]
    assert first_infill == second_infill


def test_small_detail_region_uses_hybrid_small_detail_fill_mode():
    printable = _rect(width_mm=4.0, height_mm=2.0)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert _fill_modes(infill_paths) == {"small_detail_or_text"}
    assert any(path.metadata.get("small_detail_fill_style") in {"contour_following", "sparse_strokes"} for path in infill_paths)
    assert not any(path.metadata.get("long_thin_fast_path_used") for path in infill_paths)


def test_small_detail_fill_uses_fewer_more_meaningful_strokes_than_hatch():
    slicer = pipeline_core.SlicerService()
    printable = _rect(width_mm=4.0, height_mm=2.0)
    region = _infill_region(printable, line_width_mm=0.5)

    hatch = slicer._generate_scanline_infill(
        region,
        spacing_mm=0.5,
        angle_deg=45.0,
        min_segment_length_mm=0.0,
        tolerance_mm=0.0,
        allow_pen_down_infill_connectors=False,
    )
    hybrid = slicer._generate_small_detail_fill(
        region,
        line_width_mm=0.5,
        scanline_spacing_mm=0.5,
        angle_deg=45.0,
        min_segment_length_mm=0.0,
        tolerance_mm=0.0,
        detail_tolerance_mm=0.0,
        allow_overlap=True,
    )

    hatch_segments = sum(max(0, len(path.points) - 1) for path in hatch)
    hybrid_segments = sum(max(0, len(path.points) - 1) for path in hybrid)

    assert hatch_segments > 0
    assert hybrid_segments > 0
    assert len(hybrid) <= len(hatch)
    assert max(_path_lengths(hybrid)) >= max(_path_lengths(hatch)) * 0.8


def test_small_detail_fill_stays_inside_true_polygon_and_preserves_hole():
    outer = _rect(5.0, 4.0)
    hole = Polygon([(x + 1.5, y + 1.0) for x, y in _rect(2.0, 2.0).exterior.coords[:-1]])
    printable = Polygon(outer.exterior.coords, [hole.exterior.coords])

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert _fill_modes(infill_paths) == {"small_detail_or_text"}
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable, line_width_mm=0.5))
    assert all(not hole.buffer(-0.01).covers(_line_for_path(path)) for path in infill_paths)


def test_small_detail_preview_uses_pen_up_travel_and_outline_draws_last():
    printable = _rect(width_mm=4.0, height_mm=2.0)
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=True,
        simplify_tolerance_mm=0.0,
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.5)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)

    service = GcodeService()
    _, preview = service.generate_from_toolpaths(
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
        include_comments=False,
    )

    preview_kinds = [entry["kind"] for entry in preview]
    assert "travel" in preview_kinds
    assert [entry["kind"] for entry in preview if entry["kind"] != "travel"][-1] == "outline"


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


def test_area_fill_suppresses_direct_source_outline_segments():
    printable = _rect(20.0, 20.0)
    bundle = GeometryBundle(
        printable_geometry=printable,
        outline_segments=[
            Segment(
                points=[
                    Point(-5.0, 10.0),
                    Point(25.0, 10.0),
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

    outline_paths = [path for path in toolpaths if path.kind == "outline"]
    assert outline_paths
    assert all(path.source != "mask_contour" for path in outline_paths)
    assert all(path.metadata["source_polygon_matches_infill_clip_polygon"] is True for path in outline_paths)
    assert all(path.metadata["generated_from"] == "final_fill_clip_polygon" for path in outline_paths)
    assert all(path.metadata["outline_uses_infill_clip_polygon"] is True for path in outline_paths)


def test_standalone_outline_segments_are_preserved_without_fill_geometry():
    bundle = GeometryBundle(
        outline_segments=[
            Segment(
                points=[
                    Point(0.0, 0.0),
                    Point(10.0, 0.0),
                    Point(10.0, 10.0),
                ],
                closed=False,
            )
        ],
    )

    toolpaths = generate_toolpaths(
        bundle,
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
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
    )

    outline_paths = [path for path in toolpaths if path.kind == "outline"]
    assert len(outline_paths) == 1
    assert outline_paths[0].source == "mask_contour"


def test_cleanup_outline_is_inside_or_on_printable_boundary():
    printable = Polygon([
        (0.0, 0.0),
        (12.0, 0.0),
        (10.0, 10.0),
        (1.0, 8.0),
    ])

    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.75, infill_spacing_mm=0.75)
    outline_paths = [path for path in toolpaths if path.kind == "outline"]

    assert outline_paths
    assert all(float(path.metadata["outline_offset_mm"]) <= 0.0 for path in outline_paths)
    assert all(printable.buffer(1e-6, join_style=1).covers(_line_for_path(path)) for path in outline_paths)


def test_fill_and_outline_share_same_printable_region():
    printable = _rect(20.0, 12.0)

    toolpaths = _generate_fill_toolpaths(printable)
    infill_region_ids = {
        path.metadata["source_region_id"]
        for path in toolpaths
        if path.kind == "fill-infill"
    }
    outline_region_ids = {
        path.metadata["source_region_id"]
        for path in toolpaths
        if path.kind == "outline"
    }

    assert outline_region_ids
    assert outline_region_ids.issubset(infill_region_ids)


def test_projected_cleanup_outline_logs_fill_clip_source(caplog):
    printable = _rect(10.0, 10.0)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.75, infill_spacing_mm=0.75, simplify_tolerance_mm=0.0)
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.75)

    with caplog.at_level("INFO", logger="app.services.pipeline_core"):
        projected = pipeline_core.project_toolpaths_to_ball_angles(
            prepared,
            center_lon_deg=0.0,
            center_lat_deg=0.0,
        )

    assert projected
    source_audits = [
        record.message for record in caplog.records
        if '"event":"cleanup_outline_source_audit"' in record.message
    ]
    assert source_audits
    assert all('"generated_from":"final_fill_clip_polygon"' in message for message in source_audits)
    assert all('"outline_uses_infill_clip_polygon":true' in message for message in source_audits)
    assert not any('"outline_uses_infill_clip_polygon":false' in message for message in source_audits)


def test_machine_motion_debug_matches_preview_and_gcode_paths():
    printable = _rect(10.0, 10.0)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.75, infill_spacing_mm=0.75, simplify_tolerance_mm=0.0)
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.75)
    projected = pipeline_core.project_toolpaths_to_ball_angles(
        prepared,
        center_lon_deg=0.0,
        center_lat_deg=0.0,
    )
    service = GcodeService()
    gcode, preview = service.generate_from_toolpaths(
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
    )

    debug = pipeline_core.build_machine_motion_debug(prepared, projected, preview, gcode, pen_up_s=575, pen_down_s=700)
    comparison = debug["path_coordinate_comparison"]

    assert comparison["same_path_count"] is True
    assert comparison["same_point_count_by_path"] is True
    assert comparison["mismatched_paths"] == []
    assert all((delta or 0.0) <= 1e-9 for delta in comparison["max_point_delta_deg_by_path"].values())


def test_sampling_debug_reports_same_policy_for_outline_and_infill():
    printable = _rect(10.0, 10.0)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.3, infill_spacing_mm=0.3, simplify_tolerance_mm=0.0)
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.3)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)
    sampling_debug = pipeline_core.build_sampling_debug(prepared, projected)

    assert sampling_debug["cleanup_outline_resampled"] is True
    assert sampling_debug["infill_resampled"] is True
    assert sampling_debug["same_sampling_policy"] is True
    assert sampling_debug["max_segment_length_surface_mm_by_kind"]["outline"] <= (
        sampling_debug["max_segment_length_surface_mm_by_kind"]["fill-infill"] * 2.0 + 1e-6
    )


def test_diagnostic_geometry_bundle_contains_expected_printable_geometry():
    bundle = pipeline_core.build_diagnostic_geometry_bundle("diagnostic_suite")

    assert bundle.printable_geometry is not None
    assert not bundle.printable_geometry.is_empty
    assert len(pipeline_core.normalize_geometry(bundle.printable_geometry)) >= 4


def test_3x3_square_calibration_metadata_contains_nine_equal_squares():
    bundle = pipeline_core.build_diagnostic_geometry_bundle("3x3_squares")
    toolpaths = _generate_fill_toolpaths(bundle.printable_geometry, line_width_mm=0.75, infill_spacing_mm=0.75)
    cleaned = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.75)
    projected = pipeline_core.project_toolpaths_to_ball_angles(cleaned, center_lon_deg=0.0, center_lat_deg=0.0)
    service = GcodeService()
    gcode, _preview = service.generate_from_toolpaths(
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
    )

    metadata = pipeline_core.build_calibration_pattern_metadata(
        "3x3_squares",
        bundle,
        cleaned,
        projected,
        gcode,
        ball_diameter_mm=42.67,
        pen_up_s=575,
        pen_down_s=700,
    )

    assert metadata is not None
    assert len(metadata["squares"]) == 9
    expected_labels = {
        "top-left",
        "top-center",
        "top-right",
        "middle-left",
        "middle-center",
        "middle-right",
        "bottom-left",
        "bottom-center",
        "bottom-right",
    }
    assert {square["id"] for square in metadata["squares"]} == expected_labels
    widths = {round(square["expectedSurfaceWidthMm"], 6) for square in metadata["squares"]}
    heights = {round(square["expectedSurfaceHeightMm"], 6) for square in metadata["squares"]}
    assert widths == {4.5}
    assert heights == {4.5}
    assert all(square["surfaceMmBbox"]["width"] == pytest.approx(4.5, abs=1e-6) for square in metadata["squares"])
    assert all(square["surfaceMmBbox"]["height"] == pytest.approx(4.5, abs=1e-6) for square in metadata["squares"])
    assert all(square["machineDegreeBbox"] is not None for square in metadata["squares"])
    assert all(square["gcodeBbox"] is not None for square in metadata["squares"])
    assert metadata["previewAndGcodeShareSameProjectedPaths"] is True
    assert metadata["projectedVsGcodeMismatchSquareIds"] == []


def test_3x3_square_gcode_bbox_matches_projected_bbox_within_rounding_tolerance():
    bundle = pipeline_core.build_diagnostic_geometry_bundle("3x3_squares")
    toolpaths = _generate_fill_toolpaths(bundle.printable_geometry, line_width_mm=0.75, infill_spacing_mm=0.75)
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.75)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)
    service = GcodeService()
    gcode, preview = service.generate_from_toolpaths(
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
    )
    metadata = pipeline_core.build_calibration_pattern_metadata(
        "3x3_squares",
        bundle,
        prepared,
        projected,
        gcode,
        ball_diameter_mm=42.67,
        pen_up_s=575,
        pen_down_s=700,
    )
    projected_debug = pipeline_core.build_projected_path_debug(prepared, projected, preview)

    assert metadata is not None
    assert projected_debug["preview_and_gcode_share_same_projected_paths"] is True
    for square in metadata["squares"]:
        assert square["gcodeMatchesMachineDegreeBbox"] is True
        assert square["gcodeBbox"]["width"] == pytest.approx(square["machineDegreeBbox"]["width"], abs=1e-4)
        assert square["gcodeBbox"]["height"] == pytest.approx(square["machineDegreeBbox"]["height"], abs=1e-4)


def test_smaller_mm_infill_spacing_generates_many_more_rows():
    printable = _rect(30.0, 30.0)

    sparse = _generate_fill_toolpaths(printable, line_width_mm=0.75, infill_spacing_mm=0.75)
    dense = _generate_fill_toolpaths(printable, line_width_mm=0.15, infill_spacing_mm=0.15)

    sparse_segments = sum(max(0, len(path.points) - 1) for path in sparse if path.kind == "fill-infill")
    dense_segments = sum(max(0, len(path.points) - 1) for path in dense if path.kind == "fill-infill")

    assert dense_segments > sparse_segments * 4.5


def test_pen_width_drives_dense_infill_spacing_when_spacing_matches_pen_width():
    printable = _rect(10.0, 10.0)

    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.3, infill_spacing_mm=0.3)
    infill_path = next(path for path in toolpaths if path.kind == "fill-infill")

    row_positions = []
    for point in infill_path.points:
        y_value = round(point.y, 6)
        if not row_positions or abs(row_positions[-1] - y_value) > 1e-6:
            row_positions.append(y_value)

    spacings = [row_positions[index] - row_positions[index - 1] for index in range(1, len(row_positions))]

    assert len(row_positions) > 20
    assert min(spacings) == pytest.approx(0.3, abs=1e-6)
    assert max(spacings) == pytest.approx(0.3, abs=1e-6)


def test_projection_preparation_resamples_long_diagonal_outline_segments_before_projection():
    printable = Polygon([
        (0.0, 0.0),
        (20.0, 2.0),
        (18.0, 18.0),
        (2.0, 16.0),
    ])
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.3,
        infill_spacing_mm=0.3,
        simplify_tolerance_mm=0.6,
    )

    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.3)
    projected = pipeline_core.project_toolpaths_to_ball_angles(
        prepared,
        center_lon_deg=0.0,
        center_lat_deg=0.0,
    )

    limit_mm = min(0.3 * 0.5, pipeline_core.DEFAULT_PROJECTION_SAMPLING_MAX_SEGMENT_MM)
    drawing_paths = [path for path in prepared if path.kind in {"outline", "fill-wall", "fill-infill", "detail-trace"}]
    assert drawing_paths
    assert all(float(path.metadata["max_surface_segment_mm_after_resampling"]) <= limit_mm + 1e-6 for path in drawing_paths)
    assert all(int(path.metadata.get("projection_count", 0)) == 1 for path in projected)


def test_cleanup_preserves_short_outline_segments_before_projection():
    outline = Toolpath(
        points=[
            Point(0.0, 0.0),
            Point(0.3, 0.0),
            Point(0.6, 0.2),
            Point(0.9, 0.2),
            Point(1.2, 0.4),
        ],
        kind="outline",
        closed=False,
        coordinate_space="surface_mm",
    )

    cleaned, stats = pipeline_core.cleanup_surface_toolpaths(
        [outline],
        tolerance_mm=0.0,
        min_segment_length_mm=0.5,
    )

    assert stats["short_segments_removed"] == 0
    assert len(cleaned) == 1
    assert cleaned[0].points == outline.points


def test_cleanup_can_still_prune_short_infill_segments_when_requested():
    infill = Toolpath(
        points=[
            Point(0.0, 0.0),
            Point(0.3, 0.0),
            Point(1.0, 0.0),
        ],
        kind="fill-infill",
        closed=False,
        coordinate_space="surface_mm",
    )

    cleaned, stats = pipeline_core.cleanup_surface_toolpaths(
        [infill],
        tolerance_mm=0.0,
        min_segment_length_mm=0.5,
    )

    assert stats["short_segments_removed"] == 1
    assert len(cleaned) == 1
    assert cleaned[0].points == [Point(0.0, 0.0), Point(1.0, 0.0)]


def test_prepare_projection_handles_degenerate_closed_outline_without_fake_closing_edge():
    outline = Toolpath(
        points=[
            Point(0.0, 0.0),
            Point(0.29805293668211186, 0.0),
            Point(0.0, 0.0),
        ],
        kind="outline",
        closed=True,
        coordinate_space="surface_mm",
        path_id="outline_017",
        metadata={"pen_width_mm": 0.2},
    )

    prepared = pipeline_core.prepare_toolpaths_for_projection([outline], default_pen_width_mm=0.2)

    assert len(prepared) == 1
    assert prepared[0].closed is False
    assert prepared[0].metadata["closed_path_degenerated_before_projection"] is True
    assert float(prepared[0].metadata["max_surface_segment_mm_after_resampling"]) <= 0.1 + 1e-6


def test_outer_ring_and_hole_remain_separate_paths_with_pen_up_travel():
    outer = _rect(12.0, 12.0)
    hole = Polygon([(x + 4.0, y + 4.0) for x, y in _rect(4.0, 4.0).exterior.coords[:-1]])
    printable = Polygon(outer.exterior.coords, [hole.exterior.coords])
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.3, infill_spacing_mm=0.3)
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.3)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)

    service = GcodeService()
    _, preview = service.generate_from_toolpaths(
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
        include_comments=False,
    )

    outline_paths = [entry for entry in preview if entry["kind"] == "outline"]
    travel_paths = [entry for entry in preview if entry["kind"] == "travel"]
    assert len(outline_paths) >= 2
    assert travel_paths
    assert all(entry["closed"] is True for entry in outline_paths)


def test_long_thin_preview_and_gcode_use_same_canonical_paths_with_outline_last():
    printable = _rect(50.0, 2.0)
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=True,
        simplify_tolerance_mm=0.0,
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.5)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)

    service = GcodeService()
    gcode, preview = service.generate_from_toolpaths(
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
        include_comments=False,
    )

    projected_debug = pipeline_core.build_projected_path_debug(prepared, projected, preview)
    non_travel_preview_kinds = [entry["kind"] for entry in preview if entry["kind"] != "travel"]

    assert gcode
    assert projected_debug["preview_and_gcode_share_same_projected_paths"] is True
    assert "fill-infill" in non_travel_preview_kinds
    assert non_travel_preview_kinds[-1] == "outline"


def test_preview_and_gcode_share_same_projected_points_after_resampling():
    printable = Polygon([
        (0.0, 0.0),
        (8.0, 0.0),
        (10.0, 5.0),
        (4.0, 10.0),
        (0.0, 7.0),
    ])
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.3, infill_spacing_mm=0.3)
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.3)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)

    service = GcodeService()
    _, preview = service.generate_from_toolpaths(
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
        include_comments=False,
    )

    projected_debug = pipeline_core.build_projected_path_debug(prepared, projected, preview)
    assert projected_debug["preview_and_gcode_share_same_projected_paths"] is True


def test_surface_artwork_scaling_happens_before_projection():
    original = Toolpath(
        points=[
            Point(-12.0, -10.0),
            Point(12.0, -10.0),
            Point(12.0, 10.0),
            Point(-12.0, 10.0),
            Point(-12.0, -10.0),
        ],
        kind="outline",
        closed=True,
        coordinate_space="surface_mm",
    )
    original = pipeline_core.prepare_toolpaths_for_projection([original], default_pen_width_mm=0.75)[0]
    bundle = GeometryBundle(
        outline_segments=[Segment(points=original.points, closed=original.closed)],
    )
    scaled_bundle = pipeline_core.apply_surface_artwork_scale(bundle, 50.0)
    scaled_toolpath = Toolpath(
        points=scaled_bundle.outline_segments[0].points,
        kind="outline",
        closed=True,
        coordinate_space="surface_mm",
    )
    scaled_toolpath = pipeline_core.prepare_toolpaths_for_projection([scaled_toolpath], default_pen_width_mm=0.75)[0]

    original_projected = pipeline_core.project_toolpaths_to_ball_angles([original], center_lon_deg=0.0, center_lat_deg=35.0)[0]
    scaled_projected = pipeline_core.project_toolpaths_to_ball_angles([scaled_toolpath], center_lon_deg=0.0, center_lat_deg=35.0)[0]

    original_bounds = pipeline_core._bbox_or_none(original_projected.points)
    scaled_bounds = pipeline_core._bbox_or_none(scaled_projected.points)
    assert original_bounds is not None
    assert scaled_bounds is not None
    projected_center_x = (original_bounds["minX"] + original_bounds["maxX"]) / 2.0
    projected_center_y = (original_bounds["minY"] + original_bounds["maxY"]) / 2.0
    naive_projected_points = [
        Point(
            projected_center_x + ((point.x - projected_center_x) * 0.5),
            projected_center_y + ((point.y - projected_center_y) * 0.5),
        )
        for point in original_projected.points
    ]

    assert int(scaled_projected.metadata.get("projection_count", 0)) == 1
    assert scaled_bounds["width"] < original_bounds["width"]
    assert scaled_bounds["height"] < original_bounds["height"]
    max_delta = max(
        abs(actual.x - naive.x) + abs(actual.y - naive.y)
        for actual, naive in zip(scaled_projected.points, naive_projected_points)
    )
    assert max_delta > 1e-3


def test_vertical_horizontal_and_diagonal_outline_segments_project_with_bounded_step_size():
    printable = Polygon([
        (0.0, 0.0),
        (16.0, 0.0),
        (20.0, 12.0),
        (12.0, 20.0),
        (0.0, 20.0),
    ])
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.3,
        infill_spacing_mm=0.3,
        simplify_tolerance_mm=0.8,
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.3)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=12.0)

    outline_paths = [path for path in prepared if path.kind in {"outline", "fill-wall"}]
    assert outline_paths
    assert all(float(path.metadata["max_surface_segment_mm_after_resampling"]) <= 0.15 + 1e-6 for path in outline_paths)
    projected_outline_lengths = [
        max(pipeline_core._segment_lengths_mm(path.points, closed=path.closed))
        for path in projected
        if path.kind in {"outline", "fill-wall"} and len(path.points) >= 2
    ]
    assert projected_outline_lengths
    assert max(projected_outline_lengths) < 2.5


def test_machine_projection_keeps_x_independent_from_surface_y():
    left_low = pipeline_core.surface_mm_to_ball_angles(Point(-10.0, -20.0), center_lon_deg=0.0, center_lat_deg=0.0)
    left_mid = pipeline_core.surface_mm_to_ball_angles(Point(-10.0, 0.0), center_lon_deg=0.0, center_lat_deg=0.0)
    left_high = pipeline_core.surface_mm_to_ball_angles(Point(-10.0, 20.0), center_lon_deg=0.0, center_lat_deg=0.0)
    right_low = pipeline_core.surface_mm_to_ball_angles(Point(10.0, -20.0), center_lon_deg=0.0, center_lat_deg=0.0)
    right_mid = pipeline_core.surface_mm_to_ball_angles(Point(10.0, 0.0), center_lon_deg=0.0, center_lat_deg=0.0)
    right_high = pipeline_core.surface_mm_to_ball_angles(Point(10.0, 20.0), center_lon_deg=0.0, center_lat_deg=0.0)

    assert abs(left_low.x) > abs(left_mid.x)
    assert abs(left_high.x) > abs(left_mid.x)
    assert abs(right_low.x) > abs(right_mid.x)
    assert abs(right_high.x) > abs(right_mid.x)


def test_machine_projection_keeps_y_independent_from_surface_x():
    low_left = pipeline_core.surface_mm_to_ball_angles(Point(-20.0, -10.0), center_lon_deg=0.0, center_lat_deg=0.0)
    low_mid = pipeline_core.surface_mm_to_ball_angles(Point(0.0, -10.0), center_lon_deg=0.0, center_lat_deg=0.0)
    low_right = pipeline_core.surface_mm_to_ball_angles(Point(20.0, -10.0), center_lon_deg=0.0, center_lat_deg=0.0)
    high_left = pipeline_core.surface_mm_to_ball_angles(Point(-20.0, 10.0), center_lon_deg=0.0, center_lat_deg=0.0)
    high_mid = pipeline_core.surface_mm_to_ball_angles(Point(0.0, 10.0), center_lon_deg=0.0, center_lat_deg=0.0)
    high_right = pipeline_core.surface_mm_to_ball_angles(Point(20.0, 10.0), center_lon_deg=0.0, center_lat_deg=0.0)

    assert low_left.y == pytest.approx(low_mid.y, abs=1e-9)
    assert low_mid.y == pytest.approx(low_right.y, abs=1e-9)
    assert high_left.y == pytest.approx(high_mid.y, abs=1e-9)
    assert high_mid.y == pytest.approx(high_right.y, abs=1e-9)


def test_machine_projection_uses_fixed_ball_circumference_for_x():
    radius = pipeline_core.ball_radius_mm()
    projected = pipeline_core.surface_mm_to_ball_angles(Point(10.0, 20.0), center_lon_deg=0.0, center_lat_deg=0.0)
    lat = 20.0 / radius
    expected_x_deg = math.degrees(10.0 / (radius * math.cos(lat)))

    assert projected.x == pytest.approx(expected_x_deg, abs=1e-9)


def test_merge_motion_profiles_counts_axis_and_blended_segments_across_paths():
    toolpaths = [
        Toolpath(
            points=[Point(0.0, 0.0), Point(1.0, 0.0), Point(1.0, 1.0)],
            kind="outline",
            closed=False,
        ),
        Toolpath(
            points=[Point(1.0, 1.0), Point(2.0, 2.0), Point(3.0, 3.0)],
            kind="outline",
            closed=False,
        ),
    ]

    profile = pipeline_core._merge_motion_profiles(toolpaths)

    assert profile["horizontal_segments"] == 1
    assert profile["vertical_segments"] == 1
    assert profile["blended_xy_segments"] == 2
    assert profile["total_segments"] == 4
    assert profile["max_consecutive_blended_xy_segments"] == 2
    assert abs(float(profile["blended_xy_ratio"]) - 0.5) < 1e-9
