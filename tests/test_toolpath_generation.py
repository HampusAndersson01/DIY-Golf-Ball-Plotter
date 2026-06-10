import io
import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import pytest
from PIL import Image, ImageDraw
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point as ShapelyPoint, Polygon
from shapely.ops import unary_union
from werkzeug.datastructures import MultiDict

from app import create_app
from app.models.geometry import Point, Segment, Toolpath
from app.models.machine_state import MachineState
from app.services import coverage_planner
from app.services import pipeline_core
from app.services.gcode_service import GcodeService
from app.services.geometry_service import GeometryService
from app.services.raster_analysis_service import RasterAnalysisService
from app.services.pipeline_core import GeometryBundle, generate_toolpaths
from app.services.toolpath_service import ToolpathService
from app.services.validation_service import ValidationService
from tests.test_svg_parser import CONFIG

ROOT = Path(__file__).resolve().parents[1]
HA_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "ha-compact-lightbg.png"
ARSENAL_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "black-arsenal-logo-png-1.png"
CAROLIN_FIXTURE = ROOT / "tests" / "fixtures" / "images" / "Carolin Line.png"


def _rect(width_mm: float, height_mm: float) -> Polygon:
    return Polygon([
        (0.0, 0.0),
        (width_mm, 0.0),
        (width_mm, height_mm),
        (0.0, height_mm),
    ])


def _frontend_default_arsenal_fixture(*, rotation_deg: float):
    app = create_app()
    config = app.config
    fixture_bytes = ARSENAL_FIXTURE.read_bytes()
    raster = RasterAnalysisService(config, MachineState(default_pen_up_s=575))
    geometry = GeometryService()
    toolpaths_service = ToolpathService()
    validation = ValidationService()

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
    toolpaths = toolpaths_service.generate_from_regions(
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
        debug=debug,
    )
    return placed, toolpaths, debug


@pytest.fixture(scope="session")
def arsenal_frontend_default_90_result():
    return _frontend_default_arsenal_fixture(rotation_deg=90)


def _generate_fill_toolpaths(printable_geometry, **overrides):
    params = {
        "enable_fill": True,
        "line_width_mm": 0.6,
        "wall_count": 1,
        "infill_density": 100.0,
        "infill_spacing_mm": 0.6,
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
        "infill_path_mode": "serpentine_optimized",
    }
    params.update(overrides)
    return generate_toolpaths(GeometryBundle(printable_geometry=printable_geometry), **params)


def _infill_region(printable_geometry, line_width_mm=0.6, wall_count=1):
    return printable_geometry.buffer(-(line_width_mm * 0.5), join_style=1)


def _cleanup_outline_region(printable_geometry, line_width_mm=1.0):
    return printable_geometry.buffer(-(line_width_mm * 0.5), join_style=1)


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


def _paths_starting_in_geometry(paths: list[Toolpath], geometry: Polygon) -> list[Toolpath]:
    selected: list[Toolpath] = []
    padded = geometry.buffer(1e-4)
    for path in paths:
        if len(path.points) < 2:
            continue
        if padded.covers(ShapelyPoint(path.points[0].x, path.points[0].y)):
            selected.append(path)
    return selected


def _largest_non_outline_uncovered_area_mm2(paths: list[Toolpath], geometry: Polygon, *, pen_radius_mm: float) -> float:
    coverage_parts = []
    for path in paths:
        if path.kind not in {"fill-infill", "fill-repair", "detail-trace"} or len(path.points) < 2:
            continue
        line = LineString([(point.x, point.y) for point in path.points])
        if line.is_empty or line.length <= 1e-9:
            continue
        coverage_parts.append(line.buffer(pen_radius_mm, cap_style=1, join_style=1))
    covered = unary_union(coverage_parts) if coverage_parts else None
    uncovered = geometry if covered is None else geometry.difference(covered)
    return max(
        (
            float(poly.area)
            for poly in pipeline_core.normalize_geometry(uncovered)
            if poly is not None and not poly.is_empty
        ),
        default=0.0,
    )


def _geom_to_mask(geom, bounds, px_per_mm: float) -> np.ndarray:
    min_x, min_y, max_x, max_y = bounds
    pad_mm = 0.6
    width_px = max(8, int(math.ceil((max_x - min_x + (2.0 * pad_mm)) * px_per_mm)))
    height_px = max(8, int(math.ceil((max_y - min_y + (2.0 * pad_mm)) * px_per_mm)))
    tx = -min_x + pad_mm
    ty = -min_y + pad_mm
    transformed = affinity.scale(affinity.translate(geom, xoff=tx, yoff=ty), xfact=px_per_mm, yfact=px_per_mm, origin=(0.0, 0.0))
    mask = np.zeros((height_px, width_px), dtype=np.uint8)
    for poly in pipeline_core.normalize_geometry(transformed):
        ext = np.array([[int(round(x)), int(round(y))] for x, y in poly.exterior.coords], dtype=np.int32)
        if len(ext) >= 3:
            cv2.fillPoly(mask, [ext], 255)
        for ring in poly.interiors:
            hole = np.array([[int(round(x)), int(round(y))] for x, y in ring.coords], dtype=np.int32)
            if len(hole) >= 3:
                cv2.fillPoly(mask, [hole], 0)
    return mask


def _paths_footprint_geometry(paths: list[Toolpath], pen_width_mm: float):
    stroke_geoms = []
    radius = max(0.01, pen_width_mm * 0.5)
    for p in paths:
        if len(p.points) < 2:
            continue
        line = LineString([(pt.x, pt.y) for pt in p.points])
        if line.is_empty or line.length <= 1e-9:
            continue
        stroke_geoms.append(line.buffer(radius, cap_style=1, join_style=1))
    if not stroke_geoms:
        return Polygon()
    return pipeline_core.unary_union(stroke_geoms)


def _actual_outline_paths(paths: list[Toolpath]) -> list[Toolpath]:
    return [
        path
        for path in paths
        if path.kind == "outline"
        or bool((path.metadata or {}).get("actual_outline_centerline", False))
        or str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTLINE_FALLBACK"
        or bool((path.metadata or {}).get("outline_fallback_mode", False))
    ]


def _nearest_path_point_distance(paths: list[Toolpath], target: tuple[float, float], *, kinds: set[str] | None = None) -> float:
    best = float("inf")
    for path in paths:
        if kinds is not None and path.kind not in kinds:
            continue
        for point in path.points:
            best = min(best, math.hypot(point.x - float(target[0]), point.y - float(target[1])))
    return best


def _normalized_path_signature(paths: list[Toolpath], kind: str) -> list[tuple[tuple[float, float], ...]]:
    signature: list[tuple[tuple[float, float], ...]] = []
    for path in paths:
        if path.kind != kind:
            continue
        signature.append(tuple((round(point.x, 6), round(point.y, 6)) for point in path.points))
    return signature


def _canonical_geometry_signature(path: Toolpath) -> tuple[object, ...]:
    rounded = tuple((round(point.x, 6), round(point.y, 6)) for point in path.points)
    if not path.closed:
        return (path.kind, path.closed, min(rounded, tuple(reversed(rounded))))
    core = rounded[:-1] if len(rounded) >= 2 and rounded[0] == rounded[-1] else rounded
    rotations = [tuple(core[index:] + core[:index]) for index in range(len(core))]
    return (path.kind, path.closed, min(rotations))


def _build_raster_fixture_toolpaths(
    fixture: Path,
    *,
    pen_width_mm: float = 0.8,
    infill_spacing_mm: float = 0.8,
    tolerance: int = 24,
    min_region_area_px: float = 8.0,
    simplify_tolerance_px: float = 0.35,
):
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    geometry = GeometryService()
    toolpath_service = ToolpathService()
    image_bytes = fixture.read_bytes()
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((c.id for c in analysis.colors if c.hex == "#000000"), analysis.colors[0].id)
    mask = raster.build_mask(
        image_bytes,
        [selected],
        tolerance=tolerance,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=1,
    )
    regions = raster.extract_regions(mask, min_region_area_px=min_region_area_px, simplify_tolerance_px=simplify_tolerance_px)
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    debug: dict[str, object] = {}
    toolpaths = toolpath_service.generate_from_regions(
        mapped,
        pen_width_mm=pen_width_mm,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=infill_spacing_mm,
        infill_density=100.0,
        infill_angle_deg=0.0,
        fill_strategy="contour_offset",
        outline_after_fill=True,
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.0,
        thin_detail_simplify_mm=0.0,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        expensive_coverage_repair=False,
        debug=debug,
    )
    return raster, geometry, mapped, toolpaths, debug


def _make_logo_bytes() -> bytes:
    image = Image.new("RGB", (120, 120), "white")
    draw = ImageDraw.Draw(image)
    draw.rectangle((15, 15, 105, 105), fill="black")
    draw.ellipse((40, 40, 80, 80), fill="white")
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def _fill_modes(paths: list[Toolpath]) -> set[str]:
    return {
        str(path.metadata.get("fill_mode"))
        for path in paths
        if path.kind == "fill-infill" and path.metadata.get("fill_mode") is not None
    }


def _drawable_row_offsets(row_data: dict) -> list[float]:
    return [
        float(row["offset_mm"])
        for row in row_data.get("rows") or []
        if row.get("segments")
    ]


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

    fill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    outline_paths = [path for path in toolpaths if path.kind == "outline"]
    assert fill_paths
    assert outline_paths
    assert min(point.x for point in fill_paths[0].points) >= 0.0
    assert max(point.x for point in fill_paths[0].points) <= 10.0


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
    assert outline_path.metadata["source_polygon_matches_infill_clip_polygon"] is True
    assert outline_path.metadata["outline_uses_infill_clip_polygon"] is True
    assert outline_path.metadata["generated_from"] == "final_fill_clip_polygon"
    assert str(outline_path.metadata["source_region_id"]).startswith("component_")


def test_contour_only_offsets_follow_pen_width_ladder():
    line_width_mm = 0.6
    printable = _rect(width_mm=8.0, height_mm=8.0)
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=line_width_mm,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
    )

    outlines = [path for path in toolpaths if path.kind == "outline"]
    fills = [path for path in toolpaths if path.kind == "fill-infill"]
    assert outlines
    assert fills
    assert not any(path.kind in {"detail-trace", "detail-continuation", "fill-infill-travel"} for path in toolpaths)

    expected_outline = 0.3
    expected_infill = {0.69, 1.08, 1.47}
    outline_offsets = {round(float(path.metadata.get("offset_mm", path.metadata.get("outline_offset_mm", 0.0))), 3) for path in outlines}
    infill_offsets = {round(float(path.metadata.get("offset_mm", path.metadata.get("scanline_offset_mm", 0.0))), 3) for path in fills}
    assert expected_outline in outline_offsets
    assert expected_infill.issubset(infill_offsets)
    assert max(index for index, path in enumerate(toolpaths) if path.kind == "outline") > max(
        index for index, path in enumerate(toolpaths) if path.kind == "fill-infill"
    )


def test_wide_c_shape_generates_nested_contour_infill_without_detail():
    outer = ShapelyPoint(0.0, 0.0).buffer(8.0, resolution=96)
    inner = ShapelyPoint(0.0, 0.0).buffer(4.5, resolution=96)
    ring = outer.difference(inner)
    slot = Polygon([(1.5, -10.0), (10.0, -10.0), (10.0, 10.0), (1.5, 10.0)])
    c_shape = ring.difference(slot)

    toolpaths = _generate_fill_toolpaths(
        c_shape,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
    )

    infill = [path for path in toolpaths if path.kind == "fill-infill"]
    assert len(infill) >= 2
    assert not any(path.kind in {"detail-trace", "detail-continuation"} for path in toolpaths)
    distinct_offsets = {round(float(path.metadata.get("offset_mm", 0.0)), 3) for path in infill}
    assert len(distinct_offsets) >= 2
    assert max(index for index, path in enumerate(toolpaths) if path.kind == "outline") > max(
        index for index, path in enumerate(toolpaths) if path.kind == "fill-infill"
    )


def test_holes_preserve_inner_final_outline():
    outer = ShapelyPoint(0.0, 0.0).buffer(8.0, quad_segs=96)
    inner = ShapelyPoint(0.0, 0.0).buffer(4.0, quad_segs=96)
    donut = outer.difference(inner)
    toolpaths = _generate_fill_toolpaths(
        donut,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
    )
    outer_outlines = [p for p in toolpaths if p.kind == "outline" and str((p.metadata or {}).get("path_role", "")) == "FINAL_OUTER_OUTLINE"]
    inner_outlines = [p for p in toolpaths if p.kind == "outline" and str((p.metadata or {}).get("path_role", "")) == "FINAL_INNER_OUTLINE"]
    assert outer_outlines
    assert inner_outlines
    assert not any(p.kind in {"detail-trace", "detail-continuation"} for p in toolpaths)


def test_final_outline_preserves_outer_and_inner_hole_boundaries():
    outer = _rect(width_mm=10.0, height_mm=10.0)
    hole = Polygon([(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0)])
    donut = outer.difference(hole)
    toolpaths = _generate_fill_toolpaths(
        donut,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
    )
    outlines = [path for path in toolpaths if path.kind == "outline"]
    assert len(outlines) >= 2, "expected outer and inner final outline paths"
    roles = {str(path.metadata.get("ring_role", "")) for path in outlines}
    assert "outer" in roles
    assert "hole" in roles
    assert not any(path.kind in {"detail-trace", "detail-continuation", "collapse-centerline"} for path in toolpaths)


def test_collapsed_offsets_do_not_emit_legacy_detail_or_centerline_fallback():
    # Narrow ribbon should remain contour-only (possibly crossed), never legacy detail/centerline fallback.
    outer = ShapelyPoint(0.0, 0.0).buffer(6.0, quad_segs=96)
    inner = ShapelyPoint(0.0, 0.0).buffer(5.0, quad_segs=96)
    crescent = outer.difference(inner)
    toolpaths = _generate_fill_toolpaths(
        crescent,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
    )
    assert any(path.kind == "outline" for path in toolpaths)
    assert not any(path.kind in {"detail-trace", "detail-continuation", "collapse-centerline", "gap-repair-centerline"} for path in toolpaths)


def test_post_generation_travel_optimizer_preserves_geometry_and_layer_order():
    raw_paths = [
        Toolpath(
            points=[Point(0.0, 8.0), Point(0.0, 10.0)],
            kind="fill-infill",
            closed=False,
            metadata={"source_component_id": 1},
        ),
        Toolpath(
            points=[Point(10.0, 0.0), Point(10.0, 2.0)],
            kind="fill-infill",
            closed=False,
            metadata={"source_component_id": 2},
        ),
        Toolpath(
            points=[Point(0.0, 0.0), Point(0.0, 2.0)],
            kind="fill-infill",
            closed=False,
            metadata={"source_component_id": 1},
        ),
        Toolpath(
            points=[Point(10.0, 8.0), Point(10.0, 10.0)],
            kind="fill-infill",
            closed=False,
            metadata={"source_component_id": 2},
        ),
        Toolpath(
            points=[Point(-1.0, -1.0), Point(11.0, -1.0), Point(11.0, 11.0), Point(-1.0, 11.0), Point(-1.0, -1.0)],
            kind="outline",
            closed=True,
            metadata={"source_component_id": 1},
        ),
    ]

    optimized, diagnostics = pipeline_core.optimize_post_generation_travel_order(raw_paths)

    assert diagnostics["travel_optimization_mode"] == "final_export_event_stream_ordering"
    assert diagnostics["optimizer_runs_after_path_merging"] is True
    assert diagnostics["optimizer_runs_on_final_export_paths"] is True
    assert diagnostics["preview_uses_optimized_order"] is True
    assert diagnostics["gcode_uses_optimized_order"] is True
    assert diagnostics["uses_surface_mm_for_ordering"] is True
    assert diagnostics["geometry_changed"] is False
    assert diagnostics["path_points_moved"] is False
    assert diagnostics["paths_reordered"] is True
    assert diagnostics["paths_reordered_count"] >= 2
    assert diagnostics["optimized_pen_up_travel_length_mm"] < diagnostics["raw_pen_up_travel_length_mm"]
    assert diagnostics["optimized_longest_pen_up_travel_mm"] < diagnostics["raw_longest_pen_up_travel_mm"]
    assert diagnostics["open_paths_reversed_count"] >= 1
    assert diagnostics["bad_choice_count_after_optimization"] == 0
    assert diagnostics["stale_travel_geometry_removed"] is True
    assert all(path.kind != "outline" for path in optimized[:-1])
    assert optimized[-1].kind == "outline"

    before = Counter(_canonical_geometry_signature(path) for path in raw_paths)
    after = Counter(_canonical_geometry_signature(path) for path in optimized)
    assert before == after


def test_post_generation_travel_optimizer_keeps_preview_and_gcode_path_order_aligned():
    raw_paths = [
        Toolpath(points=[Point(8.0, 0.0), Point(10.0, 0.0)], kind="fill-infill", closed=False),
        Toolpath(points=[Point(0.0, 0.0), Point(2.0, 0.0)], kind="fill-infill", closed=False),
        Toolpath(points=[Point(-1.0, -1.0), Point(11.0, -1.0), Point(11.0, 1.0), Point(-1.0, 1.0), Point(-1.0, -1.0)], kind="outline", closed=True),
    ]
    optimized, diagnostics = pipeline_core.optimize_post_generation_travel_order(raw_paths)
    prepared = pipeline_core.prepare_toolpaths_for_projection(optimized, default_pen_width_mm=0.6)
    projected = pipeline_core.assign_stable_path_ids(
        pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)
    )
    gcode, preview = GcodeService().generate_from_toolpaths(
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

    preview_ids: list[str] = []
    for entry in preview:
        if entry.get("kind") == "travel":
            continue
        effective_id = str(entry.get("chain_path_id") or entry.get("id"))
        if preview_ids and preview_ids[-1] == effective_id:
            continue
        preview_ids.append(effective_id)
    gcode_ids = []
    for line in gcode:
        if not line.startswith("(PATH_START"):
            continue
        path_id = line.split("id=", 1)[1].split(" ", 1)[0]
        gcode_ids.append(path_id)

    assert diagnostics["paths_reordered_count"] >= 1
    assert preview_ids == gcode_ids


def test_text_outline_generated_from_inset_outline_and_emits_gcode_outline_paths():
    _raster, _geometry, mapped, toolpaths, debug = _build_raster_fixture_toolpaths(CAROLIN_FIXTURE)
    actual_outline = _actual_outline_paths(toolpaths)
    projected = pipeline_core.project_toolpaths_to_ball_angles(
        pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.8),
        center_lon_deg=0.0,
        center_lat_deg=0.0,
    )
    projected_actual_outline_ids = {
        str(path.path_id)
        for path in _actual_outline_paths(projected)
        if path.path_id
    }
    gcode_debug: dict[str, object] = {}
    gcode, _preview = GcodeService().generate_from_toolpaths(
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
        debug=gcode_debug,
    )

    contour_debug = debug["contour_offset_debug"]
    assert contour_debug["outline_generation_source"] == "final_outline_offset"
    assert contour_debug["outline_offset_mode"] == "inset_by_pen_radius"
    assert contour_debug["outline_centerline_offset_mm"] == pytest.approx(0.4, abs=1e-6)
    assert actual_outline
    assert contour_debug["outline_paths_using_inset"] + contour_debug["outline_paths_using_detail_fallback"] > 0
    assert contour_debug["max_outline_overflow_mm"] <= 0.35
    assert contour_debug["outline_total_length_mm"] > 0.0
    gcode_path_ids = {
        line.split("id=", 1)[1].split(" ", 1)[0]
        for line in gcode
        if line.startswith("(PATH_START")
    }
    assert projected_actual_outline_ids
    assert projected_actual_outline_ids.intersection(gcode_path_ids)
    assert mapped.metadata["geometry_quality"]["source_mode"] == "raster"


def test_thin_raster_text_components_are_not_skipped_by_outline_generation():
    _raster, _geometry, _mapped, toolpaths, debug = _build_raster_fixture_toolpaths(CAROLIN_FIXTURE)
    assert _actual_outline_paths(toolpaths)
    assert debug["contour_offset_debug"]["thin_components_outlined"] > 0


def test_raster_text_outline_switch_does_not_change_infill_geometry():
    _raster, _geometry, mapped, raster_toolpaths, raster_debug = _build_raster_fixture_toolpaths(CAROLIN_FIXTURE)
    plain_bundle = GeometryBundle(
        outline_segments=list(mapped.outline_segments),
        fill_boundary_segments=list(mapped.fill_boundary_segments),
        detail_segments=list(mapped.detail_segments),
        fill_shapes=list(mapped.fill_shapes),
        printable_geometry=mapped.printable_geometry,
        cutout_geometry=mapped.cutout_geometry,
        metadata={},
    )
    plain_debug: dict[str, object] = {}
    plain_toolpaths = generate_toolpaths(
        plain_bundle,
        enable_fill=True,
        line_width_mm=0.8,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=0.8,
        infill_angle_deg=0.0,
        outline_after_fill=True,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        fill_strategy="contour_offset",
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.0,
        thin_detail_simplify_mm=0.0,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        expensive_coverage_repair=False,
        debug=plain_debug,
    )

    assert raster_debug["contour_offset_debug"]["outline_generation_source"] == "final_outline_offset"
    assert plain_debug["contour_offset_debug"]["outline_generation_source"] == "final_outline_offset"
    assert _normalized_path_signature(raster_toolpaths, "fill-infill") == _normalized_path_signature(plain_toolpaths, "fill-infill")
    assert sum(1 for path in raster_toolpaths if path.kind == "fill-infill") == sum(1 for path in plain_toolpaths if path.kind == "fill-infill")


def test_wide_outline_centerline_is_inset_by_pen_radius_and_stays_inside_mask():
    printable = _rect(10.0, 6.0)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.6, outline_after_fill=True)

    outline_paths = [path for path in toolpaths if path.kind == "outline"]
    assert len(outline_paths) == 1
    assert outline_paths[0].metadata["outline_generation_source"] == "final_outline_offset"
    assert float(outline_paths[0].metadata["outline_centerline_offset_mm"]) == pytest.approx(0.3, abs=1e-6)
    assert float(outline_paths[0].metadata["outline_offset_mm"]) == pytest.approx(-0.3, abs=1e-6)

    outline_line = _line_for_path(outline_paths[0])
    outline_footprint = _paths_footprint_geometry(outline_paths, pen_width_mm=0.6)
    assert outline_line.distance(printable.boundary) == pytest.approx(0.3, abs=0.02)
    assert outline_footprint.difference(printable).area == pytest.approx(0.0, abs=1e-6)


def test_hole_outline_stays_inside_filled_region_after_inset():
    outer = _rect(12.0, 12.0)
    hole = Polygon([(4.0, 4.0), (8.0, 4.0), (8.0, 8.0), (4.0, 8.0)])
    printable = outer.difference(hole)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.6, outline_after_fill=True)

    outer_paths = [path for path in toolpaths if path.kind == "outline" and not path.metadata.get("is_hole")]
    hole_paths = [path for path in toolpaths if path.kind == "outline" and path.metadata.get("is_hole")]
    assert outer_paths
    assert hole_paths

    outer_footprint = _paths_footprint_geometry(outer_paths, pen_width_mm=0.6)
    hole_footprint = _paths_footprint_geometry(hole_paths, pen_width_mm=0.6)
    assert outer_footprint.difference(printable).area == pytest.approx(0.0, abs=1e-6)
    assert hole_footprint.intersection(hole).area == pytest.approx(0.0, abs=1e-6)


def test_narrow_component_uses_fill_fallback_without_raw_boundary_outline():
    printable = Polygon([
        (0.0, 0.0),
        (0.45, 0.0),
        (0.45, 4.0),
        (0.0, 4.0),
    ])
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert any(path.kind == "fill-infill" for path in toolpaths) is False
    assert len(_actual_outline_paths(toolpaths)) == 1
    contour_debug = debug["contour_offset_debug"]
    assert contour_debug["collapsed_outline_components"] >= 1
    assert contour_debug["outline_paths_using_detail_fallback"] >= 1
    assert contour_debug["outline_overflow_area_mm2"] <= 2.0
    assert contour_debug["max_outline_overflow_mm"] <= 0.2


@pytest.mark.parametrize("width_mm", [0.6, 0.7, 0.8])
def test_one_pen_wide_passage_keeps_a_single_outline_fallback_trace(width_mm: float):
    printable = Polygon([
        (0.0, 0.0),
        (8.0, 0.0),
        (8.0, width_mm),
        (0.0, width_mm),
    ])
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    actual_outline = _actual_outline_paths(toolpaths)
    assert actual_outline
    contour_debug = debug["contour_offset_debug"]
    assert any(bool((path.metadata or {}).get("actual_outline_centerline", False)) for path in actual_outline)
    assert contour_debug["outline_paths_using_detail_fallback"] >= 1
    assert len(actual_outline) == 1
    assert pipeline_core.segment_length(actual_outline[0].points) >= 7.0
    assert not any(path.kind == "fill-repair" for path in toolpaths)


def test_slightly_wider_passage_keeps_valid_outline_centerlines():
    printable = Polygon([
        (0.0, 0.0),
        (8.0, 0.0),
        (8.0, 0.9),
        (0.0, 0.9),
    ])
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    actual_outline = _actual_outline_paths(toolpaths)
    assert len(actual_outline) >= 1
    contour_debug = debug["contour_offset_debug"]
    assert contour_debug["outline_paths_using_inset"] >= 1
    assert contour_debug["outline_overflow_area_mm2"] <= 0.01


def test_overlap_metrics_use_actual_final_outline_footprint():
    printable = Polygon([
        (0.0, 0.0),
        (8.0, 0.0),
        (8.0, 0.6),
        (0.0, 0.6),
    ])
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    actual_outline = _actual_outline_paths(toolpaths)
    fill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    actual_outline_footprint = coverage_planner._paths_footprint_union(actual_outline, pen_radius_mm=0.3)
    fill_footprint = coverage_planner._paths_footprint_union(fill_paths, pen_radius_mm=0.3)
    expected_overlap = float(fill_footprint.intersection(actual_outline_footprint).area)

    report = debug["coverage_report"]
    assert report["infill_outline_overlap_area_mm2"] == pytest.approx(expected_overlap, abs=1e-6)
    assert report["infill_beyond_outline_after_mm2"] == pytest.approx(0.0, abs=1e-6)


def test_long_connected_narrow_corridor_prefers_one_continuous_fallback_path():
    printable = Polygon([
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 0.8),
        (0.0, 0.8),
    ])
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    actual_outline = _actual_outline_paths(toolpaths)
    assert len(actual_outline) == 1
    assert actual_outline[0].metadata["path_role"] in {"FINAL_OUTLINE_FALLBACK", "PRINT_DETAIL"}
    assert pipeline_core.segment_length(actual_outline[0].points) >= 18.0
    assert not any(path.kind == "fill-repair" for path in toolpaths)


def test_final_output_emits_outline_class_fallback_for_thin_component():
    printable = Polygon([
        (0.0, 0.0),
        (0.45, 0.0),
        (0.45, 4.0),
        (0.0, 4.0),
    ])

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
    )

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert any(not path.closed for path in outline_paths)


def test_final_output_preserves_acute_triangle_tip_with_outline_fallback():
    printable = Polygon([
        (0.0, 0.0),
        (8.0, 0.0),
        (0.2, 0.2),
    ])
    tip = (0.2, 0.2)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
    )

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert _nearest_path_point_distance(outline_paths, tip, kinds={"outline", "detail-trace"}) <= 0.18


def test_final_output_preserves_narrow_ring_hole_with_outline_or_centerline_fallback():
    printable = ShapelyPoint(0.0, 0.0).buffer(4.0, resolution=64).difference(
        ShapelyPoint(0.0, 0.0).buffer(3.7, resolution=64)
    )
    probe = (3.85, 0.0)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
    )

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert _nearest_path_point_distance(outline_paths, probe, kinds={"outline", "detail-trace"}) <= 0.25


def test_final_output_preserves_thin_spoke_tip_with_outline_fallback():
    printable = unary_union([
        Polygon([(4.0, 4.0), (8.0, 4.0), (8.0, 8.0), (4.0, 8.0)]),
        Polygon([(5.0, 8.0), (5.3, 8.0), (5.3, 14.0), (5.0, 14.0)]),
    ])
    tip = (5.15, 14.0)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
    )

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert _nearest_path_point_distance(outline_paths, tip, kinds={"outline", "detail-trace"}) <= 0.35


def test_raster_hole_outline_preserves_inner_counters_without_crossing_them():
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    geometry = GeometryService()
    toolpath_service = ToolpathService()
    mask = raster.build_mask(_make_logo_bytes(), ["#000000"], tolerance=8, min_component_area_px=0, open_radius_px=0, close_radius_px=0)
    regions = raster.extract_regions(mask, min_region_area_px=10, simplify_tolerance_px=0)
    mapped = geometry.map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    debug: dict[str, object] = {}
    toolpaths = toolpath_service.generate_from_regions(
        mapped,
        pen_width_mm=0.75,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=0.75,
        infill_density=100.0,
        infill_angle_deg=0.0,
        fill_strategy="contour_offset",
        outline_after_fill=True,
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.0,
        thin_detail_simplify_mm=0.0,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        expensive_coverage_repair=False,
        debug=debug,
    )
    component_holes: dict[str, list[Polygon]] = {}
    for component_index, poly in enumerate(pipeline_core.normalize_geometry(mapped.printable_geometry), start=1):
        component_holes[f"component_{component_index:03d}"] = [Polygon(ring) for ring in poly.interiors]
    inner_outlines = [path for path in toolpaths if path.kind == "outline" and str((path.metadata or {}).get("ring_role", "")) == "hole"]
    assert inner_outlines
    assert debug["contour_offset_debug"]["outline_paths_generated"] >= len(inner_outlines)
    for path in inner_outlines:
        line = _line_for_path(path)
        source_region_id = str((path.metadata or {}).get("source_region_id", ""))
        matching_holes = component_holes.get(source_region_id, [])
        assert matching_holes
        assert any(not hole.buffer(-0.01).crosses(line) for hole in matching_holes)


def test_ha_raster_fixture_keeps_positive_infill_and_outline_output():
    _raster, _geometry, _mapped, toolpaths, debug = _build_raster_fixture_toolpaths(HA_FIXTURE)
    assert any(path.kind == "fill-infill" for path in toolpaths)
    assert any(path.kind == "outline" for path in toolpaths)
    assert debug["contour_offset_debug"]["outline_paths_generated"] > 0
    assert debug["contour_offset_debug"]["outline_total_length_mm"] > 0.0


def test_gcode_audit_has_no_legacy_detail_trace_path_start():
    toolpaths = _generate_fill_toolpaths(
        _rect(width_mm=8.0, height_mm=8.0),
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.6)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)
    gcode, _preview = GcodeService().generate_from_toolpaths(
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
        debug={},
    )
    text = "\n".join(gcode)
    assert "kind=detail-trace" not in text
    assert "kind=detail-continuation" not in text


def test_central_cross_junction_accepts_small_contour_sections():
    vertical = Polygon([(-1.2, -6.0), (1.2, -6.0), (1.2, 6.0), (-1.2, 6.0)])
    horizontal = Polygon([(-6.0, -1.2), (6.0, -1.2), (6.0, 1.2), (-6.0, 1.2)])
    junction = vertical.union(horizontal)
    debug: dict[str, object] = {}
    toolpaths = _generate_fill_toolpaths(
        junction,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )
    contour_dbg = (debug.get("contour_offset_debug", {}) or {})
    audit = (debug.get("gcode_generation_audit", {}) or {})
    forbidden = {"detail-trace", "detail-continuation", "collapse-centerline", "gap-repair-dab", "hatch", "adaptive"}

    assert int(contour_dbg.get("central_junction_candidate_sections_found", 0)) > 0
    assert int(contour_dbg.get("central_junction_sections_accepted", 0)) > 0
    assert float(contour_dbg.get("remaining_uncovered_area_mm2_after", 0.0)) < float(contour_dbg.get("remaining_uncovered_area_mm2_before", 1e9))
    assert float(contour_dbg.get("remaining_uncovered_area_ratio_after", 1.0)) <= 0.02
    assert any(
        str((p.metadata or {}).get("path_role", "")) in {"ISOLATED_CONTOUR_SECTION", "CONTOUR_SECTION_INFILL", "CROSSED_CONTOUR_SECTION"}
        for p in toolpaths
    )
    assert not any(p.kind in forbidden for p in toolpaths)
    assert int(audit.get("legacy_kinds_forbidden_count", 0)) == 0


def test_ha_fixture_contour_sections_reduce_uncovered_area():
    if not HA_FIXTURE.exists():
        pytest.skip("HA fixture missing")
    image_bytes = HA_FIXTURE.read_bytes()
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((color.id for color in analysis.colors if color.hex == "#000000"), analysis.colors[0].id if analysis.colors else None)
    assert selected is not None
    mask = raster.build_mask(image_bytes, [selected], tolerance=24, min_component_area_px=0, open_radius_px=0, close_radius_px=1)
    regions = raster.extract_regions(mask, min_region_area_px=8, simplify_tolerance_px=1.0)
    mapped = GeometryService().map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)
    debug: dict[str, object] = {}
    toolpaths = ToolpathService().generate_from_regions(
        mapped,
        pen_width_mm=0.6,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=0.6,
        infill_density=100.0,
        infill_angle_deg=0.0,
        fill_strategy="contour_offset",
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        infill_path_mode="rectilinear",
        debug=debug,
    )
    contour_dbg = (debug.get("contour_offset_debug", {}) or {})
    assert any(str((p.metadata or {}).get("path_role", "")) in {"CONTOUR_SECTION_INFILL", "ISOLATED_CONTOUR_SECTION", "CROSSED_CONTOUR_SECTION", "CORNER_CONTOUR_SECTION"} for p in toolpaths)
    assert float(contour_dbg.get("remaining_uncovered_area_mm2_after", 1.0)) <= float(contour_dbg.get("remaining_uncovered_area_mm2_before", 0.0))
    assert int(contour_dbg.get("central_junction_candidate_sections_found", 0)) >= 0


def test_contour_fill_covers_entire_mask_without_visible_gaps():
    if not HA_FIXTURE.exists():
        pytest.skip("HA fixture missing")
    image_bytes = HA_FIXTURE.read_bytes()
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((color.id for color in analysis.colors if color.hex == "#000000"), analysis.colors[0].id if analysis.colors else None)
    assert selected is not None
    mask = raster.build_mask(image_bytes, [selected], tolerance=24, min_component_area_px=0, open_radius_px=0, close_radius_px=1)
    regions = raster.extract_regions(mask, min_region_area_px=8, simplify_tolerance_px=1.0)
    mapped = GeometryService().map_bundle_to_angles(regions.bundle, regions.bounds, "contain", True, 4.0)

    debug: dict[str, object] = {}
    pen_width_mm = 0.6
    toolpaths = ToolpathService().generate_from_regions(
        mapped,
        pen_width_mm=pen_width_mm,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=pen_width_mm,
        infill_density=100.0,
        infill_angle_deg=0.0,
        fill_strategy="contour_offset",
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=True,
        small_shape_mode="single-wall",
        thin_detail_mode=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        infill_path_mode="rectilinear",
        debug=debug,
    )

    allowed_roles = {
        "CONTOUR_INFILL",
        "CONTOUR_SECTION_INFILL",
        "CORRIDOR_CONTOUR_SECTION",
        "CORNER_CONTOUR_SECTION",
        "ISOLATED_CONTOUR_SECTION",
        "CROSSED_CONTOUR_SECTION",
        "FINAL_OUTER_OUTLINE",
        "FINAL_INNER_OUTLINE",
    }
    forbidden_kinds = {"detail-trace", "detail-continuation", "hatch", "adaptive", "fill-infill-travel", "coverage_connector", "collapse-centerline"}
    forbidden_legacy_path_count = int(sum(1 for p in toolpaths if p.kind in forbidden_kinds))
    assert forbidden_legacy_path_count == 0
    for p in toolpaths:
        if p.kind not in {"fill-infill", "outline"}:
            continue
        role = str((p.metadata or {}).get("path_role", ""))
        assert role in allowed_roles

    outlines = [idx for idx, p in enumerate(toolpaths) if p.kind == "outline"]
    infills = [idx for idx, p in enumerate(toolpaths) if p.kind == "fill-infill"]
    assert outlines and infills
    outline_last = min(outlines) > max(infills)
    assert outline_last

    validation_px_per_mm = 36.0
    geom = mapped.printable_geometry
    assert geom is not None and not geom.is_empty
    footprint_geom = _paths_footprint_geometry([p for p in toolpaths if p.kind in {"fill-infill", "outline"}], pen_width_mm=pen_width_mm)
    bounds = geom.bounds
    target_mask = _geom_to_mask(geom, bounds, validation_px_per_mm)
    rendered_mask = _geom_to_mask(footprint_geom, bounds, validation_px_per_mm)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    target_core = cv2.erode(target_mask, kernel, iterations=1)
    uncovered = cv2.bitwise_and(target_core, cv2.bitwise_not(rendered_mask))
    overspill = cv2.bitwise_and(rendered_mask, cv2.bitwise_not(target_mask))

    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats((uncovered > 0).astype(np.uint8), connectivity=8)
    px_area_mm2 = 1.0 / (validation_px_per_mm * validation_px_per_mm)
    component_areas_mm2 = [float(stats[i, cv2.CC_STAT_AREA]) * px_area_mm2 for i in range(1, n_labels)]
    remaining_uncovered_pixel_count = int(np.count_nonzero(uncovered))
    remaining_uncovered_area_mm2 = float(remaining_uncovered_pixel_count) * px_area_mm2
    target_area_mm2 = float(np.count_nonzero(target_core)) * px_area_mm2
    remaining_uncovered_area_ratio = remaining_uncovered_area_mm2 / max(1e-12, target_area_mm2)
    max_uncovered_component_area_mm2 = max(component_areas_mm2) if component_areas_mm2 else 0.0
    overspill_area_mm2 = float(np.count_nonzero(overspill)) * px_area_mm2
    overspill_area_ratio = overspill_area_mm2 / max(1e-12, target_area_mm2)

    contour_dbg = (debug.get("contour_offset_debug", {}) or {}) if isinstance(debug, dict) else {}
    gcode_audit = (debug.get("gcode_generation_audit", {}) or {}) if isinstance(debug, dict) else {}
    if max_uncovered_component_area_mm2 > 0.025 or remaining_uncovered_area_ratio > 0.001:
        out_dir = ROOT / "artifacts" / "tests" / "coverage_ha"
        out_dir.mkdir(parents=True, exist_ok=True)
        Image.fromarray(target_mask).save(out_dir / "target_mask.png")
        Image.fromarray(rendered_mask).save(out_dir / "rendered_pen_footprint.png")
        Image.fromarray(uncovered).save(out_dir / "uncovered_residual.png")
        Image.fromarray(overspill).save(out_dir / "overspill.png")
        preview = np.zeros((target_mask.shape[0], target_mask.shape[1], 3), dtype=np.uint8)
        preview[:, :, 1] = target_mask
        preview[:, :, 0] = rendered_mask
        preview[:, :, 2] = uncovered
        Image.fromarray(preview).save(out_dir / "toolpath_preview.png")
        residual_component_debug = []
        for i in range(1, n_labels):
            area_mm2 = float(stats[i, cv2.CC_STAT_AREA]) * px_area_mm2
            bb = [
                int(stats[i, cv2.CC_STAT_LEFT]),
                int(stats[i, cv2.CC_STAT_TOP]),
                int(stats[i, cv2.CC_STAT_WIDTH]),
                int(stats[i, cv2.CC_STAT_HEIGHT]),
            ]
            residual_component_debug.append({"component_id": i, "area_mm2": area_mm2, "bounding_box_px": bb})
        pytest.fail(
            f"coverage invariant failed: uncovered_px={remaining_uncovered_pixel_count} "
            f"remaining_uncovered_area_mm2={remaining_uncovered_area_mm2:.6f} "
            f"remaining_uncovered_area_ratio={remaining_uncovered_area_ratio:.6f} "
            f"max_component_mm2={max_uncovered_component_area_mm2:.6f} "
            f"overspill_mm2={overspill_area_mm2:.6f} overspill_ratio={overspill_area_ratio:.6f} "
            f"forbidden_legacy_path_count={forbidden_legacy_path_count} outline_last={outline_last} "
            f"component_debug={residual_component_debug[:12]} "
            f"repair_logs={contour_dbg.get('coverage_repair_logs', [])} "
            f"audit={gcode_audit}"
        )


def test_inner_corner_turn_is_preserved_by_corner_sections():
    outer = Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)])
    notch = Polygon([(6.8, 6.8), (10.0, 6.8), (10.0, 10.0), (6.8, 10.0)])
    shape = outer.difference(notch)
    debug: dict[str, object] = {}
    toolpaths = _generate_fill_toolpaths(
        shape,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )
    contour_dbg = (debug.get("contour_offset_debug", {}) or {})
    audit = (debug.get("gcode_generation_audit", {}) or {})
    forbidden = {"detail-trace", "detail-continuation", "collapse-centerline", "gap-repair-dab", "hatch", "adaptive"}

    corner_paths = [p for p in toolpaths if str((p.metadata or {}).get("path_role", "")) == "CORNER_CONTOUR_SECTION"]
    assert int(contour_dbg.get("corner_candidate_count_total", 0)) >= 0
    assert int(contour_dbg.get("corner_accepted_section_count_total", 0)) >= 0
    corner_logs = list(contour_dbg.get("corner_candidate_logs", []) or [])
    assert corner_logs
    assert all(float(entry.get("iso_distance_error_max_mm", 1e9)) <= 0.08 for entry in corner_logs if entry.get("accepted_or_rejected") == "accepted")
    assert float(contour_dbg.get("remaining_uncovered_area_ratio_after", 1.0)) <= 0.03
    if corner_paths:
        assert int(contour_dbg.get("corner_accepted_section_count_total", 0)) > 0
    assert int(audit.get("legacy_kinds_forbidden_count", 1)) == 0
    assert not any(p.kind in forbidden for p in toolpaths)


def test_corridor_corner_rejects_diagonal_shortcut_and_keeps_parallel_repair():
    # Long corridor with inner corner; diagonal chords across the corner are invalid.
    shell = Polygon([(0.0, 0.0), (14.0, 0.0), (14.0, 3.0), (5.0, 3.0), (5.0, 12.0), (0.0, 12.0)])
    cut = Polygon([(1.5, 1.5), (12.5, 1.5), (12.5, 2.1), (4.1, 2.1), (4.1, 10.5), (1.5, 10.5)])
    shape = shell.difference(cut)
    debug: dict[str, object] = {}
    toolpaths = _generate_fill_toolpaths(
        shape,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )
    contour_dbg = (debug.get("contour_offset_debug", {}) or {})
    rejected_counts = dict(contour_dbg.get("corner_rejection_reason_counts", {}) or {})
    corner_logs = list(contour_dbg.get("corner_candidate_logs", []) or [])
    accepted_corner_logs = [entry for entry in corner_logs if entry.get("accepted_or_rejected") == "accepted"]
    forbidden = {"detail-trace", "detail-continuation", "collapse-centerline", "hatch", "adaptive"}

    assert int(contour_dbg.get("corner_candidate_count_total", 0)) > 0
    assert int(contour_dbg.get("corner_accepted_section_count_total", 0)) > 0
    assert all(float(entry.get("parallel_angle_error_deg", 999.0)) <= 60.0 for entry in accepted_corner_logs)
    assert all(float(entry.get("iso_distance_error_max_mm", 999.0)) <= 0.08 for entry in accepted_corner_logs)
    corner_paths = [p for p in toolpaths if str((p.metadata or {}).get("path_role", "")) == "CORNER_CONTOUR_SECTION"]
    assert corner_paths
    # Ensure accepted corner sections are contour-following, not straight chords.
    for cp in corner_paths:
        line = _line_for_path(cp)
        chord = math.hypot(cp.points[-1].x - cp.points[0].x, cp.points[-1].y - cp.points[0].y)
        assert len(cp.points) >= 3 or float(line.length) > (chord * 1.05)
    assert float(contour_dbg.get("remaining_uncovered_area_ratio_after", 1.0)) <= 0.04
    assert not any(p.kind in forbidden for p in toolpaths)


def test_contour_offset_levels_use_original_mask_and_keep_corner_turn_geometry():
    shell = Polygon([(0.0, 0.0), (16.0, 0.0), (16.0, 4.0), (6.0, 4.0), (6.0, 14.0), (0.0, 14.0)])
    cut = Polygon([(1.2, 1.2), (14.8, 1.2), (14.8, 2.6), (4.6, 2.6), (4.6, 12.8), (1.2, 12.8)])
    shape = shell.difference(cut)
    debug: dict[str, object] = {}
    toolpaths = _generate_fill_toolpaths(
        shape,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )
    contour_dbg = (debug.get("contour_offset_debug", {}) or {})
    levels = list(contour_dbg.get("levels", []) or [])
    corner_logs = list(contour_dbg.get("corner_candidate_logs", []) or [])

    assert levels
    assert float(contour_dbg.get("contour_overlap_spacing_factor", 0.0)) >= 0.60
    assert float(contour_dbg.get("contour_overlap_spacing_factor", 1.0)) <= 0.70
    assert all("max_iso_distance_error_mm" in lvl for lvl in levels)
    assert all("diagonal_shortcut_rejected_count" in lvl for lvl in levels)
    assert int(contour_dbg.get("diagonal_shortcut_rejected_count", 0)) >= 0
    assert int(contour_dbg.get("corner_accepted_section_count_total", 0)) >= 0
    if any(entry.get("accepted_or_rejected") == "accepted" for entry in corner_logs):
        assert all(float(entry.get("iso_distance_error_max_mm", 999.0)) <= 0.06 for entry in corner_logs if entry.get("accepted_or_rejected") == "accepted")
    assert float(contour_dbg.get("remaining_uncovered_area_ratio_after", 1.0)) <= 0.04
    assert any(str((p.metadata or {}).get("path_role", "")) in {"CONTOUR_INFILL", "CONTOUR_SECTION_INFILL", "CORNER_CONTOUR_SECTION"} for p in toolpaths)


def test_long_horizontal_corridor_contour_continuity_is_restored():
    bar = Polygon([(0.0, 0.0), (24.0, 0.0), (24.0, 3.2), (0.0, 3.2)])
    branch = Polygon([(12.0, 3.2), (15.0, 3.2), (15.0, 10.0), (12.0, 10.0)])
    notch = Polygon([(1.0, 1.0), (23.0, 1.0), (23.0, 2.2), (1.0, 2.2)])
    shape = bar.union(branch).difference(notch)
    debug: dict[str, object] = {}
    toolpaths = _generate_fill_toolpaths(
        shape,
        line_width_mm=0.6,
        fill_strategy="contour_offset",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )
    contour_dbg = (debug.get("contour_offset_debug", {}) or {})
    per_levels = list(contour_dbg.get("levels", []) or [])
    audit = (debug.get("gcode_generation_audit", {}) or {})
    forbidden = {"detail-trace", "detail-continuation", "collapse-centerline", "hatch", "adaptive", "fill-infill-travel", "coverage_connector", "gap-repair-centerline"}

    assert per_levels
    assert int(contour_dbg.get("open_section_audit_failures", 1)) == 0
    assert any(bool(entry.get("horizontal_section_expected", False)) for entry in per_levels)
    assert any(bool(entry.get("horizontal_section_present", False)) for entry in per_levels)
    assert any(int(entry.get("restored_continuity_section_count", 0)) >= 0 for entry in per_levels)
    assert not any(p.kind in forbidden for p in toolpaths)
    assert int(audit.get("legacy_kinds_forbidden_count", 1)) == 0


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

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert all(bool(path.metadata.get("actual_outline_centerline", False)) for path in outline_paths)
    assert all(path.metadata.get("small_detail_fill_style") == "single_stroke_detail" for path in outline_paths)
    assert all(len(path.points) >= 2 for path in outline_paths)
    assert not any(path.kind == "fill-wall" for path in toolpaths)


def test_tiny_blob_smaller_than_pen_uses_single_stroke_fallback():
    printable = _rect(width_mm=0.35, height_mm=0.35)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    outline_paths = [path for path in toolpaths if path.kind == "outline"]
    assert len(outline_paths) <= 1
    assert not any(path.kind == "fill-wall" for path in toolpaths)


def test_thin_band_uses_single_stroke_without_hatch_rows():
    printable = _rect(width_mm=8.0, height_mm=0.45)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert any(path.metadata.get("small_detail_fill_style") == "single_stroke_detail" for path in outline_paths)
    assert not any(path.kind == "fill-wall" for path in toolpaths)
    assert all(len(path.points) >= 2 for path in outline_paths)


def test_mixed_logo_routes_large_regions_and_tiny_regions_differently():
    large = _rect(width_mm=16.0, height_mm=10.0)
    tiny = _rect(width_mm=0.35, height_mm=0.35)
    printable = MultiPolygon([large, tiny])

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    fill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    outline_paths = [path for path in toolpaths if path.kind == "outline"]
    assert fill_paths
    assert outline_paths
    assert any(path.metadata.get("fill_strategy") == "RECTILINEAR_SERPENTINE" for path in fill_paths)
    assert not any(path.kind == "fill-wall" for path in toolpaths)


def test_outline_collapse_is_conditional_not_global():
    line_width = 0.6
    wide = _rect(width_mm=12.0, height_mm=8.0)
    thin = _rect(width_mm=8.0, height_mm=0.8)
    tiny = _rect(width_mm=0.35, height_mm=0.35)
    printable = MultiPolygon([wide, thin, tiny])

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=line_width,
        infill_spacing_mm=line_width,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )

    outlines = [p for p in toolpaths if p.kind == "outline"]
    fills = [p for p in toolpaths if p.kind == "fill-infill"]
    assert outlines, "normal/wide regions should keep outlines"
    assert any(p.metadata.get("small_detail_fill_style") == "single_stroke_detail" for p in fills), "thin regions should collapse to centerline"
    assert len(outlines) < len(toolpaths), "not all regions should become outline-only"


def test_simple_rectangle_infill_is_rectilinear_without_zigzag_connectors():
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
    assert len(infill_paths) >= 2
    assert all(len(path.points) >= 2 for path in infill_paths)
    assert all(len(path.points) <= 3 for path in infill_paths)
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


def _s_stroke_geometry() -> Polygon:
    centerline = LineString([
        (1.0, 8.5),
        (3.0, 9.2),
        (5.5, 8.3),
        (7.2, 6.7),
        (5.8, 5.0),
        (3.1, 4.1),
        (1.6, 2.4),
        (3.5, 0.8),
        (6.8, 1.0),
    ])
    return centerline.buffer(0.55, join_style=1, cap_style=1)


def test_narrow_s_shape_prefers_adaptive_detail_contour_fill():
    printable = _s_stroke_geometry()
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=0.0,
        outline_after_fill=True,
        infill_path_mode="rectilinear",
        debug=debug,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert any(path.metadata.get("fill_mode") == "detail_contour_cell" for path in infill_paths)
    assert not all(path.kind == "fill-infill-travel" for path in infill_paths)
    outline_indices = [index for index, path in enumerate(toolpaths) if path.kind == "outline"]
    infill_indices = [index for index, path in enumerate(toolpaths) if path.kind == "fill-infill"]
    assert outline_indices and infill_indices
    assert max(outline_indices) > min(infill_indices)
    adaptive_counts = (debug.get("infill_debug", {}) or {}).get("adaptive_fill_counts", {})
    assert int(adaptive_counts.get("detail_contour_cells", 0)) >= 1


def test_mixed_regions_keep_rectilinear_for_wide_and_detail_for_narrow():
    wide = _rect(12.0, 8.0)
    narrow = _s_stroke_geometry().buffer(0.0)
    printable = MultiPolygon([wide, narrow])
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        infill_path_mode="rectilinear",
        debug=debug,
    )
    infill_modes = _fill_modes([path for path in toolpaths if path.kind == "fill-infill"])
    assert any(mode in infill_modes for mode in {"large_open", "long_thin"})
    adaptive_counts = (debug.get("infill_debug", {}) or {}).get("adaptive_fill_counts", {})
    assert int(adaptive_counts.get("rectilinear_cells", 0)) >= 1


def test_projection_center_latitude_is_clamped_to_keep_y_in_bounds():
    surface_path = Toolpath(
        points=[Point(0.0, -15.0), Point(10.0, 15.0)],
        kind="fill-infill",
        coordinate_space="surface_mm",
    )
    resolved, info = pipeline_core.resolve_safe_projection_center_lat(
        [surface_path],
        requested_center_lat_deg=60.0,
        ball_diameter_mm=42.67,
        y_draw_min_deg=-45.0,
        y_draw_max_deg=45.0,
    )
    assert info["auto_clamped"] is True
    assert resolved <= float(info["allowed_center_lat_max_deg"]) + 1e-9
    projected = pipeline_core.project_toolpaths_to_ball_angles(
        [surface_path],
        center_lon_deg=0.0,
        center_lat_deg=resolved,
        ball_diameter_mm=42.67,
    )
    ys = [point.y for path in projected for point in path.points]
    assert max(ys) <= 45.0 + 1e-6
    assert min(ys) >= -45.0 - 1e-6


def test_surface_toolpaths_auto_scale_to_y_band_when_too_tall():
    path = Toolpath(
        points=[Point(0.0, -30.0), Point(0.0, 30.0)],
        kind="fill-infill",
        coordinate_space="surface_mm",
    )
    scaled, info = pipeline_core.fit_surface_toolpaths_to_y_band(
        [path],
        ball_diameter_mm=42.67,
        y_draw_min_deg=-45.0,
        y_draw_max_deg=45.0,
    )
    assert bool(info["auto_scaled"]) is True
    assert float(info["scale_factor"]) < 1.0
    projected = pipeline_core.project_toolpaths_to_ball_angles(
        scaled,
        center_lon_deg=0.0,
        center_lat_deg=0.0,
        ball_diameter_mm=42.67,
    )
    ys = [point.y for toolpath in projected for point in toolpath.points]
    assert max(ys) <= 45.0 + 1e-6
    assert min(ys) >= -45.0 - 1e-6


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
    assert len(infill_paths) > 10
    assert all(len(path.points) >= 2 for path in infill_paths)
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable))


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


def test_broken_rows_are_split_into_multiple_cells_for_hole_shape():
    outer = _rect(24.0, 24.0)
    hole = _rect(8.0, 8.0)
    hole = Polygon([(point[0] + 8.0, point[1] + 8.0) for point in hole.exterior.coords[:-1]])
    printable = Polygon(outer.exterior.coords, [hole.exterior.coords])
    debug: dict = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        allow_pen_down_infill_connectors=True,
        infill_path_mode="serpentine_optimized",
        debug=debug,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    hole_region = _rect(8.0, 8.0)
    hole_region = Polygon([(point[0] + 8.0, point[1] + 8.0) for point in hole_region.exterior.coords[:-1]])

    assert infill_paths
    assert any(path.metadata.get("fill_strategy") in {"RECTILINEAR_SERPENTINE", "CONTOUR_PARALLEL_DETAIL"} for path in infill_paths)
    assert int((debug.get("infill_debug", {}) or {}).get("adaptive_fill_counts", {}).get("detail_contour_cells", 0)) >= 0
    _assert_infill_segments_stay_inside_region(infill_paths, printable.buffer(-0.01))
    for path in infill_paths:
        for start, end in zip(path.points, path.points[1:]):
            assert not hole_region.buffer(-0.01).covers(LineString([(start.x, start.y), (end.x, end.y)]))


def test_letter_like_counter_shape_uses_pen_up_between_cells():
    # Rectangle with internal counter and narrow bridge to mimic letter-like cavities.
    outer = _rect(26.0, 24.0)
    counter = Polygon([(x + 8.0, y + 8.0) for x, y in _rect(10.0, 8.0).exterior.coords[:-1]])
    notch = Polygon([(x + 17.0, y + 10.0) for x, y in _rect(5.0, 4.0).exterior.coords[:-1]])
    printable = Polygon(outer.exterior.coords, [counter.exterior.coords]).difference(notch)

    debug: dict = {}
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        allow_pen_down_infill_connectors=True,
        infill_path_mode="serpentine_optimized",
        debug=debug,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert debug.get("rows_with_multiple_intervals", 0) > 0
    assert debug.get("rejected_cross_gap_connectors", 0) >= 0
    counter_cover = counter.buffer(-0.01)
    for path in infill_paths:
        for start, end in zip(path.points, path.points[1:]):
            assert not counter_cover.covers(LineString([(start.x, start.y), (end.x, end.y)]))


def test_bifurcation_rows_do_not_merge_into_single_cell():
    # Shape that creates a single span that splits into two spans around a wedge void.
    outer = _rect(28.0, 22.0)
    wedge_void = Polygon([
        (6.0, 8.0),
        (18.0, 8.0),
        (23.0, 13.0),
        (18.0, 18.0),
        (6.0, 18.0),
        (11.0, 13.0),
    ])
    printable = Polygon(outer.exterior.coords, [wedge_void.exterior.coords])
    debug: dict = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        allow_pen_down_infill_connectors=True,
        infill_path_mode="serpentine_optimized",
        debug=debug,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]

    assert infill_paths
    assert debug.get("rows_with_multiple_intervals", 0) > 0
    assert debug.get("local_cell_count", 0) >= 2
    assert debug.get("pen_lifts_after_cell_planning", 0) >= 0


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


def test_infill_path_mode_rectilinear_forces_separate_rows_even_when_connectors_enabled():
    printable = Polygon([
        (0.0, 0.0),
        (16.0, 0.0),
        (12.0, 18.0),
        (4.0, 18.0),
    ])
    debug: dict = {}
    toolpaths = _generate_fill_toolpaths(
        printable,
        allow_pen_down_infill_connectors=True,
        infill_path_mode="rectilinear",
        debug=debug,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert len(infill_paths) > 1
    assert debug.get("infill_debug", {}).get("infill_path_mode") == "rectilinear"
    assert debug.get("infill_debug", {}).get("allow_pen_down_infill_connectors") is True
    assert debug.get("local_cell_count", 0) >= 1


def test_infill_path_mode_serpentine_optimized_keeps_connector_attempts_and_reports_rejections():
    printable = Polygon([
        (0.0, 0.0),
        (24.0, 0.0),
        (24.0, 12.0),
        (0.0, 12.0),
    ])
    debug: dict = {}
    _generate_fill_toolpaths(
        printable,
        allow_pen_down_infill_connectors=True,
        infill_path_mode="serpentine_optimized",
        debug=debug,
    )
    reasons = debug.get("connector_rejection_reasons", {})
    assert isinstance(reasons, dict)
    assert set(reasons.keys()).issubset({"too_long", "outside_polygon", "non_adjacent_row"})


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
    assert any(pipeline_core.segment_length(path.points) < 2.0 for path in paths)


def test_rectangle_hatch_offsets_stay_on_even_surface_mm_grid():
    slicer = pipeline_core.SlicerService()
    region = _infill_region(_rect(12.0, 8.0), line_width_mm=0.5)

    row_data = slicer._collect_scanline_rows(
        region,
        spacing_mm=0.5,
        angle_deg=0.0,
        min_segment_length_mm=0.0,
    )
    offsets = _drawable_row_offsets(row_data)
    gaps = [offsets[index] - offsets[index - 1] for index in range(1, len(offsets))]

    assert offsets
    assert min(gaps) == pytest.approx(0.5, abs=1e-6)
    assert max(gaps) == pytest.approx(0.5, abs=1e-6)


def test_angled_notch_hatch_gap_detection_reinstates_skipped_rows():
    slicer = pipeline_core.SlicerService()
    polygon = Polygon([
        (0.0, 0.0),
        (24.0, 0.0),
        (24.0, 10.0),
        (16.0, 10.0),
        (13.0, 16.0),
        (4.0, 16.0),
        (0.0, 12.0),
    ])
    region = _infill_region(polygon, line_width_mm=0.5)

    row_data = slicer._collect_scanline_rows(
        region,
        spacing_mm=0.5,
        angle_deg=0.0,
        min_segment_length_mm=1.2,
    )
    offsets = _drawable_row_offsets(row_data)
    gaps = [offsets[index] - offsets[index - 1] for index in range(1, len(offsets))]

    assert offsets
    assert max(gaps) <= 0.75 + 1e-6
    assert any(offset >= 10.0 for offset in offsets)


def test_pen_down_connectors_only_join_adjacent_rows_and_do_not_cross_holes():
    printable = Polygon(
        _rect(24.0, 24.0).exterior.coords,
        [[(x + 8.0, y + 8.0) for x, y in _rect(8.0, 8.0).exterior.coords[:-1]]],
    )
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.5, infill_spacing_mm=0.5)
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]

    assert infill_paths
    assert len(infill_paths) >= 2
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable, line_width_mm=0.5))


def test_scanline_infill_order_is_preserved_after_region_planning():
    printable = Polygon([
        (0.0, 0.0),
        (26.0, 0.0),
        (26.0, 12.0),
        (14.0, 12.0),
        (9.0, 18.0),
        (0.0, 18.0),
    ])
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=0.0,
        outline_after_fill=False,
    )
    offsets = [
        float(path.metadata["scanline_offset_mm"])
        for path in toolpaths
        if path.kind == "fill-infill" and "scanline_offset_mm" in path.metadata
    ]

    assert offsets
    assert offsets[:-1] == sorted(offsets[:-1])


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
    assert _fill_modes(infill_paths) == {"detail_contour_cell"}
    assert any(path.metadata.get("fill_strategy") == "CONTOUR_PARALLEL_DETAIL" for path in infill_paths)
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


def test_adaptive_cell_mode_switches_to_single_stroke_for_narrow_fragmented_cell():
    slicer = pipeline_core.SlicerService()
    segments = [
        pipeline_core.InfillSegment(
            id="cell:r0:i0",
            component_id="component_000",
            row_index=0,
            interval_index=0,
            cell_id="component_000:cell_0000",
            scanline_offset=0.0,
            low_u=Point(0.0, 0.0),
            high_u=Point(0.6, 0.0),
            min_u=0.0,
            max_u=0.6,
            center=Point(0.3, 0.0),
            length=0.6,
            coords=[(0.0, 0.0), (0.6, 0.0)],
        ),
    ]

    decision = slicer._evaluate_adaptive_cell_mode(
        cell_segments=segments,
        spacing_mm=0.6,
        line_width_mm=0.6,
        cover_region=_rect(2.0, 2.0),
    )
    assert decision.mode == "single_stroke"
    assert "only_one_useful_hatch_row" in decision.reasons or "width_lte_1p5x_pen" in decision.reasons


def test_infill_debug_reports_single_stroke_and_switch_diagnostics():
    debug: dict = {}
    printable = Polygon([
        (0.0, 0.0),
        (12.0, 0.0),
        (12.0, 0.9),
        (0.0, 0.9),
    ])
    _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        debug=debug,
    )
    infill_debug = debug.get("infill_debug", {})
    assert "diagnostics" in infill_debug
    assert "single_stroke_cells" in infill_debug.get("adaptive_fill_counts", {})
    assert "switched_single_stroke_width" in infill_debug.get("adaptive_fill_counts", {})


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
    assert _fill_modes(infill_paths) == {"detail_contour_cell"}
    assert any(path.metadata.get("fill_strategy") == "CONTOUR_PARALLEL_DETAIL" for path in infill_paths)
    _assert_infill_segments_stay_inside_region(infill_paths, _infill_region(printable, line_width_mm=0.5))
    assert all(not hole.buffer(-0.01).covers(_line_for_path(path)) for path in infill_paths)


def test_arsenal_frontend_defaults_detail_fill_is_rotation_stable(arsenal_frontend_default_90_result):
    detail_counts: dict[int, int] = {}
    detail_coverages: dict[int, float] = {}
    outline_overflows: dict[int, float] = {}
    for rotation_deg in (0, 90):
        if rotation_deg == 90:
            _placed, toolpaths, debug = arsenal_frontend_default_90_result
        else:
            _placed, toolpaths, debug = _frontend_default_arsenal_fixture(rotation_deg=rotation_deg)
        detail_counts[rotation_deg] = sum(1 for path in toolpaths if path.kind == "detail-trace")
        detail_coverages[rotation_deg] = float(debug.get("detail_coverage_after_repair_percent", 0.0))
        outline_overflows[rotation_deg] = float((debug.get("contour_offset_debug") or {}).get("max_outline_overflow_mm", 0.0))

        assert int(debug.get("detail_paths_dropped_as_redundant_overlap", 0)) >= 1
        assert detail_coverages[rotation_deg] >= float(debug.get("required_detail_coverage_percent", 0.0)) - 4.0
        assert _actual_outline_paths(toolpaths)
        assert float((debug.get("contour_offset_debug") or {}).get("outline_centerline_offset_mm", 0.0)) == pytest.approx(0.3, abs=1e-6)
        assert outline_overflows[rotation_deg] <= 0.05

    assert abs(detail_counts[0] - detail_counts[90]) <= 2
    assert abs(detail_coverages[0] - detail_coverages[90]) <= 1.0
    assert abs(outline_overflows[0] - outline_overflows[90]) <= 0.01


def test_arsenal_preview_pipeline_does_not_use_legacy_slicer_backfill(monkeypatch: pytest.MonkeyPatch):
    calls = 0
    original = pipeline_core.SlicerService._enforce_region_coverage_backfill

    def _tracking_backfill(self, paths, **kwargs):
        nonlocal calls
        calls += 1
        return original(self, paths, **kwargs)

    monkeypatch.setattr(pipeline_core.SlicerService, "_enforce_region_coverage_backfill", _tracking_backfill)
    _frontend_default_arsenal_fixture(rotation_deg=90)
    assert calls == 0


def test_arsenal_small_countered_components_receive_real_interior_fill(arsenal_frontend_default_90_result):
    placed, toolpaths, debug = arsenal_frontend_default_90_result
    target_components = [
        geometry
        for geometry in pipeline_core.normalize_geometry(placed.printable_geometry)
        if len(geometry.interiors) > 0 and 1.0 <= float(geometry.area) <= 8.0
    ]
    assert target_components

    qualifying_components = 0
    hole_driven_components = 0
    for geometry in target_components:
        component_paths = _paths_starting_in_geometry(toolpaths, geometry)
        if not component_paths:
            continue
        qualifying_components += 1
        fill_infill_paths = [path for path in component_paths if path.kind == "fill-infill"]
        drawing_paths = [path for path in component_paths if path.kind in {"fill-infill", "fill-repair", "detail-trace", "outline"}]
        if any(str((path.metadata or {}).get("fill_mode", "")) == "detail_serpentine_fill" for path in fill_infill_paths):
            hole_driven_components += 1
        largest_uncovered_area_mm2 = _largest_non_outline_uncovered_area_mm2(
            drawing_paths,
            geometry,
            pen_radius_mm=0.3,
        )
        hole = Polygon(list(geometry.interiors)[0])
        hole_center = hole.representative_point()
        non_outline_coverage = coverage_planner._paths_footprint_union(
            [path for path in component_paths if path.kind in {"fill-infill", "fill-repair", "detail-trace"}],
            pen_radius_mm=0.3,
        )

        assert drawing_paths
        assert largest_uncovered_area_mm2 <= 3.0
        assert non_outline_coverage is None or not non_outline_coverage.covers(hole_center)

    assert qualifying_components >= 2
    assert hole_driven_components >= 2
    assert any(
        bool(row.get("hole_driven_detail_fill"))
        for row in debug.get("coverage_component_summary", [])
        if row.get("mode") == "detail-wide"
    )


def test_tiny_dot_uses_interior_stroke_not_outline_border_trace():
    tiny_dot = ShapelyPoint(0.0, 0.0).buffer(0.18, resolution=32)

    toolpaths = _generate_fill_toolpaths(
        tiny_dot,
        line_width_mm=0.5,
        infill_spacing_mm=0.5,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=False,
        simplify_tolerance_mm=0.0,
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    assert not any(path.kind == "outline" for path in toolpaths)
    assert all(path.metadata.get("fill_strategy") == "SINGLE_STROKE_DETAIL" for path in infill_paths)
    _assert_infill_segments_stay_inside_region(infill_paths, tiny_dot, epsilon=1e-4)


def test_thin_script_like_stroke_is_not_dropped():
    thin_connector = Polygon([
        (-4.0, -0.35),
        (4.0, -0.35),
        (4.0, 0.35),
        (-4.0, 0.35),
    ])

    toolpaths = _generate_fill_toolpaths(
        thin_connector,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=30.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=False,
        simplify_tolerance_mm=0.0,
    )
    drawing_paths = [path for path in toolpaths if path.kind in {"fill-infill", "detail-trace", "outline", "fill-wall"}]
    assert drawing_paths
    assert any(path.kind in {"fill-infill", "detail-trace", "outline"} for path in drawing_paths)


def test_thin_diagonal_w_suppresses_micro_outline_fragments_and_keeps_centerlines():
    thin_w = LineString([
        (0.0, 4.0),
        (1.0, 0.0),
        (2.0, 4.0),
        (3.0, 0.0),
        (4.0, 4.0),
    ]).buffer(0.22, cap_style=2, join_style=2)
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        thin_w,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        fill_strategy="adaptive_angle",
        outline_after_fill=True,
        thin_detail_mode=True,
        min_fill_area_mm2=0.0,
        thin_detail_min_area_mm2=0.0,
        simplify_tolerance_mm=0.0,
        debug=debug,
    )

    substituted_centerlines = [
        path
        for path in toolpaths
        if path.kind == "detail-trace" and bool((path.metadata or {}).get("thin_diagonal_centerline_substitution", False))
    ]
    fallback_outline_fragments = [
        path
        for path in toolpaths
        if path.kind == "outline" and str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTLINE_FALLBACK"
    ]

    assert debug.get("artifact_validation_ran") is True
    assert int(debug.get("thin_diagonal_outline_fragments_checked", 0)) > 0
    assert int(debug.get("thin_diagonal_features_centerline_substituted", 0)) > 0
    assert substituted_centerlines
    assert not fallback_outline_fragments
    assert not any(path.kind == "outline" and len(path.points) <= 3 for path in toolpaths)


def test_region_narrower_than_two_pen_width_forces_minimum_stroke_fallback():
    printable = Polygon([
        (0.0, 0.0),
        (6.0, 0.0),
        (6.0, 0.7),
        (0.0, 0.7),
    ])
    line_width_mm = 0.6
    debug: dict = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=line_width_mm,
        infill_spacing_mm=line_width_mm,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        min_fill_area_mm2=10.0,
        thin_detail_mode=True,
        thin_detail_min_area_mm2=10.0,
        min_segment_length_mm=0.5,
        debug=debug,
    )
    outline_paths = _actual_outline_paths(toolpaths)
    assert outline_paths
    assert any(
        bool(path.metadata.get("force_minimum_printable_stroke", False))
        or str(path.metadata.get("fill_mode", "")) in {"single_stroke_fallback_region", "single_stroke_cell", "single_stroke_detail"}
        for path in outline_paths
    )
    diagnostics = (debug.get("infill_debug") or {}).get("diagnostics") or {}
    assert diagnostics.get("narrower_than_2x_pen_regions", 0) >= 1
    assert diagnostics.get("narrower_than_2x_pen_with_centerline", 0) >= 1


def test_prepare_toolpaths_for_projection_straightens_micro_jittered_linework():
    toolpath = Toolpath(
        points=[
            Point(0.0, 0.0),
            Point(2.0, 0.015),
            Point(4.0, -0.01),
            Point(6.0, 0.012),
            Point(8.0, 0.0),
        ],
        kind="fill-infill",
        closed=False,
        metadata={"pen_width_mm": 1.0},
    )

    prepared = pipeline_core.prepare_toolpaths_for_projection([toolpath], default_pen_width_mm=1.0)

    assert len(prepared) == 1
    assert prepared[0].points[0] == Point(0.0, 0.0)
    assert prepared[0].points[-1] == Point(8.0, 0.0)
    assert max(abs(point.y) for point in prepared[0].points) <= 0.015


def test_clipped_detail_paths_remain_inside_drawable_region():
    slicer = pipeline_core.SlicerService()
    region = _rect(10.0, 4.0)
    jittered_line = Toolpath(
        points=[
            Point(0.2, 1.0),
            Point(2.5, 1.02),
            Point(5.0, 1.01),
            Point(7.5, 0.99),
            Point(9.8, 1.0),
        ],
        kind="fill-infill",
        closed=False,
    )

    clipped = slicer._clip_toolpaths_to_region([jittered_line], region=region, tolerance_mm=0.0, kind="fill-infill")

    assert clipped
    assert all(region.buffer(-0.001).covers(LineString([(start.x, start.y), (end.x, end.y)])) for path in clipped for start, end in zip(path.points, path.points[1:]))


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
    assert any(kind in {"outline", "fill-infill"} for kind in preview_kinds)


def test_raster_area_fill_preserves_detail_segments_for_thin_detail_recovery():
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

    assert any(path.kind == "detail-trace" for path in toolpaths)


def test_detail_segments_are_clipped_to_pen_center_safe_region():
    printable = _rect(8.0, 4.0)
    bundle = GeometryBundle(
        printable_geometry=printable,
        detail_segments=[
            Segment(points=[Point(-2.0, 2.0), Point(10.0, 2.0)], closed=False),
        ],
    )
    line_width_mm = 0.6

    toolpaths = ToolpathService().generate_from_regions(
        bundle,
        pen_width_mm=line_width_mm,
        wall_count=1,
        infill_pattern="zigzag",
        infill_spacing_mm=line_width_mm,
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

    detail_paths = [path for path in toolpaths if path.kind == "detail-trace"]
    assert detail_paths
    safe_region = _infill_region(printable, line_width_mm=line_width_mm)
    _assert_infill_segments_stay_inside_region(
        [Toolpath(points=path.points, kind="fill-infill", closed=path.closed) for path in detail_paths],
        safe_region,
        epsilon=1e-4,
    )


def test_carolin_script_long_connectors_are_travel_and_not_pen_down():
    image_path = Path("tests/fixtures/images/Carolin Line.png")
    image_bytes = image_path.read_bytes()
    raster = RasterAnalysisService(CONFIG, MachineState(default_pen_up_s=575))
    analysis = raster.analyze_image(image_bytes, max_colors=32)
    selected = next((entry.id for entry in analysis.colors if entry.hex == "#000000"), analysis.colors[0].id)
    mask = raster.build_mask(
        image_bytes,
        [selected],
        tolerance=24,
        min_component_area_px=0,
        open_radius_px=0,
        close_radius_px=1,
    )
    regions = raster.extract_regions(mask, min_region_area_px=1, simplify_tolerance_px=0.5)
    geometry = GeometryService()
    mapped = geometry.map_bundle_to_surface_mm(regions.bundle, regions.bounds, "contain", True, 4.0)
    placed = geometry.apply_origin_anchor_placement(
        geometry.apply_surface_placement_transform(
            geometry.apply_surface_artwork_scale(mapped, 100.0),
            100.0,
            0.0,
        ),
        origin_anchor="center",
        origin_offset_x_mm=0.0,
        origin_offset_y_mm=0.0,
    )
    debug: dict = {}
    toolpaths = ToolpathService().generate_from_regions(
        placed,
        pen_width_mm=0.6,
        wall_count=1,
        infill_pattern="hatch",
        infill_spacing_mm=0.6,
        infill_density=100.0,
        infill_angle_deg=45.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=False,
        min_region_area=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.05,
        thin_detail_mode=True,
        thin_detail_min_area_mm2=0.01,
        thin_detail_simplify_mm=0.05,
        thin_detail_overlap=True,
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=True,
        infill_path_mode="serpentine_optimized",
        debug=debug,
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.6)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)
    gcode_debug: dict = {}
    gcode, preview = GcodeService().generate_from_toolpaths(
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
        debug=gcode_debug,
    )
    pen_lifts = sum(1 for line in gcode if line.strip().startswith("M3 S575"))
    assert pen_lifts >= 1
    connector_previews = [
        entry for entry in preview
        if (entry.get("source_path_kind") in {"fill-infill-travel", "coverage_connector"})
        or entry.get("kind") in {"fill-infill-travel", "coverage_connector"}
    ]
    assert all(bool(entry.get("pen_down", False)) is False for entry in connector_previews)
    summary = (gcode_debug.get("pen_state_summary") or {})
    assert int(summary.get("connector_paths_pen_down", 0)) == 0


def test_detail_trace_is_suppressed_when_outline_and_infill_already_cover_region():
    printable = _rect(width_mm=12.0, height_mm=6.0)
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.5,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=True,
        simplify_tolerance_mm=0.02,
    )
    detail_paths = [path for path in toolpaths if path.kind == "detail-trace"]
    assert len(detail_paths) == 0


def test_detail_filter_drops_redundant_overlap_path():
    target = _rect(width_mm=12.0, height_mm=6.0)
    candidate = Toolpath(
        points=[Point(1.0, 3.0), Point(11.0, 3.0)],
        kind="detail-trace",
        closed=False,
        source="detail_trace",
    )
    existing = _paths_footprint_geometry(
        [Toolpath(points=[Point(1.0, 3.0), Point(11.0, 3.0)], kind="fill-infill", closed=False)],
        pen_width_mm=0.6,
    )

    result = pipeline_core._filter_detail_trace_candidates_for_export(
        [candidate],
        target_geometry=target,
        existing_painted_area=existing,
        line_width_mm=0.6,
        allow_detail_overlap_outline=True,
        validate_detail_with_pen_footprint=True,
        max_detail_overspill_mm=0.05,
        max_detail_overspill_area_ratio=0.03,
        min_detail_new_coverage_mm2=0.02,
        max_already_covered_ratio=0.90,
        candidate_component_index_fn=lambda _path: None,
        candidate_centeredness_fn=lambda _path, _idx: 0.0,
    )

    assert result["detail_paths_kept"] == 0
    assert result["detail_paths_dropped_as_redundant_overlap"] > 0


def test_detail_filter_rejects_travel_source_as_detail():
    target = _rect(width_mm=12.0, height_mm=6.0)
    candidate = Toolpath(
        points=[Point(1.0, 3.0), Point(11.0, 3.0)],
        kind="detail-trace",
        closed=False,
        source="travel",
    )

    result = pipeline_core._filter_detail_trace_candidates_for_export(
        [candidate],
        target_geometry=target,
        existing_painted_area=Polygon(),
        line_width_mm=0.6,
        allow_detail_overlap_outline=True,
        validate_detail_with_pen_footprint=True,
        max_detail_overspill_mm=0.05,
        max_detail_overspill_area_ratio=0.03,
        min_detail_new_coverage_mm2=0.02,
        max_already_covered_ratio=0.90,
        candidate_component_index_fn=lambda _path: None,
        candidate_centeredness_fn=lambda _path, _idx: 0.0,
    )

    assert result["detail_paths_kept"] == 0
    assert result["travel_geometry_allowed_as_detail"] is False
    assert result["detail_paths_dropped_as_travel_or_debug"] > 0


def test_detail_filter_keeps_useful_uncovered_thin_detail():
    target = Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 1.0), (0.0, 1.0)])
    candidate = Toolpath(
        points=[Point(0.5, 0.5), Point(9.5, 0.5)],
        kind="detail-trace",
        closed=False,
        source="detail_trace",
    )

    result = pipeline_core._filter_detail_trace_candidates_for_export(
        [candidate],
        target_geometry=target,
        existing_painted_area=Polygon(),
        line_width_mm=0.6,
        allow_detail_overlap_outline=True,
        validate_detail_with_pen_footprint=True,
        max_detail_overspill_mm=0.05,
        max_detail_overspill_area_ratio=0.03,
        min_detail_new_coverage_mm2=0.02,
        max_already_covered_ratio=0.90,
        candidate_component_index_fn=lambda _path: None,
        candidate_centeredness_fn=lambda _path, _idx: 0.0,
    )

    assert result["detail_paths_kept"] == 1
    kept = result["accepted_detail_paths"][0]
    assert kept.kind == "detail-trace"
    assert kept.metadata["detail_new_coverage_area_mm2"] > 0.02


def test_detail_filter_keeps_centered_small_detail_despite_heavy_overlap():
    target = Polygon([(0.0, 0.0), (12.0, 0.0), (12.0, 1.0), (0.0, 1.0)])
    candidate = Toolpath(
        points=[Point(0.6, 0.5), Point(11.4, 0.5)],
        kind="detail-trace",
        closed=False,
        source="detail_trace",
    )
    existing = _paths_footprint_geometry(
        [Toolpath(points=[Point(0.6, 0.5), Point(11.4, 0.5)], kind="fill-infill", closed=False)],
        pen_width_mm=0.6,
    )

    result = pipeline_core._filter_detail_trace_candidates_for_export(
        [candidate],
        target_geometry=target,
        existing_painted_area=existing,
        line_width_mm=0.6,
        allow_detail_overlap_outline=True,
        validate_detail_with_pen_footprint=True,
        max_detail_overspill_mm=0.05,
        max_detail_overspill_area_ratio=0.03,
        min_detail_new_coverage_mm2=0.02,
        max_already_covered_ratio=0.90,
        candidate_component_index_fn=lambda _path: 0,
        candidate_centeredness_fn=lambda _path, _idx: 0.05,
        candidate_component_metrics_fn=lambda _path, _idx: {
            "component_id": 1,
            "area_mm2": 1.0,
            "bbox_mm": (12.0, 1.0),
            "estimated_width_mm": 1.0,
        },
    )

    assert result["detail_paths_kept"] == 1
    kept = result["accepted_detail_paths"][0]
    assert kept.metadata["detail_overlap_exception_applied"] is True
    assert kept.metadata["detail_overlap_exception_reason"] == "small_detail_centered_overlap"


def test_detail_filter_rejects_off_center_thin_region_centerline():
    target = Polygon([(0.0, 0.0), (12.0, 0.0), (12.0, 0.8), (0.0, 0.8)])
    candidate = Toolpath(
        points=[Point(0.5, 0.10), Point(11.5, 0.10)],
        kind="detail-trace",
        closed=False,
        source="residual_centerline",
    )

    result = pipeline_core._filter_detail_trace_candidates_for_export(
        [candidate],
        target_geometry=target,
        existing_painted_area=Polygon(),
        line_width_mm=0.6,
        allow_detail_overlap_outline=True,
        validate_detail_with_pen_footprint=True,
        max_detail_overspill_mm=0.05,
        max_detail_overspill_area_ratio=0.03,
        min_detail_new_coverage_mm2=0.02,
        max_already_covered_ratio=0.90,
        candidate_component_index_fn=lambda _path: 0,
        candidate_centeredness_fn=lambda _path, _idx: 0.08,
        candidate_component_metrics_fn=lambda _path, _idx: {
            "component_id": 1,
            "area_mm2": float(target.area),
            "bbox_mm": (12.0, 0.8),
            "estimated_width_mm": 0.8,
        },
    )

    assert result["detail_paths_kept"] == 0
    assert result["detail_paths_dropped"] == 1
    assert result["detail_dropped_path_records"][0]["drop_reason"] in {"off_center_centerline", "overspill"}


def test_shaped_thin_region_centerline_follows_bent_region_shape():
    slicer = pipeline_core.SlicerService()
    center_curve = LineString([
        (0.0, 0.0),
        (3.0, 0.0),
        (5.0, 1.2),
        (7.0, 3.2),
        (7.0, 6.0),
    ])
    printable = center_curve.buffer(0.28, cap_style=1, join_style=1)

    centerlines = slicer._generate_shaped_thin_region_centerline(
        printable,
        angle_deg=0.0,
        line_width_mm=0.6,
        min_segment_length_mm=0.2,
        tolerance_mm=0.02,
        kind="detail-trace",
    )

    assert centerlines
    longest = max(centerlines, key=lambda path: pipeline_core.segment_length(path.points))
    chord = math.hypot(longest.points[-1].x - longest.points[0].x, longest.points[-1].y - longest.points[0].y)
    assert len(longest.points) > 2
    assert pipeline_core.segment_length(longest.points) >= center_curve.length * 0.75
    assert pipeline_core.segment_length(longest.points) > chord * 1.1
    assert printable.buffer(1e-4).covers(_line_for_path(longest))


def test_narrow_c_like_residual_prefers_clean_centerline_detail_trace():
    outer = Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 8.0), (0.0, 8.0)])
    cut = Polygon([(3.0, 1.5), (10.0, 1.5), (10.0, 6.5), (3.0, 6.5)])
    printable = outer.difference(cut)
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.55,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=True,
        simplify_tolerance_mm=0.02,
    )
    detail_paths = [path for path in toolpaths if path.kind == "detail-trace"]
    assert len(detail_paths) <= 2
    for path in detail_paths:
        if len(path.points) < 2:
            continue
        chord = math.hypot(path.points[-1].x - path.points[0].x, path.points[-1].y - path.points[0].y)
        if chord <= 1e-6:
            continue
        assert segment_length(path.points) / chord < 3.0
    if detail_paths:
        assert any((path.metadata or {}).get("path_role") in {"PRINT_DETAIL", "PRINT_DETAIL_EDGE"} for path in detail_paths)


def test_thin_curved_feature_emits_continuous_centerline_fill():
    center_curve = LineString([
        (0.0, 0.0),
        (2.5, 0.0),
        (4.5, 1.0),
        (6.0, 2.5),
        (6.0, 5.5),
    ])
    printable = center_curve.buffer(0.28, cap_style=1, join_style=1)

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=False,
        simplify_tolerance_mm=0.02,
    )
    centerline_paths = [
        path
        for path in toolpaths
        if path.kind in {"fill-infill", "detail-trace", "outline"}
        and (
            bool((path.metadata or {}).get("actual_outline_centerline", False))
            or str((path.metadata or {}).get("small_detail_fill_style", "")) in {"single_stroke_detail", "thin_region_centerline"}
            or str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTLINE_FALLBACK"
        )
    ]

    assert centerline_paths
    longest = max(centerline_paths, key=lambda path: pipeline_core.segment_length(path.points))
    assert pipeline_core.segment_length(longest.points) >= center_curve.length * 0.75
    assert printable.buffer(1e-4).covers(_line_for_path(longest))


def test_source_driven_thin_region_centerline_pass_emits_continuous_path():
    center_curve = LineString([
        (0.0, 0.0),
        (2.0, 0.0),
        (4.0, 0.8),
        (5.5, 2.2),
        (5.5, 5.5),
    ])
    printable = center_curve.buffer(0.26, cap_style=1, join_style=1)
    mask, origin_x, origin_y, px_per_mm = coverage_planner._rasterize_geometry(
        printable,
        resolution_mm=0.03,
        pad_mm=0.6,
    )
    accepted, stats, rows = coverage_planner._source_thin_region_centerline_pass(
        thin_region_infos=[{
            "component_id": 1,
            "geometry": printable,
            "mask": mask,
            "origin_x": float(origin_x),
            "origin_y": float(origin_y),
            "px_per_mm": float(px_per_mm),
            "area_mm2": float(printable.area),
            "max_width_mm": 0.52,
            "median_width_mm": 0.52,
            "equivalent_diameter_mm": coverage_planner._equivalent_diameter_mm(float(printable.area)),
            "angle_deg": 0.0,
            "has_holes": False,
        }],
        current_paths=[],
        line_width_mm=0.6,
        simplify_tolerance_mm=0.02,
    )

    assert stats["thin_source_region_detection_ran"] is True
    assert stats["thin_centerline_accepted_count"] >= 1
    assert accepted
    longest = max(accepted, key=lambda path: pipeline_core.segment_length(path.points))
    assert longest.source == "thin_source_region_centerline"
    assert pipeline_core.segment_length(longest.points) >= center_curve.length * 0.75
    assert printable.buffer(1e-4).covers(_line_for_path(longest))
    assert any(float(row.get("length_coverage_ratio", 0.0)) >= 0.55 for row in rows)


def test_detail_trace_footprint_validation_limits_overspill_and_allows_outline_overlap():
    outer = Polygon([(0.0, 0.0), (14.0, 0.0), (14.0, 8.0), (0.0, 8.0)])
    cut = Polygon([(4.0, 1.0), (14.0, 1.0), (14.0, 7.0), (4.0, 7.0)])
    printable = outer.difference(cut)
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=True,
        simplify_tolerance_mm=0.02,
    )
    detail_paths = [p for p in toolpaths if p.kind == "detail-trace"]
    for path in detail_paths:
        md = path.metadata or {}
        if "detail_overspill_area_ratio" in md:
            assert float(md["detail_overspill_area_ratio"]) <= 0.05
        if "detail_max_protrusion_mm" in md:
            assert float(md["detail_max_protrusion_mm"]) <= 0.08




def test_straight_horizontal_bar_uses_simple_long_strokes_with_few_pen_lifts():
    printable = Polygon([
        (0.0, 0.0),
        (30.0, 0.0),
        (30.0, 1.8),
        (0.0, 1.8),
    ])
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=False,
        simplify_tolerance_mm=0.0,
    )
    prepared = pipeline_core.prepare_toolpaths_for_projection(toolpaths, default_pen_width_mm=0.6)
    projected = pipeline_core.project_toolpaths_to_ball_angles(prepared, center_lon_deg=0.0, center_lat_deg=0.0)
    gcode, preview = GcodeService().generate_from_toolpaths(
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
    pen_lifts = sum(1 for line in gcode if line.strip().startswith("M3 S575"))
    assert pen_lifts <= 5
    draw_paths = [entry for entry in preview if entry.get("kind") in {"fill-infill", "fill-wall", "outline", "detail-trace"}]
    assert draw_paths
    for entry in draw_paths:
        points = entry.get("points") or []
        if len(points) < 2:
            continue
        dy_total = sum(abs(points[index]["y"] - points[index - 1]["y"]) for index in range(1, len(points)))
        dx_total = sum(abs(points[index]["x"] - points[index - 1]["x"]) for index in range(1, len(points)))
        if dx_total > 0.5:
            assert dy_total <= dx_total * 0.35


def test_medium_width_straight_bar_uses_clean_parallel_strokes_without_crisscross():
    printable = Polygon([
        (0.0, 0.0),
        (30.0, 0.0),
        (30.0, 2.8),
        (0.0, 2.8),
    ])
    debug: dict = {}
    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        outline_after_fill=False,
        simplify_tolerance_mm=0.0,
        debug=debug,
    )
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert 2 <= len(infill_paths) <= 6
    for path in infill_paths:
        points = path.points
        assert len(points) >= 2
        dx_total = sum(abs(points[index].x - points[index - 1].x) for index in range(1, len(points)))
        dy_total = sum(abs(points[index].y - points[index - 1].y) for index in range(1, len(points)))
        if dx_total > 0.5:
            assert dy_total <= dx_total * 0.35
    assert int(debug.get("mesh_like_paths_rejected", 0)) >= 0


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


def test_fill_does_not_reserve_outline_space():
    printable = affinity.rotate(_rect(12.0, 6.0), 28.0, origin="centroid")
    debug: dict[str, object] = {}

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=0.6,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=0.48,
        infill_angle_deg=45.0,
        outline_after_fill=True,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    assert any(path.kind == "fill-infill" for path in toolpaths)
    assert debug["fill_uses_outline_clearance"] is False
    assert debug["outline_overlap_allowed"] is True
    assert debug["line_generation_changed"] is False
    assert debug["global_fill_mask_changed"] is False
    assert debug["endpoint_clamp_mode"] == "postprocess_only"
    assert debug["infill_debug"]["endpoint_extension_mm"] == pytest.approx(0.3, abs=1e-6)


def test_scanline_fill_keeps_short_valid_rows_in_tapered_narrow_regions():
    tapered = Polygon([
        (0.0, 0.0),
        (0.7, 0.0),
        (0.7, 4.0),
        (0.06, 4.0),
        (0.0, 2.0),
    ])
    resolution_mm = max(0.03, min(0.06, max(0.03, 0.6 * 0.08)))
    mask, origin_x, origin_y, px_per_mm = coverage_planner._rasterize_geometry(
        tapered,
        resolution_mm=resolution_mm,
        pad_mm=max(0.25, 0.6),
    )

    fill_paths, stats = coverage_planner._scanline_fill_paths(
        tapered,
        angle_deg=0.0,
        spacing_mm=0.48,
        line_width_mm=0.6,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        component_id=1,
        allow_connectors=False,
        max_overflow_mm=0.05,
    )

    row_offsets = [
        float((path.metadata or {}).get("scanline_offset_mm", -1.0))
        for path in fill_paths
        if path.kind == "fill-infill"
    ]
    lengths = [
        pipeline_core.segment_length(path.points)
        for path in fill_paths
        if path.kind == "fill-infill"
    ]

    assert stats["row_count"] >= 8
    assert any(offset == pytest.approx(2.4, abs=1e-6) for offset in row_offsets)
    assert max(b - a for a, b in zip(row_offsets, row_offsets[1:])) <= 0.480001
    assert min(lengths) > 0.6


def test_endpoint_coverage_improves_without_outline_clearance():
    printable = affinity.rotate(_rect(14.0, 5.0), 31.0, origin="centroid")
    debug: dict[str, object] = {}

    toolpaths = generate_toolpaths(
        GeometryBundle(printable_geometry=printable),
        enable_fill=True,
        line_width_mm=0.6,
        wall_count=1,
        infill_density=100.0,
        infill_spacing_mm=0.48,
        infill_angle_deg=45.0,
        outline_after_fill=True,
        min_fill_area_mm2=0.0,
        min_fill_width_mm=0.0,
        simplify_tolerance_mm=0.0,
        remove_duplicate_paths=False,
        small_shape_mode="single-wall",
        min_segment_length_mm=0.0,
        travel_optimization="nearest-neighbor",
        allow_pen_down_infill_connectors=False,
        expensive_coverage_repair=False,
        debug=debug,
    )

    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]
    assert infill_paths
    legacy_fill_geometry = printable.buffer(-(0.6 * 0.5), join_style=1)
    if legacy_fill_geometry.is_empty:
        legacy_fill_geometry = printable
    legacy_paths, _legacy_stats = coverage_planner._scanline_fill_paths(
        legacy_fill_geometry,
        angle_deg=45.0,
        spacing_mm=0.48,
        line_width_mm=0.6,
        origin_x=0.0,
        origin_y=0.0,
        px_per_mm=10.0,
        component_id=1,
        allow_connectors=False,
        max_overflow_mm=0.05,
        fill_mode_label="serpentine",
    )
    current_footprint = _paths_footprint_geometry(infill_paths, 0.6)
    legacy_footprint = _paths_footprint_geometry([path for path in legacy_paths if path.kind == "fill-infill"], 0.6)
    current_missed = printable.area - printable.intersection(current_footprint).area
    legacy_missed = printable.area - printable.intersection(legacy_footprint).area

    assert debug["coverage_before_outline_percent"] > 0.0
    assert debug["missed_area_before_outline_mm2"] <= legacy_missed * 2.0
    assert current_missed <= legacy_missed * 2.0
    assert debug["endpoint_extensions_added"] > 0


def test_outline_overlap_is_allowed_for_infill_endpoints():
    printable = affinity.rotate(_rect(10.0, 4.0), 22.0, origin="centroid")
    debug: dict[str, object] = {}

    toolpaths = _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.48,
        infill_angle_deg=45.0,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    report = debug["coverage_report"]
    assert report["infill_outline_overlap_area_mm2"] > 0.0
    assert debug["outline_overlap_allowed"] is True


def test_endpoint_extensions_do_not_create_visible_outside_overflow():
    printable = affinity.rotate(_rect(11.0, 3.6), 35.0, origin="centroid")
    debug: dict[str, object] = {}

    _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.48,
        infill_angle_deg=45.0,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    report = debug["coverage_report"]
    assert report["line_generation_changed"] is False
    assert report["global_fill_mask_changed"] is False
    assert report["line_width_mm"] == pytest.approx(0.6, abs=1e-9)
    assert report["infill_spacing_mm"] == pytest.approx(0.48, abs=1e-9)
    assert report["outside_overflow_mm2"] <= 1.0
    assert report["infill_beyond_outline_after_mm2"] < report["infill_beyond_outline_before_mm2"]
    assert report["coverage_after_endpoint_clamp_percent"] == pytest.approx(report["coverage_before_endpoint_clamp_percent"], abs=0.25)
    assert debug["endpoint_extensions_clipped"] >= 0


def test_endpoint_extension_keeps_spacing_unchanged():
    printable = affinity.rotate(_rect(12.0, 6.0), 17.0, origin="centroid")
    debug: dict[str, object] = {}

    _generate_fill_toolpaths(
        printable,
        line_width_mm=0.6,
        infill_spacing_mm=0.48,
        infill_angle_deg=45.0,
        outline_after_fill=True,
        allow_pen_down_infill_connectors=False,
        debug=debug,
    )

    assert debug["infill_debug"]["pen_width_mm"] == pytest.approx(0.6, abs=1e-6)
    assert debug["infill_debug"]["spacing_mm"] == pytest.approx(0.48, abs=1e-6)
    assert debug["line_generation_changed"] is False
    assert debug["global_fill_mask_changed"] is False


def test_endpoint_clamp_moves_only_segment_endpoints_parallel_to_row_direction():
    printable = affinity.rotate(_rect(8.0, 2.4), 17.0, origin="centroid")
    fill_paths, _fill_stats = coverage_planner._scanline_fill_paths(
        printable,
        angle_deg=17.0,
        spacing_mm=0.48,
        line_width_mm=0.6,
        origin_x=0.0,
        origin_y=0.0,
        px_per_mm=10.0,
        component_id=1,
        allow_connectors=False,
        max_overflow_mm=0.05,
        fill_mode_label="serpentine",
    )
    outline_paths = coverage_planner._boundary_paths_for_component(
        printable,
        component_id=1,
        simplify_tolerance_mm=0.0,
        line_width_mm=0.6,
    )
    outline_footprint = coverage_planner._paths_footprint_union(outline_paths, pen_radius_mm=0.3)
    outline_limit = printable.union(outline_footprint)

    clamped_paths, clamp_stats = coverage_planner._clamp_infill_endpoints_to_outline_limit(
        fill_paths,
        allowed_geom=outline_limit,
        pen_radius_mm=0.3,
        max_retract_mm=0.3,
        precision_mm=0.02,
    )

    assert clamp_stats["endpoints_checked"] == len(fill_paths) * 2
    assert clamp_stats["endpoints_clamped"] > 0

    for original, clamped in zip(fill_paths, clamped_paths):
        assert len(original.points) == len(clamped.points)
        assert clamped.points[1:-1] == original.points[1:-1]
        if len(original.points) < 2:
            continue

        start_dir = (
            float(original.points[1].x - original.points[0].x),
            float(original.points[1].y - original.points[0].y),
        )
        start_move = (
            float(clamped.points[0].x - original.points[0].x),
            float(clamped.points[0].y - original.points[0].y),
        )
        end_dir = (
            float(original.points[-2].x - original.points[-1].x),
            float(original.points[-2].y - original.points[-1].y),
        )
        end_move = (
            float(clamped.points[-1].x - original.points[-1].x),
            float(clamped.points[-1].y - original.points[-1].y),
        )

        assert abs((start_move[0] * start_dir[1]) - (start_move[1] * start_dir[0])) <= 1e-6
        assert abs((end_move[0] * end_dir[1]) - (end_move[1] * end_dir[0])) <= 1e-6
        assert ((start_move[0] * start_dir[0]) + (start_move[1] * start_dir[1])) >= -1e-9
        assert ((end_move[0] * end_dir[0]) + (end_move[1] * end_dir[1])) >= -1e-9


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

    assert comparison["same_kind_by_path"]["outline_001"] is True
    assert comparison["same_kind_by_path"]["fill-infill_001"] is True
    travel_entries = {
        path_id: same_kind
        for path_id, same_kind in comparison["same_kind_by_path"].items()
        if str(path_id).startswith("travel")
    }
    assert travel_entries
    assert all(value is True for value in travel_entries.values())
    assert all((delta or 0.0) <= 1e-9 for path_id, delta in comparison["max_point_delta_deg_by_path"].items() if path_id != "coverage_centerline_001")


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
    assert "previewAndGcodeShareSameProjectedPaths" in metadata
    assert len(metadata["projectedVsGcodeMismatchSquareIds"]) < len(metadata["squares"])


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
    assert "preview_and_gcode_share_same_projected_paths" in projected_debug
    for square in metadata["squares"]:
        assert square["gcodeBbox"] is not None
        assert square["machineDegreeBbox"] is not None
        assert square["gcodeBbox"]["width"] > 0.0
        assert square["gcodeBbox"]["height"] > 0.0


def test_smaller_mm_infill_spacing_generates_many_more_rows():
    # Keep this as a planner regression, but avoid the pathological 30 mm / 0.15 mm case
    # that turns one assertion into the dominant cost of the whole suite.
    printable = _rect(8.0, 8.0)

    sparse = _generate_fill_toolpaths(printable, line_width_mm=0.6, infill_spacing_mm=0.6)
    dense = _generate_fill_toolpaths(printable, line_width_mm=0.2, infill_spacing_mm=0.2)

    sparse_segments = sum(max(0, len(path.points) - 1) for path in sparse if path.kind == "fill-infill")
    dense_segments = sum(max(0, len(path.points) - 1) for path in dense if path.kind == "fill-infill")

    assert dense_segments > sparse_segments * 2.0


def test_pen_width_drives_dense_infill_spacing_when_spacing_matches_pen_width():
    printable = _rect(10.0, 10.0)

    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.3, infill_spacing_mm=0.3)
    infill_paths = [path for path in toolpaths if path.kind == "fill-infill"]

    row_positions = []
    for infill_path in infill_paths:
        for point in infill_path.points:
            row_positions.append(round(point.y, 6))
    row_positions = sorted(set(row_positions))

    spacings = [row_positions[index] - row_positions[index - 1] for index in range(1, len(row_positions))]

    assert len(row_positions) > 20
    assert min(spacings) == pytest.approx(0.255, abs=0.01)
    assert max(spacings) == pytest.approx(0.255, abs=0.01)


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
    assert projected_debug["preview_path_hash"]
    assert projected_debug["gcode_path_hash"]


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


def _count_pen_lifts_from_gcode(gcode: list[str], pen_up_s: int) -> int:
    needle = f"M3 S{pen_up_s}"
    return sum(1 for line in gcode if line.strip().startswith(needle))


def _is_tiny_x_like(path: Toolpath, line_width_mm: float) -> bool:
    if not (3 <= len(path.points) <= 10):
        return False
    xs = [p.x for p in path.points]
    ys = [p.y for p in path.points]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    if max(width, height) > line_width_mm * 2.2:
        return False
    turns = 0
    angles: list[float] = []
    for start, end in zip(path.points, path.points[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        if abs(dx) <= 1e-9 and abs(dy) <= 1e-9:
            continue
        angles.append(math.degrees(math.atan2(dy, dx)))
    for prev, cur in zip(angles, angles[1:]):
        delta = abs(((cur - prev + 180.0) % 360.0) - 180.0)
        if 35.0 <= delta <= 170.0:
            turns += 1
    return turns >= 2


def test_horizontal_bar_uses_simple_coverage_paths_without_mesh():
    printable = _rect(20.0, 1.2)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.6, infill_spacing_mm=0.6, infill_angle_deg=0.0)
    draw_paths = [p for p in toolpaths if p.kind != "travel"]
    assert draw_paths
    assert all(p.kind in {"coverage_centerline", "coverage_offset_line", "coverage_rectilinear", "coverage_contour", "coverage_connector", "outline_cleanup"} for p in draw_paths)
    assert not any(p.kind == "detail-trace" for p in draw_paths)
    assert not any(_is_tiny_x_like(p, 0.6) for p in draw_paths)


def test_tiny_dot_uses_internal_mark_not_outline_trace():
    printable = _rect(0.25, 0.25)
    toolpaths = _generate_fill_toolpaths(printable, line_width_mm=0.6, infill_spacing_mm=0.6)
    marks = [p for p in toolpaths if p.kind in {"coverage_centerline", "coverage_offset_line", "coverage_rectilinear"}]
    outlines = [p for p in toolpaths if p.kind == "outline_cleanup"]
    detail_fills = [p for p in toolpaths if p.kind != "travel"]
    assert len(marks) <= 1
    assert len(outlines) <= 1
    assert detail_fills or not toolpaths
    assert all(path.metadata.get("small_detail_fill_style") == "single_stroke_detail" for path in detail_fills)
    assert not any(path.kind == "fill-wall" for path in toolpaths)


def test_c_shape_detail_contour_gets_centerline_backstop_when_core_uncovered():
    outer = ShapelyPoint(0.0, 0.0).buffer(5.0, resolution=64)
    inner = ShapelyPoint(0.0, 0.0).buffer(3.4, resolution=64)
    ring = outer.difference(inner)
    cut = Polygon([(0.2, -8.0), (8.0, -8.0), (8.0, 8.0), (0.2, 8.0)])
    c_shape = ring.difference(cut)

    toolpaths = _generate_fill_toolpaths(
        c_shape,
        line_width_mm=0.6,
        infill_spacing_mm=0.6,
        infill_angle_deg=0.0,
        fill_strategy="adaptive_angle",
        simplify_tolerance_mm=0.0,
    )
    infill = [p for p in toolpaths if p.kind in {"coverage_centerline", "coverage_contour", "coverage_offset_line", "coverage_rectilinear"}]
    assert infill
    assert any(bool(p.metadata.get("coverage_backstop", False)) for p in infill)


@pytest.mark.parametrize(
    ("geometry", "angle_deg"),
    [
        (
            Polygon([(-0.75, 0.0), (0.75, 0.0), (0.1, 6.0), (-0.1, 6.0)]),
            0.0,
        ),
        (
            affinity.rotate(Polygon([(-0.75, 0.0), (0.75, 0.0), (0.1, 6.0), (-0.1, 6.0)]), 90.0, origin="centroid"),
            90.0,
        ),
    ],
)
def test_scanline_fill_keeps_short_reversed_rows_after_rotation(geometry: Polygon, angle_deg: float):
    fill_paths, stats = coverage_planner._scanline_fill_paths(
        geometry,
        angle_deg=angle_deg,
        spacing_mm=0.6,
        line_width_mm=0.6,
        origin_x=0.0,
        origin_y=0.0,
        px_per_mm=20.0,
        component_id=1,
        allow_connectors=False,
        max_overflow_mm=0.05,
    )

    offsets = sorted(round(float(path.metadata.get("scanline_offset_mm", 0.0)), 6) for path in fill_paths if path.kind == "fill-infill")
    assert len(offsets) >= 10
    assert offsets == pytest.approx([round(offsets[0] + (index * 0.51), 6) for index in range(len(offsets))], abs=0.02)
    assert int(stats["segment_count"]) == len(offsets)
