import pytest

from app.models.geometry import Point, Segment
from app.models.machine_state import MachineState
from app.services.geometry_service import GeometryService
from app.services.pipeline_core import GeometryBundle
from app.services.svg_parser import SvgParser

from tests.test_svg_parser import CONFIG


def test_bounds_and_mapping_work_for_simple_svg():
    parser = SvgParser(CONFIG, MachineState(default_pen_up_s=575))
    geometry = GeometryService()
    svg = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 10 20"><rect x="0" y="0" width="10" height="20" fill="#000"/></svg>'
    bundle, bounds, _ = parser.extract_svg_bundle(
        svg,
        parser_mode="visible_geometry",
        color_mapping_mode=False,
        trace_stroke_only_paths=True,
        fill_only_dark_svg_fills=True,
        debug={},
    )
    mapped = geometry.map_bundle_to_angles(bundle, bounds, "contain", True, 4.0)
    assert mapped.fill_shapes
    placed = geometry.apply_placement_transform(mapped, 100.0, 0.0, 0.0, 0.0)
    assert placed.fill_shapes


def test_surface_artwork_scale_halves_bbox_and_preserves_center():
    geometry = GeometryService()
    bundle = GeometryBundle(
        outline_segments=[
            Segment(
                points=[
                    Point(-10.0, -4.0),
                    Point(10.0, -4.0),
                    Point(10.0, 4.0),
                    Point(-10.0, 4.0),
                    Point(-10.0, -4.0),
                ],
                closed=True,
            ),
        ],
    )

    original_bounds = geometry.bounds_from_bundle(bundle)
    scaled = geometry.apply_surface_artwork_scale(bundle, 50.0)
    scaled_bounds = geometry.bounds_from_bundle(scaled)

    assert scaled_bounds.width == pytest.approx(original_bounds.width * 0.5, abs=1e-6)
    assert scaled_bounds.height == pytest.approx(original_bounds.height * 0.5, abs=1e-6)
    assert (scaled_bounds.min_x + scaled_bounds.max_x) / 2.0 == pytest.approx((original_bounds.min_x + original_bounds.max_x) / 2.0, abs=1e-6)
    assert (scaled_bounds.min_y + scaled_bounds.max_y) / 2.0 == pytest.approx((original_bounds.min_y + original_bounds.max_y) / 2.0, abs=1e-6)


def test_surface_artwork_scale_100_preserves_geometry_bounds():
    geometry = GeometryService()
    bundle = GeometryBundle(
        outline_segments=[
            Segment(points=[Point(-3.0, -2.0), Point(5.0, -2.0), Point(5.0, 6.0), Point(-3.0, 6.0), Point(-3.0, -2.0)], closed=True),
        ],
    )

    original_bounds = geometry.bounds_from_bundle(bundle)
    scaled = geometry.apply_surface_artwork_scale(bundle, 100.0)
    scaled_bounds = geometry.bounds_from_bundle(scaled)

    assert scaled_bounds.min_x == pytest.approx(original_bounds.min_x, abs=1e-6)
    assert scaled_bounds.min_y == pytest.approx(original_bounds.min_y, abs=1e-6)
    assert scaled_bounds.max_x == pytest.approx(original_bounds.max_x, abs=1e-6)
    assert scaled_bounds.max_y == pytest.approx(original_bounds.max_y, abs=1e-6)
