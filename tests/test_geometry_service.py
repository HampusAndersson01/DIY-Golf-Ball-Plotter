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


def _make_rect_bundle() -> GeometryBundle:
    return GeometryBundle(
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


@pytest.mark.parametrize(
    ("anchor", "expected_bounds"),
    [
        ("center", (-10.0, -4.0, 10.0, 4.0)),
        ("min-x", (0.0, -4.0, 20.0, 4.0)),
        ("max-x", (-20.0, -4.0, 0.0, 4.0)),
        ("min-y", (-10.0, 0.0, 10.0, 8.0)),
        ("max-y", (-10.0, -8.0, 10.0, 0.0)),
        ("bottom-left", (0.0, 0.0, 20.0, 8.0)),
        ("top-right", (-20.0, -8.0, 0.0, 0.0)),
    ],
)
def test_origin_anchor_placement_moves_expected_bbox_point_to_surface_origin(anchor: str, expected_bounds: tuple[float, float, float, float]):
    geometry = GeometryService()
    placed = geometry.apply_origin_anchor_placement(
        _make_rect_bundle(),
        origin_anchor=anchor,
        origin_offset_x_mm=0.0,
        origin_offset_y_mm=0.0,
    )

    bounds = geometry.bounds_from_bundle(placed)
    assert (bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y) == pytest.approx(expected_bounds, abs=1e-6)


def test_origin_anchor_custom_uses_center_until_custom_point_ui_exists():
    geometry = GeometryService()
    center_placed = geometry.apply_origin_anchor_placement(
        _make_rect_bundle(),
        origin_anchor="center",
        origin_offset_x_mm=3.0,
        origin_offset_y_mm=-2.0,
    )
    custom_placed = geometry.apply_origin_anchor_placement(
        _make_rect_bundle(),
        origin_anchor="custom",
        origin_offset_x_mm=3.0,
        origin_offset_y_mm=-2.0,
    )

    center_bounds = geometry.bounds_from_bundle(center_placed)
    custom_bounds = geometry.bounds_from_bundle(custom_placed)
    assert (custom_bounds.min_x, custom_bounds.min_y, custom_bounds.max_x, custom_bounds.max_y) == pytest.approx(
        (center_bounds.min_x, center_bounds.min_y, center_bounds.max_x, center_bounds.max_y),
        abs=1e-6,
    )


def test_origin_anchor_manual_offset_translates_geometry_after_anchor_resolution():
    geometry = GeometryService()
    placed = geometry.apply_origin_anchor_placement(
        _make_rect_bundle(),
        origin_anchor="bottom-left",
        origin_offset_x_mm=5.0,
        origin_offset_y_mm=2.0,
    )

    bounds = geometry.bounds_from_bundle(placed)
    assert bounds.min_x == pytest.approx(5.0, abs=1e-6)
    assert bounds.min_y == pytest.approx(2.0, abs=1e-6)
    assert bounds.max_x == pytest.approx(25.0, abs=1e-6)
    assert bounds.max_y == pytest.approx(10.0, abs=1e-6)


def test_origin_anchor_is_applied_after_artwork_scale():
    geometry = GeometryService()
    scaled = geometry.apply_surface_artwork_scale(_make_rect_bundle(), 50.0)
    placed = geometry.apply_origin_anchor_placement(
        scaled,
        origin_anchor="bottom-left",
        origin_offset_x_mm=0.0,
        origin_offset_y_mm=0.0,
    )

    bounds = geometry.bounds_from_bundle(placed)
    assert bounds.min_x == pytest.approx(0.0, abs=1e-6)
    assert bounds.min_y == pytest.approx(0.0, abs=1e-6)
    assert bounds.max_x == pytest.approx(10.0, abs=1e-6)
    assert bounds.max_y == pytest.approx(4.0, abs=1e-6)
