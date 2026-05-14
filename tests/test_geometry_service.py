from app.models.machine_state import MachineState
from app.services.geometry_service import GeometryService
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
