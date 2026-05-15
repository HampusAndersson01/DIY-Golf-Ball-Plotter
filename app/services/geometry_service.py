from __future__ import annotations

from . import pipeline_core


class GeometryService:
    bounds_from_bundle = staticmethod(pipeline_core.bounds_from_bundle)
    map_bundle_to_surface_mm = staticmethod(pipeline_core.map_bundle_to_surface_mm)
    apply_surface_placement_transform = staticmethod(pipeline_core.apply_surface_placement_transform)
    mm_to_ball_degrees = staticmethod(pipeline_core.mm_to_ball_degrees)
    debug_append_bundle = staticmethod(pipeline_core.debug_append_bundle)
