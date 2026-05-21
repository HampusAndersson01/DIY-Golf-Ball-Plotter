from __future__ import annotations

from . import pipeline_core


class GeometryService:
    bounds_from_bundle = staticmethod(pipeline_core.bounds_from_bundle)
    map_bundle_to_surface_mm = staticmethod(pipeline_core.map_bundle_to_surface_mm)
    apply_surface_artwork_scale = staticmethod(pipeline_core.apply_surface_artwork_scale)
    apply_surface_placement_transform = staticmethod(pipeline_core.apply_surface_placement_transform)
    mm_to_ball_degrees = staticmethod(pipeline_core.mm_to_ball_degrees)
    debug_append_bundle = staticmethod(pipeline_core.debug_append_bundle)

    @staticmethod
    def map_bundle_to_angles(bundle, bounds, fit_mode, invert_y, margin_percent):
        return pipeline_core.map_bundle_to_surface_mm(bundle, bounds, fit_mode, invert_y, margin_percent)

    @staticmethod
    def apply_placement_transform(bundle, placement_scale, rotation_deg, placement_offset_x, placement_offset_y):
        # Legacy compatibility: the current raster/SVG pipeline applies XY offsets during
        # G-code projection, not during the surface-mm placement step.
        return pipeline_core.apply_surface_placement_transform(bundle, placement_scale, rotation_deg)
