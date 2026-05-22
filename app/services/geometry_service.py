from __future__ import annotations

from . import pipeline_core


class GeometryService:
    bounds_from_bundle = staticmethod(pipeline_core.bounds_from_bundle)
    compute_artwork_bbox = staticmethod(pipeline_core.compute_artwork_bbox)
    map_bundle_to_surface_mm = staticmethod(pipeline_core.map_bundle_to_surface_mm)
    apply_surface_artwork_scale = staticmethod(pipeline_core.apply_surface_artwork_scale)
    apply_surface_placement_transform = staticmethod(pipeline_core.apply_surface_placement_transform)
    resolve_origin_anchor_point = staticmethod(pipeline_core.resolve_origin_anchor_point)
    apply_origin_anchor_placement = staticmethod(pipeline_core.apply_origin_anchor_placement)
    apply_surface_mm_translation = staticmethod(pipeline_core.apply_surface_mm_translation)
    mm_to_ball_degrees = staticmethod(pipeline_core.mm_to_ball_degrees)
    debug_append_bundle = staticmethod(pipeline_core.debug_append_bundle)

    @staticmethod
    def map_bundle_to_angles(bundle, bounds, fit_mode, invert_y, margin_percent):
        return pipeline_core.map_bundle_to_surface_mm(bundle, bounds, fit_mode, invert_y, margin_percent)

    @staticmethod
    def apply_placement_transform(bundle, placement_scale, rotation_deg, placement_offset_x, placement_offset_y):
        # Legacy compatibility: this helper keeps the older call signature used by tests and
        # callers that pass placement offsets, but the canonical placement pipeline now resolves
        # origin-anchor translation separately in surface-mm space before projection.
        return pipeline_core.apply_surface_placement_transform(bundle, placement_scale, rotation_deg)
