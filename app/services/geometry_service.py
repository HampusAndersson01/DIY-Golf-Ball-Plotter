from __future__ import annotations

from ._legacy import legacy


class GeometryService:
    bounds_from_bundle = staticmethod(legacy.bounds_from_bundle)
    map_bundle_to_angles = staticmethod(legacy.map_bundle_to_angles)
    apply_placement_transform = staticmethod(legacy.apply_placement_transform)
    mm_to_ball_degrees = staticmethod(legacy.mm_to_ball_degrees)
    debug_append_bundle = staticmethod(legacy.debug_append_bundle)
