from __future__ import annotations

from dataclasses import replace

from . import pipeline_core


class ToolpathService:
    generate_toolpaths = staticmethod(pipeline_core.generate_toolpaths)
    summarize_toolpaths = staticmethod(pipeline_core.summarize_toolpaths)

    def generate_from_regions(
        self,
        regions,
        *,
        pen_width_mm: float,
        wall_count: int,
        infill_pattern: str = "hatch",
        infill_spacing_mm: float | None = None,
        infill_density: float = 100.0,
        infill_angle_deg: float = 0.0,
        fill_strategy: str = "horizontal_scanline",
        alternate_fill_angle_deg: float = -45.0,
        outline_after_fill: bool = False,
        min_region_area: float = 0.0,
        min_fill_width_mm: float = 0.0,
        simplify_tolerance_mm: float = 0.0,
        remove_duplicate_paths: bool = True,
        small_shape_mode: str = "single-wall",
        thin_detail_mode: bool = True,
        thin_detail_min_area_mm2: float = 0.05,
        thin_detail_simplify_mm: float = 0.1,
        thin_detail_overlap: bool = True,
        min_segment_length_mm: float = 0.0,
        travel_optimization: str = "nearest-neighbor",
        allow_pen_down_infill_connectors: bool = False,
        infill_path_mode: str = "rectilinear",
        debug=None,
    ):
        if infill_pattern not in {"zigzag", "hatch"}:
            raise ValueError("Invalid raster infill pattern")
        effective_regions = regions
        if (
            regions.printable_geometry is not None
            and not regions.printable_geometry.is_empty
            and regions.detail_segments
        ):
            # Raster region extraction includes skeleton-derived detail traces for all components.
            # When area fill is available, the slicer should decide where thin-detail fallback is needed.
            effective_regions = replace(regions, detail_segments=[])
        return pipeline_core.generate_toolpaths(
            effective_regions,
            enable_fill=True,
            line_width_mm=pen_width_mm,
            wall_count=wall_count,
            infill_density=infill_density,
            infill_spacing_mm=infill_spacing_mm if infill_spacing_mm is not None else pen_width_mm,
            infill_angle_deg=infill_angle_deg,
            outline_after_fill=outline_after_fill,
            min_fill_area_mm2=min_region_area,
            min_fill_width_mm=min_fill_width_mm,
            simplify_tolerance_mm=simplify_tolerance_mm,
            remove_duplicate_paths=remove_duplicate_paths,
            small_shape_mode=small_shape_mode,
            fill_strategy=fill_strategy,
            alternate_fill_angle_deg=alternate_fill_angle_deg,
            thin_detail_mode=thin_detail_mode,
            thin_detail_min_area_mm2=thin_detail_min_area_mm2,
            thin_detail_simplify_mm=thin_detail_simplify_mm,
            thin_detail_overlap=thin_detail_overlap,
            min_segment_length_mm=min_segment_length_mm,
            travel_optimization=travel_optimization,
            allow_pen_down_infill_connectors=allow_pen_down_infill_connectors,
            infill_path_mode=infill_path_mode,
            debug=debug,
        )
