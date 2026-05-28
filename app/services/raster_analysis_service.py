from __future__ import annotations

import base64
import io
import json
from dataclasses import asdict, dataclass
from typing import Any

import cv2
import numpy as np
from PIL import Image, ImageOps
from shapely.geometry import Polygon
from shapely.ops import unary_union

from . import pipeline_core


@dataclass
class RasterColorSwatch:
    id: str
    hex: str
    rgb: list[int]
    pixel_count: int
    coverage: float
    coverage_percent: float
    luminance: float
    is_transparent: bool = False


@dataclass
class RasterAnalysisResult:
    width: int
    height: int
    colors: list[RasterColorSwatch]
    original_preview_url: str
    quantized_preview_url: str
    ignored_transparent_pixels: int
    total_opaque_pixels: int
    color_count: int


@dataclass
class MaskResult:
    width: int
    height: int
    selected_colors: list[str]
    tolerance: int
    printable_pixel_count: int
    connected_component_count: int
    mask: np.ndarray
    mask_preview_url: str


@dataclass
class RegionGeometryResult:
    bundle: pipeline_core.GeometryBundle
    bounds: pipeline_core.SvgBounds
    region_count: int
    hole_count: int
    selected_component_count: int
    detail_trace_component_count: int
    detail_trace_path_count: int
    skeleton_pixel_count: int
    contour_count: int
    polygon_count: int
    printable_area_px: float
    boundary_preview_url: str


@dataclass
class _LoadedImage:
    rgba: np.ndarray
    rgb: np.ndarray
    opaque_mask: np.ndarray
    width: int
    height: int


@dataclass
class _GroupedColorAnalysis:
    colors: list[RasterColorSwatch]
    labels: np.ndarray
    quantized_rgb: np.ndarray
    ignored_transparent_pixels: int
    total_opaque_pixels: int


@dataclass
class _WorkingColorGroup:
    sum_rgb: np.ndarray
    pixel_count: int
    centroid: np.ndarray
    representative_rgb: np.ndarray
    representative_count: int

    def absorb(self, color: np.ndarray, count: int) -> None:
        self.sum_rgb += color.astype(np.int64) * count
        self.pixel_count += int(count)
        self.centroid = self.sum_rgb.astype(np.float64) / float(self.pixel_count)
        if int(count) > self.representative_count:
            self.representative_rgb = color.astype(np.uint8)
            self.representative_count = int(count)


def _rgb_to_hex(rgb: np.ndarray | list[int] | tuple[int, int, int]) -> str:
    r, g, b = [int(max(0, min(255, value))) for value in rgb]
    return f"#{r:02X}{g:02X}{b:02X}"


def _hex_to_rgb(value: str) -> np.ndarray:
    text = value.strip().lstrip("#")
    if len(text) != 6:
        raise ValueError(f"Invalid hex color: {value}")
    return np.array([int(text[0:2], 16), int(text[2:4], 16), int(text[4:6], 16)], dtype=np.int16)


def _luminance(rgb: np.ndarray | list[int]) -> float:
    r, g, b = [int(value) for value in rgb]
    return (0.2126 * r) + (0.7152 * g) + (0.0722 * b)


def _image_to_data_url(image: np.ndarray) -> str:
    with io.BytesIO() as buffer:
        Image.fromarray(image.astype(np.uint8), mode="RGB").save(buffer, format="PNG")
        encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
    return f"data:image/png;base64,{encoded}"


class RasterAnalysisService:
    ALPHA_THRESHOLD = 16
    COLOR_MERGE_THRESHOLD = 36.0

    def __init__(self, config, state) -> None:
        self._config = config
        self._state = state
        pipeline_core.configure_runtime(config, state.raw, pipeline_core.serial_lock)

    def analyze_image(
        self,
        image_bytes: bytes,
        *,
        simplify_colors: bool = True,
        max_colors: int = 32,
    ) -> RasterAnalysisResult:
        image = self._load_image(image_bytes)
        grouped = self._analyze_grouped_colors(
            image,
            simplify_colors=simplify_colors,
            max_colors=max(1, max_colors),
        )
        return RasterAnalysisResult(
            width=image.width,
            height=image.height,
            colors=grouped.colors,
            original_preview_url=_image_to_data_url(image.rgb),
            quantized_preview_url=_image_to_data_url(grouped.quantized_rgb),
            ignored_transparent_pixels=grouped.ignored_transparent_pixels,
            total_opaque_pixels=grouped.total_opaque_pixels,
            color_count=len(grouped.colors),
        )

    def build_mask(
        self,
        image_bytes: bytes,
        selected_colors: list[str],
        *,
        simplify_colors: bool = True,
        max_colors: int | None = None,
        tolerance: int = 24,
        min_component_area_px: int = 0,
        open_radius_px: int = 0,
        close_radius_px: int = 1,
    ) -> MaskResult:
        if not selected_colors:
            raise ValueError("Select at least one color to print")

        image = self._load_image(image_bytes)
        grouped = self._analyze_grouped_colors(
            image,
            simplify_colors=simplify_colors,
            max_colors=max_colors,
        )
        labels = grouped.labels
        swatches_by_id = {color.id: index for index, color in enumerate(grouped.colors)}
        swatches_by_hex = {color.hex.upper(): index for index, color in enumerate(grouped.colors)}

        mask = np.zeros((image.height, image.width), dtype=bool)
        unresolved: list[str] = []
        for value in selected_colors:
            normalized = value.strip()
            label_index = swatches_by_id.get(normalized)
            if label_index is None:
                label_index = swatches_by_hex.get(normalized.upper())
            if label_index is None:
                unresolved.append(normalized)
                continue
            mask |= labels == label_index

        if unresolved:
            rgb = image.rgba[:, :, :3].astype(np.int16)
            legacy_mask = np.zeros((image.height, image.width), dtype=bool)
            for color in unresolved:
                target = _hex_to_rgb(color)
                distance = np.max(np.abs(rgb - target.reshape(1, 1, 3)), axis=2)
                legacy_mask |= distance <= max(0, tolerance)
            mask |= legacy_mask & image.opaque_mask

        mask_uint8 = self._clean_mask(
            mask.astype(np.uint8) * 255,
            min_component_area_px=max(0, min_component_area_px),
            open_radius_px=max(0, open_radius_px),
            close_radius_px=max(0, close_radius_px),
        )
        printable_pixels = int(np.count_nonzero(mask_uint8))
        connected_component_count = 0
        if printable_pixels > 0:
            component_count, _, _, _ = cv2.connectedComponentsWithStats(mask_uint8, connectivity=8)
            connected_component_count = max(0, int(component_count) - 1)
        preview_rgb = np.full((image.height, image.width, 3), 255, dtype=np.uint8)
        preview_rgb[mask_uint8 > 0] = np.array([17, 24, 39], dtype=np.uint8)

        return MaskResult(
            width=image.width,
            height=image.height,
            selected_colors=[color.strip() for color in selected_colors if color.strip()],
            tolerance=int(tolerance),
            printable_pixel_count=printable_pixels,
            connected_component_count=connected_component_count,
            mask=mask_uint8,
            mask_preview_url=_image_to_data_url(preview_rgb),
        )

    def extract_regions(
        self,
        mask_result: MaskResult | np.ndarray,
        *,
        min_region_area_px: float = 16.0,
        simplify_tolerance_px: float = 1.0,
    ) -> RegionGeometryResult:
        if isinstance(mask_result, MaskResult):
            mask = mask_result.mask
            width = mask_result.width
            height = mask_result.height
        else:
            mask = mask_result
            height, width = mask.shape[:2]

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None or not contours:
            empty_bundle = pipeline_core.GeometryBundle()
            return RegionGeometryResult(
                bundle=empty_bundle,
                bounds=pipeline_core.SvgBounds(0.0, 0.0, float(width), float(height)),
                region_count=0,
                hole_count=0,
                selected_component_count=0,
                detail_trace_component_count=0,
                detail_trace_path_count=0,
                skeleton_pixel_count=0,
                contour_count=0,
                polygon_count=0,
                printable_area_px=0.0,
                boundary_preview_url=_image_to_data_url(np.full((height, width, 3), 255, dtype=np.uint8)),
            )

        hierarchy = hierarchy[0]
        polygons: list[Polygon] = []
        hole_count = 0
        for index, entry in enumerate(hierarchy):
            parent_index = int(entry[3])
            if parent_index != -1:
                continue
            shell = self._contour_ring(contours[index])
            if len(shell) < 3:
                continue

            holes: list[list[tuple[float, float]]] = []
            child_index = int(entry[2])
            while child_index != -1:
                hole_ring = self._contour_ring(contours[child_index])
                if len(hole_ring) >= 3:
                    holes.append(hole_ring)
                    hole_count += 1
                child_index = int(hierarchy[child_index][0])

            polygon = Polygon(shell, holes)
            if simplify_tolerance_px > 0:
                polygon = polygon.simplify(simplify_tolerance_px, preserve_topology=True)
            if not polygon.is_valid:
                polygon = polygon.buffer(0)
            if polygon.is_empty:
                continue
            polygons.extend(pipeline_core.normalize_geometry(polygon))

        filtered = [poly for poly in polygons if poly.area >= float(min_region_area_px)]
        printable_geometry = unary_union(filtered) if filtered else None
        normalized = pipeline_core.normalize_geometry(printable_geometry)
        fill_boundary_segments: list[pipeline_core.Segment] = []
        fill_shapes: list[pipeline_core.SvgFillShape] = []
        for polygon in normalized:
            fill_shapes.append(
                pipeline_core.SvgFillShape(
                    geometry=polygon,
                    fill_rule="evenodd",
                    source_tag="raster-mask",
                )
            )
            fill_boundary_segments.append(
                pipeline_core.Segment(
                    points=[pipeline_core.Point(float(x), float(y)) for x, y in polygon.exterior.coords],
                    closed=True,
                )
            )
            for ring in polygon.interiors:
                fill_boundary_segments.append(
                    pipeline_core.Segment(
                        points=[pipeline_core.Point(float(x), float(y)) for x, y in ring.coords],
                        closed=True,
                    )
                )

        detail_segments, detail_trace_component_count, skeleton_pixel_count = self._extract_detail_trace_segments(
            mask,
            min_component_area_px=float(min_region_area_px),
        )

        preview_rgb = np.full((height, width, 3), 255, dtype=np.uint8)
        if mask.any():
            preview_rgb[mask > 0] = np.array([226, 232, 240], dtype=np.uint8)
        cv2.drawContours(preview_rgb, contours, -1, (37, 99, 235), 1)

        bundle = pipeline_core.GeometryBundle(
            outline_segments=[],
            fill_boundary_segments=fill_boundary_segments,
            detail_segments=detail_segments,
            fill_shapes=fill_shapes,
            printable_geometry=printable_geometry,
            cutout_geometry=None,
        )
        bundle.metadata["connector_validation"] = {
            "source_space": "raster_mask_px",
            "mask": mask.copy(),
            "mask_width": int(width),
            "mask_height": int(height),
            "selected_colors": list(mask_result.selected_colors) if isinstance(mask_result, MaskResult) else [],
            "source_to_current_matrix": (1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            "current_to_source_matrix": (1.0, 0.0, 0.0, 1.0, 0.0, 0.0),
            "sample_step_mm_default": 0.05,
        }
        selected_component_count = 0
        if mask.any():
            component_count, _, _, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            selected_component_count = max(0, int(component_count) - 1)
        return RegionGeometryResult(
            bundle=bundle,
            bounds=pipeline_core.SvgBounds(0.0, 0.0, float(width), float(height)),
            region_count=len(normalized),
            hole_count=hole_count,
            selected_component_count=selected_component_count,
            detail_trace_component_count=detail_trace_component_count,
            detail_trace_path_count=len(detail_segments),
            skeleton_pixel_count=skeleton_pixel_count,
            contour_count=len(contours),
            polygon_count=len(filtered),
            printable_area_px=float(printable_geometry.area) if printable_geometry is not None and not printable_geometry.is_empty else 0.0,
            boundary_preview_url=_image_to_data_url(preview_rgb),
        )

    @staticmethod
    def serialize_analysis(result: RasterAnalysisResult) -> dict[str, Any]:
        return {
            "width": result.width,
            "height": result.height,
            "colors": [asdict(color) for color in result.colors],
            "original_preview_url": result.original_preview_url,
            "quantized_preview_url": result.quantized_preview_url,
            "ignored_transparent_pixels": result.ignored_transparent_pixels,
            "total_opaque_pixels": result.total_opaque_pixels,
            "color_count": result.color_count,
        }

    @staticmethod
    def serialize_mask(result: MaskResult) -> dict[str, Any]:
        return {
            "width": result.width,
            "height": result.height,
            "selected_colors": result.selected_colors,
            "tolerance": result.tolerance,
            "printable_pixel_count": result.printable_pixel_count,
            "connected_component_count": result.connected_component_count,
            "mask_preview_url": result.mask_preview_url,
        }

    @staticmethod
    def serialize_regions(result: RegionGeometryResult) -> dict[str, Any]:
        return {
            "bounds": asdict(result.bounds),
            "region_count": result.region_count,
            "hole_count": result.hole_count,
            "selected_component_count": result.selected_component_count,
            "detail_trace_component_count": result.detail_trace_component_count,
            "detail_trace_path_count": result.detail_trace_path_count,
            "skeleton_pixel_count": result.skeleton_pixel_count,
            "contour_count": result.contour_count,
            "polygon_count": result.polygon_count,
            "printable_area_px": result.printable_area_px,
            "boundary_preview_url": result.boundary_preview_url,
        }

    @staticmethod
    def parse_selected_colors(raw_value: str | None) -> list[str]:
        if not raw_value:
            return []
        try:
            parsed = json.loads(raw_value)
        except json.JSONDecodeError as exc:
            raise ValueError("Selected colors must be valid JSON") from exc
        if not isinstance(parsed, list):
            raise ValueError("Selected colors must be a JSON array")
        return [str(value).strip() for value in parsed if str(value).strip()]

    def _load_image(self, image_bytes: bytes) -> _LoadedImage:
        with Image.open(io.BytesIO(image_bytes)) as image:
            image = ImageOps.exif_transpose(image)
            rgba = image.convert("RGBA")
            background = Image.new("RGBA", rgba.size, (255, 255, 255, 255))
            composited = Image.alpha_composite(background, rgba).convert("RGB")
            rgba_array = np.array(rgba, dtype=np.uint8)
            rgb = np.array(composited, dtype=np.uint8)
            opaque_mask = rgba_array[:, :, 3] > self.ALPHA_THRESHOLD
        return _LoadedImage(
            rgba=rgba_array,
            rgb=rgb,
            opaque_mask=opaque_mask,
            width=int(rgb.shape[1]),
            height=int(rgb.shape[0]),
        )

    def _analyze_grouped_colors(
        self,
        image: _LoadedImage,
        *,
        simplify_colors: bool,
        max_colors: int | None,
    ) -> _GroupedColorAnalysis:
        height, width = image.height, image.width
        labels = np.full((height, width), -1, dtype=np.int32)
        total_pixels = height * width
        opaque_pixels = image.rgba[:, :, :3][image.opaque_mask]
        total_opaque_pixels = int(opaque_pixels.shape[0])
        ignored_transparent_pixels = int(total_pixels - total_opaque_pixels)
        quantized_rgb = image.rgb.copy()

        if total_opaque_pixels == 0:
            return _GroupedColorAnalysis(
                colors=[],
                labels=labels,
                quantized_rgb=quantized_rgb,
                ignored_transparent_pixels=ignored_transparent_pixels,
                total_opaque_pixels=0,
            )

        group_assignments, groups = self._group_opaque_pixels(opaque_pixels, simplify_colors=simplify_colors)
        ranking = sorted(
            range(len(groups)),
            key=lambda index: (
                -groups[index].pixel_count,
                _luminance(groups[index].representative_rgb),
                _rgb_to_hex(groups[index].representative_rgb),
            ),
        )
        if max_colors is not None and max_colors > 0:
            ranking = ranking[:max_colors]

        order_lookup = {group_index: order for order, group_index in enumerate(ranking)}
        visible_mask = np.array([group_index in order_lookup for group_index in group_assignments], dtype=bool)
        opaque_positions = np.argwhere(image.opaque_mask)
        if np.any(visible_mask):
            visible_positions = opaque_positions[visible_mask]
            visible_assignments = np.array([order_lookup[group_index] for group_index in group_assignments[visible_mask]], dtype=np.int32)
            labels[visible_positions[:, 0], visible_positions[:, 1]] = visible_assignments

        colors: list[RasterColorSwatch] = []
        representative_colors: dict[int, np.ndarray] = {}
        for order, group_index in enumerate(ranking):
            group = groups[group_index]
            representative_rgb = group.representative_rgb
            representative_colors[order] = representative_rgb
            coverage = float(group.pixel_count) / float(max(1, total_opaque_pixels))
            colors.append(
                RasterColorSwatch(
                    id=f"color-group-{order}",
                    hex=_rgb_to_hex(representative_rgb),
                    rgb=[int(value) for value in representative_rgb.tolist()],
                    pixel_count=int(group.pixel_count),
                    coverage=coverage,
                    coverage_percent=coverage * 100.0,
                    luminance=float(_luminance(representative_rgb)),
                    is_transparent=False,
                )
            )

        for order, mean_rgb in representative_colors.items():
            quantized_rgb[labels == order] = mean_rgb

        return _GroupedColorAnalysis(
            colors=colors,
            labels=labels,
            quantized_rgb=quantized_rgb,
            ignored_transparent_pixels=ignored_transparent_pixels,
            total_opaque_pixels=total_opaque_pixels,
        )

    def _group_opaque_pixels(self, opaque_pixels: np.ndarray, *, simplify_colors: bool) -> tuple[np.ndarray, list[_WorkingColorGroup]]:
        unique_colors, inverse, counts = np.unique(opaque_pixels, axis=0, return_inverse=True, return_counts=True)
        ordering = sorted(
            range(len(unique_colors)),
            key=lambda index: (
                -int(counts[index]),
                float(_luminance(unique_colors[index])),
                _rgb_to_hex(unique_colors[index]),
            ),
        )

        unique_to_group = np.full(len(unique_colors), -1, dtype=np.int32)
        groups: list[_WorkingColorGroup] = []
        merge_threshold = self.COLOR_MERGE_THRESHOLD if simplify_colors else 0.0

        for color_index in ordering:
            color = unique_colors[color_index].astype(np.float64)
            count = int(counts[color_index])
            best_group_index: int | None = None
            best_distance = float("inf")

            for group_index, group in enumerate(groups):
                distance = float(np.max(np.abs(group.centroid - color)))
                if distance > merge_threshold or distance >= best_distance:
                    continue
                best_group_index = group_index
                best_distance = distance

            if best_group_index is None:
                groups.append(
                    _WorkingColorGroup(
                        sum_rgb=unique_colors[color_index].astype(np.int64) * count,
                        pixel_count=count,
                        centroid=color.copy(),
                        representative_rgb=unique_colors[color_index].astype(np.uint8),
                        representative_count=count,
                    )
                )
                unique_to_group[color_index] = len(groups) - 1
                continue

            groups[best_group_index].absorb(unique_colors[color_index], count)
            unique_to_group[color_index] = best_group_index

        return unique_to_group[inverse], groups

    def _clean_mask(
        self,
        mask: np.ndarray,
        *,
        min_component_area_px: int,
        open_radius_px: int,
        close_radius_px: int,
    ) -> np.ndarray:
        cleaned = mask.copy()
        if open_radius_px > 0:
            open_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                ((open_radius_px * 2) + 1, (open_radius_px * 2) + 1),
            )
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, open_kernel)
        if close_radius_px > 0:
            close_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                ((close_radius_px * 2) + 1, (close_radius_px * 2) + 1),
            )
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, close_kernel)
        if min_component_area_px > 0:
            component_count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
            filtered = np.zeros_like(cleaned)
            for component_index in range(1, component_count):
                area = int(stats[component_index, cv2.CC_STAT_AREA])
                if area >= min_component_area_px:
                    filtered[labels == component_index] = 255
            cleaned = filtered
        return cleaned

    def _extract_detail_trace_segments(
        self,
        mask: np.ndarray,
        *,
        min_component_area_px: float,
    ) -> tuple[list[pipeline_core.Segment], int, int]:
        if mask is None or not np.any(mask):
            return [], 0, 0

        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        segments: list[pipeline_core.Segment] = []
        traced_components = 0
        skeleton_pixel_count = 0
        minimum_area = max(0.0, float(min_component_area_px))

        for component_index in range(1, int(component_count)):
            area = float(stats[component_index, cv2.CC_STAT_AREA])
            if area < minimum_area:
                continue

            component_mask = (labels == component_index).astype(np.uint8) * 255
            component_segments = self._component_detail_segments(component_mask)
            if not component_segments:
                continue
            traced_components += 1
            segments.extend(component_segments)
            skeleton_pixel_count += int(np.count_nonzero(self._skeletonize_component(component_mask)))

        return segments, traced_components, skeleton_pixel_count

    def _component_detail_segments(self, component_mask: np.ndarray) -> list[pipeline_core.Segment]:
        segments: list[pipeline_core.Segment] = []
        skeleton = self._skeletonize_component(component_mask)
        skeleton_segments = self._skeleton_to_segments(skeleton)
        if skeleton_segments:
            segments.extend(skeleton_segments)
        elif not segments:
            fallback = self._fallback_component_segment(component_mask)
            if fallback is not None:
                segments.append(fallback)

        if segments:
            return segments

        fallback = self._fallback_component_segment(component_mask)
        return [fallback] if fallback is not None else []

    def _skeletonize_component(self, component_mask: np.ndarray) -> np.ndarray:
        binary = (component_mask > 0).astype(np.uint8)
        if not np.any(binary):
            return binary

        try:
            from skimage.morphology import skeletonize  # type: ignore

            return skeletonize(binary.astype(bool)).astype(np.uint8)
        except Exception:
            pass

        thinning = getattr(getattr(cv2, "ximgproc", None), "thinning", None)
        if thinning is not None:
            try:
                return (thinning((binary * 255).astype(np.uint8)) > 0).astype(np.uint8)
            except Exception:
                pass

        working = (binary * 255).astype(np.uint8)
        skeleton = np.zeros_like(working)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while True:
            eroded = cv2.erode(working, kernel)
            opened = cv2.dilate(eroded, kernel)
            residue = cv2.subtract(working, opened)
            skeleton = cv2.bitwise_or(skeleton, residue)
            working = eroded
            if cv2.countNonZero(working) == 0:
                break
        return (skeleton > 0).astype(np.uint8)

    def _skeleton_to_segments(self, skeleton: np.ndarray) -> list[pipeline_core.Segment]:
        pixels = np.argwhere(skeleton > 0)
        if not len(pixels):
            return []

        pixel_set = {tuple(int(v) for v in pixel) for pixel in pixels}
        neighbors: dict[tuple[int, int], list[tuple[int, int]]] = {}
        for y, x in pixel_set:
            linked: list[tuple[int, int]] = []
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    candidate = (y + dy, x + dx)
                    if candidate in pixel_set:
                        linked.append(candidate)
            neighbors[(y, x)] = linked

        node_pixels = {pixel for pixel, linked in neighbors.items() if len(linked) != 2}
        visited_edges: set[tuple[tuple[int, int], tuple[int, int]]] = set()
        segments: list[pipeline_core.Segment] = []

        def edge_key(a: tuple[int, int], b: tuple[int, int]) -> tuple[tuple[int, int], tuple[int, int]]:
            return (a, b) if a <= b else (b, a)

        def trace_path(start: tuple[int, int], nxt: tuple[int, int]) -> list[tuple[int, int]]:
            path = [start, nxt]
            visited_edges.add(edge_key(start, nxt))
            previous = start
            current = nxt
            while True:
                options = [candidate for candidate in neighbors[current] if candidate != previous]
                if current in node_pixels and current != start:
                    break
                next_step = None
                for candidate in options:
                    key = edge_key(current, candidate)
                    if key not in visited_edges:
                        next_step = candidate
                        visited_edges.add(key)
                        break
                if next_step is None:
                    break
                path.append(next_step)
                previous, current = current, next_step
                if current == start:
                    break
            return path

        for start in sorted(node_pixels):
            for nxt in neighbors[start]:
                key = edge_key(start, nxt)
                if key in visited_edges:
                    continue
                path_pixels = trace_path(start, nxt)
                if len(path_pixels) < 2:
                    continue
                segments.append(
                    pipeline_core.Segment(
                        points=[pipeline_core.Point(float(x) + 0.5, float(y) + 0.5) for y, x in path_pixels],
                        closed=path_pixels[0] == path_pixels[-1],
                    )
                )

        for start in sorted(pixel_set):
            remaining = [candidate for candidate in neighbors[start] if edge_key(start, candidate) not in visited_edges]
            if not remaining:
                continue
            path_pixels = trace_path(start, remaining[0])
            if len(path_pixels) < 2:
                continue
            segments.append(
                pipeline_core.Segment(
                    points=[pipeline_core.Point(float(x) + 0.5, float(y) + 0.5) for y, x in path_pixels],
                    closed=path_pixels[0] == path_pixels[-1],
                )
            )

        return [segment for segment in segments if len(segment.points) >= 2]

    def _fallback_component_segment(self, component_mask: np.ndarray) -> pipeline_core.Segment | None:
        ys, xs = np.nonzero(component_mask > 0)
        if len(xs) == 0:
            return None

        min_x = int(xs.min())
        max_x = int(xs.max())
        min_y = int(ys.min())
        max_y = int(ys.max())
        center_x = int(round(xs.mean()))
        center_y = int(round(ys.mean()))

        horizontal = component_mask[center_y, min_x:max_x + 1] > 0 if 0 <= center_y < component_mask.shape[0] else np.array([], dtype=bool)
        vertical = component_mask[min_y:max_y + 1, center_x] > 0 if 0 <= center_x < component_mask.shape[1] else np.array([], dtype=bool)

        if horizontal.any():
            offsets = np.where(horizontal)[0]
            start_x = float(min_x + int(offsets[0])) + 0.5
            end_x = float(min_x + int(offsets[-1])) + 0.5
            y = float(center_y) + 0.5
            return pipeline_core.Segment(
                points=[pipeline_core.Point(start_x, y), pipeline_core.Point(end_x, y)],
                closed=False,
            )

        if vertical.any():
            offsets = np.where(vertical)[0]
            start_y = float(min_y + int(offsets[0])) + 0.5
            end_y = float(min_y + int(offsets[-1])) + 0.5
            x = float(center_x) + 0.5
            return pipeline_core.Segment(
                points=[pipeline_core.Point(x, start_y), pipeline_core.Point(x, end_y)],
                closed=False,
            )

        center = pipeline_core.Point(float(center_x) + 0.5, float(center_y) + 0.5)
        return pipeline_core.Segment(
            points=[center, pipeline_core.Point(center.x + 0.25, center.y + 0.25)],
            closed=False,
        )

    @staticmethod
    def _contour_ring(contour: np.ndarray) -> list[tuple[float, float]]:
        ring = contour.reshape(-1, 2)
        if len(ring) >= 2 and np.array_equal(ring[0], ring[-1]):
            ring = ring[:-1]
        return [(float(x), float(y)) for x, y in ring]
