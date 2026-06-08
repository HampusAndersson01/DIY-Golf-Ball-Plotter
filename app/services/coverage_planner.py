from __future__ import annotations

from collections import Counter, defaultdict
import heapq
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable

import cv2
import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point as ShapelyPoint, Polygon
from shapely.ops import unary_union

from . import pipeline_core

Point = pipeline_core.Point
Toolpath = pipeline_core.Toolpath

REPAIR_PATH_KIND = "fill-repair"
REPAIR_PATH_SOURCE = "mask_space_coverage_repair"
REPAIR_MAX_ITERATIONS = 8
REPAIR_TARGET_COVERAGE_RATIO = 0.99
REPAIR_MAX_OVERFLOW_RATIO = 0.005
MIN_CENTER_DOMAIN_AREA_MM2 = 0.01
MIN_SAFE_AREA_MM2 = 0.01
THIN_TOL_MM = 0.03


def _geometry_parts(geometry: Any) -> list[Polygon]:
    return pipeline_core.normalize_geometry(geometry) if geometry is not None else []


def _preferred_safe_centerline_geometry(
    geometry: Any,
    *,
    inset_mm: float,
) -> tuple[Any, float]:
    if geometry is None or getattr(geometry, "is_empty", True):
        return geometry, 0.0
    if inset_mm <= 1e-9:
        return geometry, 0.0
    for scale in (1.0, 0.85, 0.7, 0.5):
        trial_inset_mm = float(inset_mm) * float(scale)
        try:
            inset_geometry = geometry.buffer(-trial_inset_mm, join_style=1)
        except Exception:
            inset_geometry = geometry.buffer(0).buffer(-trial_inset_mm, join_style=1)
        if inset_geometry is not None and not getattr(inset_geometry, "is_empty", True):
            return inset_geometry, trial_inset_mm
    return geometry, 0.0


def _strict_safe_centerline_geometry(
    geometry: Any,
    *,
    inset_mm: float,
) -> Any:
    if geometry is None or getattr(geometry, "is_empty", True):
        return geometry
    if inset_mm <= 1e-9:
        return geometry
    try:
        return geometry.buffer(-float(inset_mm), join_style=1)
    except Exception:
        return geometry.buffer(0).buffer(-float(inset_mm), join_style=1)


def _is_repair_path(path: Toolpath) -> bool:
    metadata = path.metadata or {}
    return (
        path.kind == REPAIR_PATH_KIND
        or bool(metadata.get("repair_candidate", False))
        or bool(metadata.get("is_fill_repair", False))
        or str(path.source) == REPAIR_PATH_SOURCE
    )


def _export_repair_toolpath(path: Toolpath, *, strategy_key: str | None = None, blob_id: int | None = None) -> Toolpath:
    metadata = {
        **(path.metadata or {}),
        "repair_candidate": True,
        "is_fill_repair": True,
        "repair_export_kind": REPAIR_PATH_KIND,
        "repair_export_source": REPAIR_PATH_SOURCE,
    }
    if strategy_key:
        metadata["repair_strategy_key"] = str(strategy_key)
    if blob_id is not None:
        metadata["missed_blob_id"] = int(blob_id)
    return Toolpath(
        points=list(path.points),
        kind=REPAIR_PATH_KIND,
        closed=bool(path.closed),
        source=REPAIR_PATH_SOURCE,
        region_id=path.region_id,
        metadata=metadata,
    )


def _component_mask_to_geometry(component_mask: np.ndarray, *, origin_x: float, origin_y: float, px_per_mm: float) -> Any:
    contours, hierarchy = cv2.findContours(component_mask.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None or not contours:
        return None
    hierarchy = hierarchy[0]
    polygons: list[Polygon] = []
    for index, entry in enumerate(hierarchy):
        if int(entry[3]) != -1:
            continue
        shell = _contour_to_ring(contours[index], origin_x=origin_x, origin_y=origin_y, px_per_mm=px_per_mm)
        if len(shell) < 3:
            continue
        holes: list[list[tuple[float, float]]] = []
        child_index = int(entry[2])
        while child_index != -1:
            hole = _contour_to_ring(contours[child_index], origin_x=origin_x, origin_y=origin_y, px_per_mm=px_per_mm)
            if len(hole) >= 3:
                holes.append(hole)
            child_index = int(hierarchy[child_index][0])
        polygon = Polygon(shell, holes)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        polygons.extend(_geometry_parts(polygon))
    if not polygons:
        return None
    return unary_union(polygons) if len(polygons) > 1 else polygons[0]


def _contour_to_ring(contour: np.ndarray, *, origin_x: float, origin_y: float, px_per_mm: float) -> list[tuple[float, float]]:
    ring: list[tuple[float, float]] = []
    for point in contour.reshape(-1, 2):
        x = origin_x + (float(point[0]) / px_per_mm)
        y = origin_y + (float(point[1]) / px_per_mm)
        ring.append((x, y))
    if ring and ring[0] != ring[-1]:
        ring.append(ring[0])
    return ring


def _rasterize_geometry(geometry: Any, *, resolution_mm: float, pad_mm: float) -> tuple[np.ndarray, float, float, float]:
    if geometry is None or getattr(geometry, "is_empty", True):
        mask = np.zeros((8, 8), dtype=np.uint8)
        return mask, 0.0, 0.0, 1.0 / max(0.03, resolution_mm)

    min_x, min_y, max_x, max_y = geometry.bounds
    px_per_mm = 1.0 / max(0.03, float(resolution_mm))
    origin_x = float(min_x) - float(pad_mm)
    origin_y = float(min_y) - float(pad_mm)
    width_px = max(8, int(math.ceil(((float(max_x) - origin_x) + float(pad_mm)) * px_per_mm)))
    height_px = max(8, int(math.ceil(((float(max_y) - origin_y) + float(pad_mm)) * px_per_mm)))
    mask = np.zeros((height_px, width_px), dtype=np.uint8)

    for polygon in _geometry_parts(geometry):
        exterior = np.asarray([
            [int(round((float(x) - origin_x) * px_per_mm)), int(round((float(y) - origin_y) * px_per_mm))]
            for x, y in polygon.exterior.coords
        ], dtype=np.int32)
        if len(exterior) >= 3:
            cv2.fillPoly(mask, [exterior], 255)
        for ring in polygon.interiors:
            hole = np.asarray([
                [int(round((float(x) - origin_x) * px_per_mm)), int(round((float(y) - origin_y) * px_per_mm))]
                for x, y in ring.coords
            ], dtype=np.int32)
            if len(hole) >= 3:
                cv2.fillPoly(mask, [hole], 0)
    return mask, origin_x, origin_y, px_per_mm


def _skeletonize_mask(component_mask: np.ndarray) -> np.ndarray:
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


def _skeleton_to_segments(skeleton: np.ndarray) -> list[list[tuple[int, int]]]:
    pixels = np.argwhere(skeleton > 0)
    if pixels.size == 0:
        return []

    pixel_set = {tuple(int(value) for value in pixel) for pixel in pixels}
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
    segments: list[list[tuple[int, int]]] = []

    def edge_key(a: tuple[int, int], b: tuple[int, int]) -> tuple[tuple[int, int], tuple[int, int]]:
        return (a, b) if a <= b else (b, a)

    def trace_path(start: tuple[int, int], nxt: tuple[int, int]) -> list[tuple[int, int]]:
        path = [start, nxt]
        visited_edges.add(edge_key(start, nxt))
        previous = start
        current = nxt
        while True:
            if current in node_pixels and current != start:
                break
            options = [candidate for candidate in neighbors[current] if candidate != previous]
            next_step = None
            for candidate in options:
                key = edge_key(current, candidate)
                if key not in visited_edges:
                    next_step = candidate
                    break
            if next_step is None:
                if options:
                    next_step = options[0]
                else:
                    break
            visited_edges.add(edge_key(current, next_step))
            path.append(next_step)
            previous = current
            current = next_step
            if len(path) > len(pixel_set) + 4:
                break
        return path

    for pixel in sorted(node_pixels):
        for neighbor in neighbors[pixel]:
            if edge_key(pixel, neighbor) in visited_edges:
                continue
            segments.append(trace_path(pixel, neighbor))

    for pixel in sorted(pixel_set):
        for neighbor in neighbors[pixel]:
            key = edge_key(pixel, neighbor)
            if key in visited_edges:
                continue
            segments.append(trace_path(pixel, neighbor))

    return [segment for segment in segments if len(segment) >= 2]


def _skeleton_neighbors(skeleton: np.ndarray) -> dict[tuple[int, int], list[tuple[int, int]]]:
    pixels = np.argwhere(skeleton > 0)
    if pixels.size == 0:
        return {}
    pixel_set = {tuple(int(value) for value in pixel) for pixel in pixels}
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
    return neighbors


def _connected_skeleton_components(neighbors: dict[tuple[int, int], list[tuple[int, int]]]) -> list[set[tuple[int, int]]]:
    remaining = set(neighbors.keys())
    components: list[set[tuple[int, int]]] = []
    while remaining:
        start = remaining.pop()
        component = {start}
        stack = [start]
        while stack:
            current = stack.pop()
            for neighbor in neighbors.get(current, []):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return components


def _skeleton_edge_key(a: tuple[int, int], b: tuple[int, int]) -> tuple[tuple[int, int], tuple[int, int]]:
    return (a, b) if a <= b else (b, a)


def _skeleton_edge_length_mm(a: tuple[int, int], b: tuple[int, int], *, px_per_mm: float) -> float:
    return float(math.hypot(float(a[1] - b[1]), float(a[0] - b[0])) / max(px_per_mm, 1e-9))


def _subgraph_edge_set(
    nodes: set[tuple[int, int]],
    neighbors: dict[tuple[int, int], list[tuple[int, int]]],
) -> set[tuple[tuple[int, int], tuple[int, int]]]:
    edges: set[tuple[tuple[int, int], tuple[int, int]]] = set()
    for node in nodes:
        for neighbor in neighbors.get(node, []):
            if neighbor in nodes:
                edges.add(_skeleton_edge_key(node, neighbor))
    return edges


def _route_nodes_to_toolpath(
    route_nodes: list[tuple[int, int]],
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    simplify_tolerance_mm: float,
    kind: str,
) -> Toolpath | None:
    if len(route_nodes) < 2:
        return None
    points = [
        Point(
            float(origin_x + (float(node[1]) / max(px_per_mm, 1e-9))),
            float(origin_y + (float(node[0]) / max(px_per_mm, 1e-9))),
        )
        for node in route_nodes
    ]
    simplified = pipeline_core.simplify_segment_points(points, simplify_tolerance_mm, False)
    if len(simplified) < 2 or float(pipeline_core.segment_length(simplified)) <= 1e-9:
        return None
    return Toolpath(points=simplified, kind=kind, closed=False)


def _longest_path_in_edge_subgraph(
    *,
    edges: set[tuple[tuple[int, int], tuple[int, int]]],
    px_per_mm: float,
) -> list[tuple[int, int]]:
    if not edges:
        return []
    adjacency: dict[tuple[int, int], list[tuple[tuple[int, int], float]]] = defaultdict(list)
    for a, b in edges:
        weight = _skeleton_edge_length_mm(a, b, px_per_mm=px_per_mm)
        adjacency[a].append((b, weight))
        adjacency[b].append((a, weight))
    terminals = [node for node, linked in adjacency.items() if len(linked) != 2]

    def _farthest_from(start: tuple[int, int]) -> tuple[tuple[int, int], dict[tuple[int, int], float], dict[tuple[int, int], tuple[int, int] | None]]:
        distances: dict[tuple[int, int], float] = {start: 0.0}
        previous: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        heap: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        while heap:
            current_neg_dist, current = heapq.heappop(heap)
            current_dist = -current_neg_dist
            if current_dist + 1e-9 < distances.get(current, 0.0):
                continue
            for neighbor, weight in adjacency.get(current, []):
                next_dist = current_dist + weight
                if next_dist > distances.get(neighbor, -1.0) + 1e-9:
                    distances[neighbor] = next_dist
                    previous[neighbor] = current
                    heapq.heappush(heap, (-next_dist, neighbor))
        candidates = terminals if len(terminals) >= 2 else list(adjacency.keys())
        end = max(candidates, key=lambda node: distances.get(node, -1.0))
        return end, distances, previous

    seed = terminals[0] if terminals else next(iter(adjacency.keys()))
    first_end, _first_distances, _first_previous = _farthest_from(seed)
    second_end, _second_distances, second_previous = _farthest_from(first_end)
    route: list[tuple[int, int]] = []
    cursor: tuple[int, int] | None = second_end
    while cursor is not None:
        route.append(cursor)
        cursor = second_previous.get(cursor)
    route.reverse()
    return route if len(route) >= 2 else []


def _cover_skeleton_subgraph_with_routes(
    *,
    nodes: set[tuple[int, int]],
    neighbors: dict[tuple[int, int], list[tuple[int, int]]],
    px_per_mm: float,
    origin_x: float,
    origin_y: float,
    simplify_tolerance_mm: float,
    kind: str,
    min_route_length_mm: float,
) -> tuple[list[Toolpath], float]:
    if not nodes:
        return [], 0.0
    max_y = max(node[0] for node in nodes)
    max_x = max(node[1] for node in nodes)
    selected_mask = np.zeros((max_y + 1, max_x + 1), dtype=np.uint8)
    for y, x in nodes:
        selected_mask[y, x] = 1
    segments = sorted(_skeleton_to_segments(selected_mask), key=len, reverse=True)
    routes: list[Toolpath] = []
    useful_length_mm = 0.0
    for segment in segments:
        route = _route_nodes_to_toolpath(
            segment,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            simplify_tolerance_mm=simplify_tolerance_mm,
            kind=kind,
        )
        if route is None:
            continue
        route_length_mm = float(pipeline_core.segment_length(route.points))
        useful_length_mm += route_length_mm
        if route_length_mm >= min_route_length_mm:
            routes.append(route)
    return routes, useful_length_mm


def _pixel_path_to_toolpath(
    points_px: list[tuple[float, float]],
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    kind: str,
    source: str,
    component_id: int,
    metadata: dict[str, Any] | None = None,
    closed: bool = False,
) -> Toolpath | None:
    if len(points_px) < 2:
        return None
    points_mm = [Point(origin_x + (float(x) / px_per_mm), origin_y + (float(y) / px_per_mm)) for x, y in points_px]
    if pipeline_core.segment_length(points_mm) < 0.02:
        return None
    merged_metadata = {"source_region_id": f"component_{int(component_id):03d}", **(metadata or {})}
    return Toolpath(
        points=points_mm,
        kind=kind,
        closed=closed,
        source=source,
        region_id=int(component_id),
        metadata=merged_metadata,
    )


def _component_fill_angle(component_mask: np.ndarray) -> float:
    ys, xs = np.nonzero(component_mask > 0)
    if xs.size < 2:
        return 0.0
    pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
    try:
        vx, vy, _x0, _y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx_f = float(np.asarray(vx).reshape(-1)[0])
        vy_f = float(np.asarray(vy).reshape(-1)[0])
        angle = math.degrees(math.atan2(vy_f, vx_f))
        return _normalize_angle(angle)
    except Exception:
        width = float(xs.max() - xs.min())
        height = float(ys.max() - ys.min())
        return 0.0 if width >= height else 90.0


def _normalize_angle(angle_deg: float) -> float:
    value = float(angle_deg) % 180.0
    if value > 90.0:
        value -= 180.0
    return value


def _largest_blob_diameter_mm(mask: np.ndarray, px_per_mm: float) -> float:
    if mask is None or mask.size == 0 or not np.any(mask > 0):
        return 0.0
    comp_count, _labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
    if comp_count <= 1:
        return 0.0
    largest = max(int(stats[index, cv2.CC_STAT_AREA]) for index in range(1, int(comp_count)))
    return 2.0 * math.sqrt(float(largest) / max(1e-9, math.pi * px_per_mm * px_per_mm))


def _clip_extended_segment_to_allowed_area(
    segment: LineString,
    *,
    sweep_direction: tuple[float, float],
    endpoint_extension_mm: float,
    line_width_mm: float,
    allowed_geom: Any,
) -> tuple[LineString | None, bool]:
    if segment.is_empty or segment.length <= 1e-9:
        return None, False
    coords = list(segment.coords)
    start = coords[0]
    end = coords[-1]
    ux = float(end[0]) - float(start[0])
    uy = float(end[1]) - float(start[1])
    direction_norm = math.hypot(ux, uy)
    if direction_norm <= 1e-9:
        ux, uy = sweep_direction
        direction_norm = math.hypot(ux, uy)
    if direction_norm <= 1e-9:
        return segment, False
    ux /= direction_norm
    uy /= direction_norm
    ext = max(0.0, float(endpoint_extension_mm))
    clipped = False
    radius = max(0.01, float(line_width_mm) * 0.5)
    for _ in range(18):
        candidate = LineString([
            (float(start[0]) - (ux * ext), float(start[1]) - (uy * ext)),
            (float(end[0]) + (ux * ext), float(end[1]) + (uy * ext)),
        ])
        if candidate.buffer(radius, cap_style=1, join_style=1).within(allowed_geom):
            return candidate, clipped
        ext *= 0.5
        clipped = True
    return segment, True


def _clone_toolpath(path: Toolpath) -> Toolpath:
    return Toolpath(
        points=[Point(float(point.x), float(point.y)) for point in path.points],
        kind=path.kind,
        closed=bool(path.closed),
        source=path.source,
        metadata=dict(path.metadata or {}),
        path_id=path.path_id,
        region_id=path.region_id,
    )


def _clip_toolpath_to_geometry(
    path: Toolpath,
    *,
    allowed_geom: Any,
    simplify_tolerance_mm: float,
) -> Toolpath | None:
    if allowed_geom is None or getattr(allowed_geom, "is_empty", True) or len(path.points) < 2:
        return _clone_toolpath(path)
    line = LineString([(float(point.x), float(point.y)) for point in path.points])
    clipped = line.intersection(allowed_geom)
    parts = [
        part
        for part in pipeline_core.extract_lines(clipped)
        if not getattr(part, "is_empty", True) and float(part.length) > 1e-9
    ]
    if not parts:
        return None
    best_part = max(parts, key=lambda part: float(part.length))
    clipped_points = pipeline_core.simplify_segment_points(
        [Point(float(x), float(y)) for x, y in best_part.coords],
        simplify_tolerance_mm,
        False,
    )
    if len(clipped_points) < 2:
        return None
    return Toolpath(
        points=clipped_points,
        kind=path.kind,
        closed=False,
        source=path.source,
        metadata=dict(path.metadata or {}),
        path_id=path.path_id,
        region_id=path.region_id,
    )


def _endpoint_protrusion_area_mm2(
    point: Point,
    *,
    allowed_geom: Any,
    pen_radius_mm: float,
) -> float:
    if allowed_geom is None or getattr(allowed_geom, "is_empty", True):
        return 0.0
    endpoint_cap = ShapelyPoint(float(point.x), float(point.y)).buffer(max(0.01, float(pen_radius_mm)), cap_style=1, join_style=1)
    return float(endpoint_cap.difference(allowed_geom).area)


def _retract_endpoint_along_path(
    points: list[Point],
    *,
    endpoint_index: int,
    allowed_geom: Any,
    pen_radius_mm: float,
    max_retract_mm: float,
    precision_mm: float,
) -> tuple[list[Point], float]:
    if len(points) < 2:
        return points, 0.0
    if endpoint_index == 0:
        outer_point = points[0]
        inner_point = points[1]
    else:
        outer_point = points[-1]
        inner_point = points[-2]

    dx = float(inner_point.x - outer_point.x)
    dy = float(inner_point.y - outer_point.y)
    segment_length = math.hypot(dx, dy)
    if segment_length <= 1e-9:
        return points, 0.0
    if _endpoint_protrusion_area_mm2(outer_point, allowed_geom=allowed_geom, pen_radius_mm=pen_radius_mm) <= 1e-9:
        return points, 0.0

    ux = dx / segment_length
    uy = dy / segment_length
    max_step = max(0.0, min(float(max_retract_mm), max(0.0, segment_length - 1e-6)))
    if max_step <= 1e-9:
        return points, 0.0

    def moved_point(distance_mm: float) -> Point:
        return Point(
            float(outer_point.x + (ux * distance_mm)),
            float(outer_point.y + (uy * distance_mm)),
        )

    def protrusion_at(distance_mm: float) -> float:
        return _endpoint_protrusion_area_mm2(
            moved_point(distance_mm),
            allowed_geom=allowed_geom,
            pen_radius_mm=pen_radius_mm,
        )

    high = max_step
    if protrusion_at(high) > 1e-9:
        chosen_distance = high
    else:
        low = 0.0
        while (high - low) > max(0.001, float(precision_mm)):
            mid = (low + high) * 0.5
            if protrusion_at(mid) <= 1e-9:
                high = mid
            else:
                low = mid
        chosen_distance = high

    if chosen_distance <= 1e-9:
        return points, 0.0

    updated = list(points)
    if endpoint_index == 0:
        updated[0] = moved_point(chosen_distance)
    else:
        updated[-1] = moved_point(chosen_distance)
    return updated, float(chosen_distance)


def _clamp_infill_endpoints_to_outline_limit(
    paths: list[Toolpath],
    *,
    allowed_geom: Any,
    pen_radius_mm: float,
    max_retract_mm: float,
    precision_mm: float,
) -> tuple[list[Toolpath], dict[str, Any]]:
    adjusted_paths: list[Toolpath] = []
    endpoints_checked = 0
    endpoints_clamped = 0
    max_endpoint_retract_mm = 0.0

    for path in paths:
        if path.kind != "fill-infill" or len(path.points) < 2:
            adjusted_paths.append(_clone_toolpath(path))
            continue
        scanline_spacing_mm = float(path.metadata.get("scanline_spacing_mm", 0.0) or 0.0)
        preserve_small_detail_coverage = bool(
            "detail" in str(path.metadata.get("fill_mode", "")).lower()
            or str(path.source or "") == "detail_serpentine_fill"
            or (
                str(path.source or "") == "coverage_serpentine_fill"
                and scanline_spacing_mm >= (float(pen_radius_mm) * 2.0 * 0.995)
            )
        )
        if preserve_small_detail_coverage:
            adjusted_paths.append(_clone_toolpath(path))
            continue
        updated_points = [Point(float(point.x), float(point.y)) for point in path.points]
        for endpoint_index in (0, -1):
            endpoints_checked += 1
            updated_points, retract_mm = _retract_endpoint_along_path(
                updated_points,
                endpoint_index=endpoint_index,
                allowed_geom=allowed_geom,
                pen_radius_mm=pen_radius_mm,
                max_retract_mm=max_retract_mm,
                precision_mm=precision_mm,
            )
            if retract_mm > 1e-9:
                endpoints_clamped += 1
                max_endpoint_retract_mm = max(max_endpoint_retract_mm, float(retract_mm))
        adjusted_paths.append(Toolpath(
            points=updated_points,
            kind=path.kind,
            closed=path.closed,
            source=path.source,
            metadata=dict(path.metadata or {}),
            path_id=path.path_id,
            region_id=path.region_id,
        ))

    return adjusted_paths, {
        "endpoint_clamp_mode": "postprocess_only",
        "endpoints_checked": int(endpoints_checked),
        "endpoints_clamped": int(endpoints_clamped),
        "max_endpoint_retract_mm": float(max_endpoint_retract_mm),
    }


def _scanline_fill_paths(
    component_geometry: Any,
    *,
    angle_deg: float,
    spacing_mm: float,
    line_width_mm: float,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    component_id: int,
    allow_connectors: bool,
    max_overflow_mm: float,
    fill_mode_label: str = "serpentine",
    intersection_geometry: Any | None = None,
    endpoint_limit_geometry: Any | None = None,
    centerline_generation_mode: str = "raw-mask",
    safe_centerline_inset_mm: float = 0.0,
) -> tuple[list[Toolpath], dict[str, Any]]:
    base_geometry = component_geometry
    fill_geometry = intersection_geometry
    if intersection_geometry is not None and getattr(intersection_geometry, "is_empty", True):
        return [], {
            "segment_count": 0,
            "row_count": 0,
            "connector_count": 0,
            "endpoint_extension_mm": float(line_width_mm * 0.5),
            "endpoint_extensions_added": 0,
            "endpoint_extensions_clipped": 0,
            "fill_geometry_empty": True,
            "centerline_generation_mode": str(centerline_generation_mode),
            "safe_centerline_inset_mm": float(safe_centerline_inset_mm),
        }
    if fill_geometry is None or getattr(fill_geometry, "is_empty", True):
        fill_geometry = component_geometry
    if fill_geometry is None or getattr(fill_geometry, "is_empty", True):
        return [], {
            "segment_count": 0,
            "row_count": 0,
            "connector_count": 0,
            "endpoint_extension_mm": float(line_width_mm * 0.5),
            "endpoint_extensions_added": 0,
            "endpoint_extensions_clipped": 0,
            "fill_geometry_empty": True,
        }

    centroid_source = base_geometry if base_geometry is not None and not getattr(base_geometry, "is_empty", True) else fill_geometry
    centroid = centroid_source.representative_point()
    rotated_fill = affinity.rotate(fill_geometry, -angle_deg, origin=(centroid.x, centroid.y))
    min_x, min_y, max_x, max_y = rotated_fill.bounds
    # When spacing is effectively equal to pen width, use a small overlap so
    # coverage does not depend on perfect tangent contact between rows.
    step_scale = 0.85 if float(spacing_mm) >= (float(line_width_mm) * 0.995) else 1.0
    step = max(0.2, float(spacing_mm) * step_scale)
    # Keep short but valid scanline spans in narrow glyph/detail regions.
    # The previous 0.15x pen-width floor (0.09 mm at 0.6 mm pen width)
    # caused alternating rows to disappear in tapered shapes.
    min_segment_length_mm = max(0.02, min(line_width_mm * 0.10, max(0.04, spacing_mm * 0.125)))
    allowed_geom = endpoint_limit_geometry
    if allowed_geom is None or getattr(allowed_geom, "is_empty", True):
        allowed_geom = fill_geometry.buffer(max_overflow_mm, join_style=1) if max_overflow_mm > 0 else fill_geometry
    rotated_allowed_geom = affinity.rotate(allowed_geom, -angle_deg, origin=(centroid.x, centroid.y))
    endpoint_extension_mm = max(0.0, line_width_mm * 0.5)

    toolpaths: list[Toolpath] = []
    row_count = 0
    segment_count = 0
    connector_count = 0
    endpoint_extensions_added = 0
    endpoint_extensions_clipped = 0
    last_end_px: tuple[float, float] | None = None
    row_index = 0
    y = min_y - step
    while y <= max_y + step:
        scan = LineString([(min_x - step, y), (max_x + step, y)])
        row_segments = [segment for segment in pipeline_core.extract_lines(rotated_fill.intersection(scan)) if segment.length >= min_segment_length_mm]
        if not row_segments:
            y += step
            continue
        row_count += 1
        row_segments.sort(key=lambda segment: segment.coords[0][0])
        if row_index % 2 == 1:
            row_segments = [LineString(list(reversed(list(segment.coords)))) for segment in reversed(row_segments)]
        else:
            row_segments = [LineString(list(segment.coords)) for segment in row_segments]
        row_index += 1

        for segment in row_segments:
            coords = list(segment.coords)
            if len(coords) < 2:
                continue
            segment, segment_clipped = _clip_extended_segment_to_allowed_area(
                segment,
                sweep_direction=(1.0, 0.0),
                endpoint_extension_mm=endpoint_extension_mm,
                line_width_mm=line_width_mm,
                allowed_geom=rotated_allowed_geom,
            )
            if segment is None or segment.length < min_segment_length_mm:
                continue
            segment_count += 1
            endpoint_extensions_added += 2
            if segment_clipped:
                endpoint_extensions_clipped += 1
            scanline_offset_mm = float(y - min_y)
            world = affinity.rotate(segment, angle_deg, origin=(centroid.x, centroid.y))
            start = world.coords[0]
            end = world.coords[-1]
            if last_end_px is not None and allow_connectors:
                connector = LineString([last_end_px, start])
                if connector.length > 1e-9:
                    sample_count = max(2, int(math.ceil(connector.length / max(0.05, line_width_mm * 0.2))) + 1)
                    connector_inside = True
                    for sample_index in range(sample_count):
                        distance_mm = min(connector.length, (connector.length * sample_index) / max(sample_count - 1, 1))
                        sample_point = connector.interpolate(distance_mm)
                        if not fill_geometry.covers(ShapelyPoint(float(sample_point.x), float(sample_point.y))):
                            connector_inside = False
                            break
                    if connector_inside and connector.buffer(max(0.01, line_width_mm * 0.15), cap_style=1, join_style=1).within(allowed_geom):
                        connector_points = [Point(float(last_end_px[0]), float(last_end_px[1])), Point(float(start[0]), float(start[1]))]
                        connector_path = Toolpath(
                            points=connector_points,
                            kind="fill-infill-travel",
                            closed=False,
                            source="coverage_connector",
                            region_id=int(component_id),
                            metadata={
                                "coverage_connector": True,
                                "component_id": int(component_id),
                                "connector_kind": "serpentine_row",
                                "connector_pen_down_allowed": True,
                            },
                        )
                        toolpaths.append(connector_path)
                        connector_count += 1
            path_points = [Point(float(x), float(y)) for x, y in world.coords]
            if pipeline_core.segment_length(path_points) >= 0.02:
                toolpaths.append(Toolpath(
                    points=path_points,
                    kind="fill-infill",
                    closed=False,
                    source="coverage_serpentine_fill",
                    region_id=int(component_id),
                    metadata={
                        "source_region_id": f"component_{int(component_id):03d}",
                        "fill_mode": str(fill_mode_label),
                        "component_id": int(component_id),
                        "fill_angle_deg": float(angle_deg),
                        "scanline_spacing_mm": float(step),
                        "scanline_offset_mm": float(scanline_offset_mm),
                        "scanline_row_index": int(row_index),
                        "endpoint_extension_mm": float(endpoint_extension_mm),
                        "endpoint_extension_clipped": bool(segment_clipped),
                        "centerline_generation_mode": str(centerline_generation_mode),
                        "safe_centerline_inset_mm": float(safe_centerline_inset_mm),
                        "generated_from_safe_mask": bool(str(centerline_generation_mode) == "safe-mask"),
                        "fill_strategy": "CONTOUR_PARALLEL_DETAIL" if str(fill_mode_label) == "detail_contour_cell" else "SERPENTINE",
                        "small_detail_fill_style": "contour_following",
                    },
                ))
                last_end_px = end
        y += step

    return toolpaths, {
        "segment_count": segment_count,
        "row_count": row_count,
        "connector_count": connector_count,
        "endpoint_extension_mm": float(endpoint_extension_mm),
        "endpoint_extensions_added": int(endpoint_extensions_added),
        "endpoint_extensions_clipped": int(endpoint_extensions_clipped),
        "fill_geometry_empty": False,
        "centerline_generation_mode": str(centerline_generation_mode),
        "safe_centerline_inset_mm": float(safe_centerline_inset_mm),
    }


def _skeleton_paths_for_component(
    component_mask: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    component_id: int,
    line_width_mm: float,
    small_detail_fill_style: str,
) -> tuple[list[Toolpath], dict[str, Any]]:
    skeleton = _skeletonize_mask(component_mask)
    segments = _skeleton_to_segments(skeleton)
    paths: list[Toolpath] = []
    for index, segment in enumerate(segments):
        points_px = [(float(x), float(y)) for y, x in segment]
        path = _pixel_path_to_toolpath(
            points_px,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            kind="fill-infill",
            source="coverage_skeleton",
            component_id=component_id,
            metadata={
                "fill_mode": "single_stroke_detail",
                "fill_strategy": "SINGLE_STROKE_DETAIL",
                "component_id": int(component_id),
                "source_region_id": f"component_{int(component_id):03d}",
                "skeleton_segment_index": int(index),
                "small_detail_fill_style": str(small_detail_fill_style),
                "force_minimum_printable_stroke": True,
            },
        )
        if path is not None and pipeline_core.segment_length(path.points) >= max(0.02, line_width_mm * 0.1):
            paths.append(path)
    return paths, {"skeleton_segment_count": len(segments)}


def _thin_detail_fill_paths_for_component(
    component_mask: np.ndarray,
    *,
    component_geometry: Any,
    component_endpoint_limit: Any,
    mark_outline_fallback: bool,
    prefer_connected_centerline: bool,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    component_id: int,
    line_width_mm: float,
    simplify_tolerance_mm: float,
    detail_fill_spacing_mm: float,
    angle_deg: float,
    small_detail_fill_style: str,
) -> tuple[list[Toolpath], dict[str, Any]]:
    base_paths, base_stats = _skeleton_paths_for_component(
        component_mask,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        component_id=component_id,
        line_width_mm=line_width_mm,
        small_detail_fill_style=small_detail_fill_style,
    )
    if component_geometry is None or getattr(component_geometry, "is_empty", True):
        return base_paths, dict(base_stats)

    region_mask = (component_mask > 0).astype(np.uint8)
    current_to_source = (
        float(px_per_mm),
        0.0,
        0.0,
        float(px_per_mm),
        -float(origin_x) * float(px_per_mm),
        -float(origin_y) * float(px_per_mm),
    )
    pen_radius_px = max(1, int(round(line_width_mm * px_per_mm / 2.0)))
    pen_radius_mm = line_width_mm * 0.5
    dt = cv2.distanceTransform(region_mask, cv2.DIST_L2, 3)
    width_mm = float(np.max(dt) * 2.0 / max(1e-9, px_per_mm)) if np.any(dt > 0) else 0.0
    safe_component_geometry = _strict_safe_centerline_geometry(component_geometry, inset_mm=pen_radius_mm)
    safe_component_geometry_empty = bool(
        safe_component_geometry is None or getattr(safe_component_geometry, "is_empty", True)
    )

    def _coverage_stats(paths: list[Toolpath]) -> tuple[np.ndarray, dict[str, float], list[dict[str, Any]], dict[str, float | int]]:
        painted = _path_points_to_mask(
            paths,
            shape=region_mask.shape,
            current_to_source_matrix=current_to_source,
            pen_radius_px=pen_radius_px,
        )
        metrics = _painted_metrics(
            target_mask=region_mask,
            painted_mask=painted,
            allowed_mask=region_mask,
            px_per_mm=px_per_mm,
        )
        blobs = _connected_mask_blob_rows(
            ((region_mask > 0) & ~(painted > 0)).astype(np.uint8),
            px_per_mm=px_per_mm,
            min_blob_area_mm2=0.01,
        )
        return painted, metrics, blobs, _blob_summary(blobs)

    current_paths = list(base_paths)
    _painted, metrics_before, blobs_before, summary_before = _coverage_stats(current_paths)
    coverage_target_percent = 98.0
    largest_allowed_blob_mm = 0.10
    has_holes = any(len(polygon.interiors) > 0 for polygon in _geometry_parts(component_geometry))

    supplemental_paths_added = 0
    repair_paths_added = 0
    repair_iterations = 0
    prefer_single_centerline = bool(prefer_connected_centerline or (not has_holes and width_mm <= (line_width_mm * 1.35)))
    if mark_outline_fallback:
        for path in base_paths:
            path.metadata = {
                **(path.metadata or {}),
                "actual_outline_centerline": True,
                "path_role": "FINAL_OUTLINE_FALLBACK",
                "outline_generation_source": "detail_centerline_fallback",
                "outline_fallback_mode": "centerline_detail",
            }

    if (
        not prefer_single_centerline
        and not safe_component_geometry_empty
        and (
        has_holes
        or float(metrics_before["coverage_percent"]) < coverage_target_percent
        or float(summary_before["largest_missed_blob_equivalent_diameter_mm"]) > largest_allowed_blob_mm
        )
    ):
        supplemental_paths, supplemental_stats = _scanline_fill_paths(
            component_geometry,
            angle_deg=angle_deg,
            spacing_mm=detail_fill_spacing_mm,
            line_width_mm=line_width_mm,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            component_id=component_id,
            allow_connectors=False,
            max_overflow_mm=0.0,
            fill_mode_label="thin_detail_serpentine_fill",
            intersection_geometry=safe_component_geometry,
            endpoint_limit_geometry=safe_component_geometry,
            centerline_generation_mode="safe-mask",
            safe_centerline_inset_mm=pen_radius_mm,
        )
        supplemental_paths, _endpoint_clamp_stats = _clamp_infill_endpoints_to_outline_limit(
            supplemental_paths,
            allowed_geom=safe_component_geometry,
            pen_radius_mm=pen_radius_mm,
            max_retract_mm=pen_radius_mm,
            precision_mm=0.02,
        )
        for path in supplemental_paths:
            path.source = "thin_detail_serpentine_fill"
            path.kind = "fill-infill"
            path.metadata = {
                **(path.metadata or {}),
                "fill_mode": "thin_detail_serpentine_fill",
                "fill_strategy": "DETAIL_SERPENTINE_FILL",
                "small_detail_fill_style": "scanline_backfill",
                "detail_region_component_id": int(component_id),
                "detail_region_classification": "thin",
            }
        current_paths.extend(supplemental_paths)
        supplemental_paths_added = len(supplemental_paths)
        base_stats["thin_detail_supplement_segments"] = int(supplemental_stats.get("segment_count", 0))

    current_painted, metrics_after_supplement, current_blob_rows, current_blob_summary = _coverage_stats(current_paths)
    max_repair_iterations = 8
    while (
        not prefer_single_centerline
        and
        repair_iterations < max_repair_iterations
        and current_blob_rows
        and (
            float(metrics_after_supplement["coverage_percent"]) < coverage_target_percent
            or float(current_blob_summary["largest_missed_blob_equivalent_diameter_mm"]) > largest_allowed_blob_mm
        )
    ):
        best_choice: dict[str, Any] | None = None
        for blob_row in current_blob_rows:
            blob_mask = np.asarray(blob_row["mask"], dtype=np.uint8)
            blob_geometry = _component_mask_to_geometry(
                blob_mask * 255,
                origin_x=origin_x,
                origin_y=origin_y,
                px_per_mm=px_per_mm,
            )
            candidate_groups = defaultdict(list)
            for strategy_key, candidate in _detail_repair_candidates_for_blob(
                blob_mask,
                blob_geometry=blob_geometry,
                origin_x=origin_x,
                origin_y=origin_y,
                px_per_mm=px_per_mm,
                line_width_mm=line_width_mm,
                simplify_tolerance_mm=simplify_tolerance_mm,
                detail_fill_spacing_mm=detail_fill_spacing_mm,
                region_component_id=component_id,
                region_area_mm2=float(component_geometry.area),
                region_max_width_mm=0.0,
                region_equivalent_diameter_mm=_equivalent_diameter_mm(float(component_geometry.area)),
                main_angle_deg=angle_deg,
            ):
                candidate_groups[strategy_key].append(candidate)

            for strategy_key in [
                "serpentine",
                "contour_offset",
                "parallel_stroke",
                "perpendicular_stroke",
                "centerline_fallback",
            ]:
                for candidate in candidate_groups.get(strategy_key, []):
                    candidate_mask = _path_points_to_mask(
                        [candidate],
                        shape=region_mask.shape,
                        current_to_source_matrix=current_to_source,
                        pen_radius_px=pen_radius_px,
                    )
                    trial_painted = np.maximum(current_painted, candidate_mask)
                    trial_metrics = _painted_metrics(
                        target_mask=region_mask,
                        painted_mask=trial_painted,
                        allowed_mask=region_mask,
                        px_per_mm=px_per_mm,
                    )
                    candidate_footprint = _paths_footprint_union([candidate], pen_radius_mm=pen_radius_mm)
                    newly_covered_target_area_mm2 = max(
                        0.0,
                        float(trial_metrics["painted_inside_area_mm2"]) - float(metrics_after_supplement["painted_inside_area_mm2"]),
                    )
                    outside_region_overflow_mm2 = (
                        float(candidate_footprint.difference(component_endpoint_limit).area)
                        if candidate_footprint is not None and not getattr(candidate_footprint, "is_empty", True)
                        else 0.0
                    )
                    if newly_covered_target_area_mm2 <= 0.003 or outside_region_overflow_mm2 > 0.005:
                        continue
                    score = float(newly_covered_target_area_mm2) - (outside_region_overflow_mm2 * 0.5)
                    if best_choice is None or score > float(best_choice["score"]):
                        best_choice = {
                            "candidate": candidate,
                            "score": float(score),
                            "trial_painted": trial_painted,
                            "trial_metrics": trial_metrics,
                        }
                if best_choice is not None:
                    break
            if best_choice is not None:
                break

        if best_choice is None:
            break

        current_paths.append(best_choice["candidate"])
        repair_paths_added += 1
        repair_iterations += 1
        current_painted = best_choice["trial_painted"]
        metrics_after_supplement = best_choice["trial_metrics"]
        current_blob_rows = _connected_mask_blob_rows(
            ((region_mask > 0) & ~(current_painted > 0)).astype(np.uint8),
            px_per_mm=px_per_mm,
            min_blob_area_mm2=0.01,
        )
        current_blob_summary = _blob_summary(current_blob_rows)

    _painted, metrics_after, blobs_after, summary_after = _coverage_stats(current_paths)
    return current_paths, {
        **dict(base_stats),
        "thin_detail_initial_coverage_percent": float(metrics_before["coverage_percent"]),
        "thin_detail_initial_largest_missed_blob_mm": float(summary_before["largest_missed_blob_equivalent_diameter_mm"]),
        "thin_detail_final_coverage_percent": float(metrics_after["coverage_percent"]),
        "thin_detail_final_largest_missed_blob_mm": float(summary_after["largest_missed_blob_equivalent_diameter_mm"]),
        "thin_detail_supplemental_paths_added": int(supplemental_paths_added),
        "thin_detail_repair_paths_added": int(repair_paths_added),
        "thin_detail_repair_iterations": int(repair_iterations),
        "thin_detail_has_holes": bool(has_holes),
        "thin_detail_missed_blob_count_after": int(len(blobs_after)),
        "thin_detail_prefer_single_centerline": bool(prefer_single_centerline),
        "thin_detail_estimated_width_mm": float(width_mm),
        "thin_detail_safe_centerline_geometry_empty": bool(safe_component_geometry_empty),
    }


def _boundary_paths_for_component(
    component_geometry: Any,
    *,
    component_id: int,
    simplify_tolerance_mm: float,
    line_width_mm: float,
    centerline_offset_mm: float = 0.0,
) -> list[Toolpath]:
    if component_geometry is None or getattr(component_geometry, "is_empty", True):
        return []
    outline_geometry = component_geometry
    if centerline_offset_mm > 0.0:
        outline_geometry = component_geometry.buffer(-float(centerline_offset_mm), join_style=1)
        if outline_geometry is None or getattr(outline_geometry, "is_empty", True):
            return []
    boundary_paths: list[Toolpath] = []
    simplify_mm = max(0.0, min(max(simplify_tolerance_mm, 0.02), max(0.08, line_width_mm * 0.35)))
    outline_generation_source = "final_outline_offset" if centerline_offset_mm > 0.0 else "final_target_mask"
    for polygon in _geometry_parts(outline_geometry):
        exterior = pipeline_core.simplify_segment_points(
            [Point(float(x), float(y)) for x, y in polygon.exterior.coords],
            simplify_mm,
            True,
        )
        if len(exterior) >= 4:
            boundary_paths.append(
                Toolpath(
                    points=exterior,
                    kind="outline",
                    closed=True,
                    source="coverage_boundary",
                    region_id=int(component_id),
                    metadata={
                        "boundary_kind": "exterior",
                        "ring_role": "outer",
                        "component_id": int(component_id),
                        "source_region_id": f"component_{int(component_id):03d}",
                        "path_role": "FINAL_OUTER_OUTLINE",
                        "generated_from": "final_fill_clip_polygon",
                        "outline_generation_source": outline_generation_source,
                        "source_polygon_matches_infill_clip_polygon": True,
                        "outline_uses_infill_clip_polygon": True,
                        "offset_mm": float(centerline_offset_mm),
                        "outline_centerline_offset_mm": float(centerline_offset_mm),
                        "outline_offset_mm": -float(centerline_offset_mm),
                        "simplify_tolerance_mm": float(simplify_mm),
                    },
                )
            )
        for ring in polygon.interiors:
            inner = pipeline_core.simplify_segment_points(
                [Point(float(x), float(y)) for x, y in ring.coords],
                simplify_mm,
                True,
            )
            if len(inner) >= 4:
                boundary_paths.append(
                    Toolpath(
                        points=inner,
                        kind="outline",
                        closed=True,
                        source="coverage_boundary",
                        region_id=int(component_id),
                        metadata={
                            "boundary_kind": "hole",
                            "ring_role": "hole",
                            "component_id": int(component_id),
                            "source_region_id": f"component_{int(component_id):03d}",
                            "path_role": "FINAL_INNER_OUTLINE",
                            "generated_from": "final_fill_clip_polygon",
                            "outline_generation_source": outline_generation_source,
                            "source_polygon_matches_infill_clip_polygon": True,
                            "outline_uses_infill_clip_polygon": True,
                            "offset_mm": float(centerline_offset_mm),
                            "outline_centerline_offset_mm": float(centerline_offset_mm),
                            "outline_offset_mm": -float(centerline_offset_mm),
                            "simplify_tolerance_mm": float(simplify_mm),
                            "is_hole": True,
                        },
                    )
                )
    return boundary_paths


def _boundary_paths_for_geometry(
    geometry: Any,
    *,
    simplify_tolerance_mm: float,
    line_width_mm: float,
    centerline_offset_mm: float = 0.0,
) -> list[Toolpath]:
    if geometry is None or getattr(geometry, "is_empty", True):
        return []
    outline_geometry = geometry
    if centerline_offset_mm > 0.0:
        outline_geometry = geometry.buffer(-float(centerline_offset_mm), join_style=1)
        if outline_geometry is None or getattr(outline_geometry, "is_empty", True):
            return []
    boundary_paths: list[Toolpath] = []
    simplify_mm = max(0.0, min(float(simplify_tolerance_mm), max(0.08, line_width_mm * 0.35)))
    outline_generation_source = "final_outline_offset" if centerline_offset_mm > 0.0 else "final_target_mask"
    for component_id, polygon in enumerate(_geometry_parts(outline_geometry), start=1):
        exterior = pipeline_core.simplify_segment_points(
            [Point(float(x), float(y)) for x, y in polygon.exterior.coords],
            simplify_mm,
            True,
        )
        if len(exterior) >= 4:
            boundary_paths.append(
                Toolpath(
                    points=exterior,
                    kind="outline",
                    closed=True,
                    source="coverage_boundary",
                    region_id=int(component_id),
                    metadata={
                        "boundary_kind": "exterior",
                        "ring_role": "outer",
                        "component_id": int(component_id),
                        "source_region_id": f"component_{int(component_id):03d}",
                        "path_role": "FINAL_OUTER_OUTLINE",
                        "generated_from": "final_fill_clip_polygon",
                        "outline_generation_source": outline_generation_source,
                        "source_polygon_matches_infill_clip_polygon": True,
                        "outline_uses_infill_clip_polygon": True,
                        "offset_mm": float(centerline_offset_mm),
                        "outline_centerline_offset_mm": float(centerline_offset_mm),
                        "outline_offset_mm": -float(centerline_offset_mm),
                        "simplify_tolerance_mm": float(simplify_mm),
                    },
                )
            )
        for ring in polygon.interiors:
            inner = pipeline_core.simplify_segment_points(
                [Point(float(x), float(y)) for x, y in ring.coords],
                simplify_mm,
                True,
            )
            if len(inner) >= 4:
                boundary_paths.append(
                    Toolpath(
                        points=inner,
                        kind="outline",
                        closed=True,
                        source="coverage_boundary",
                        region_id=int(component_id),
                        metadata={
                            "boundary_kind": "hole",
                            "ring_role": "hole",
                            "component_id": int(component_id),
                            "source_region_id": f"component_{int(component_id):03d}",
                            "path_role": "FINAL_INNER_OUTLINE",
                            "generated_from": "final_fill_clip_polygon",
                            "outline_generation_source": outline_generation_source,
                            "source_polygon_matches_infill_clip_polygon": True,
                            "outline_uses_infill_clip_polygon": True,
                            "offset_mm": float(centerline_offset_mm),
                            "outline_centerline_offset_mm": float(centerline_offset_mm),
                            "outline_offset_mm": -float(centerline_offset_mm),
                            "simplify_tolerance_mm": float(simplify_mm),
                            "is_hole": True,
                        },
                    )
                )
    return boundary_paths


def _promote_outline_fallback_path(path: Toolpath) -> Toolpath:
    metadata = {
        **(path.metadata or {}),
        "actual_outline_centerline": True,
        "path_role": "FINAL_OUTLINE_FALLBACK",
        "outline_generation_source": str((path.metadata or {}).get("outline_generation_source", "detail_centerline_fallback")),
        "outline_fallback_mode": str((path.metadata or {}).get("outline_fallback_mode", "centerline_detail")),
        "outline_debug_label": str((path.metadata or {}).get("outline_debug_label", "centerline fallback")),
        "force_minimum_printable_stroke": True,
    }
    return Toolpath(
        points=[Point(float(point.x), float(point.y)) for point in path.points],
        kind="outline",
        closed=bool(path.closed),
        source=str(path.source or "coverage_outline_fallback"),
        region_id=path.region_id,
        metadata=metadata,
        path_id=path.path_id,
    )


def _vertex_angle_deg(prev_pt: tuple[float, float], pt: tuple[float, float], next_pt: tuple[float, float]) -> float:
    ax = float(prev_pt[0] - pt[0])
    ay = float(prev_pt[1] - pt[1])
    bx = float(next_pt[0] - pt[0])
    by = float(next_pt[1] - pt[1])
    la = math.hypot(ax, ay)
    lb = math.hypot(bx, by)
    if la <= 1e-9 or lb <= 1e-9:
        return 180.0
    cos_theta = max(-1.0, min(1.0, ((ax * bx) + (ay * by)) / max(1e-9, la * lb)))
    return math.degrees(math.acos(cos_theta))


def _boundary_fallback_paths_for_component(
    component_geometry: Any,
    *,
    component_id: int,
    existing_boundary_paths: list[Toolpath],
    simplify_tolerance_mm: float,
    line_width_mm: float,
) -> list[Toolpath]:
    if component_geometry is None or getattr(component_geometry, "is_empty", True):
        return []
    if not existing_boundary_paths:
        return []

    boundary_lines: list[Any] = []
    for path in existing_boundary_paths:
        if len(path.points) < 2:
            continue
        line = LineString([(point.x, point.y) for point in path.points])
        if not line.is_empty and line.length > 1e-9:
            boundary_lines.append(line)
    if not boundary_lines:
        return []
    emitted_boundary = unary_union(boundary_lines)
    max_allowed_distance_mm = max(line_width_mm * 0.6, (line_width_mm * 0.5) * 1.6, 0.08)
    simplify_mm = min(max(0.0, float(simplify_tolerance_mm)), 0.01)
    fallback_paths: list[Toolpath] = []

    def ring_needs_fallback(ring: Any) -> tuple[bool, bool]:
        coords = [(float(x), float(y)) for x, y in ring.coords]
        core = coords[:-1] if len(coords) >= 2 and coords[0] == coords[-1] else coords
        if len(core) < 2:
            return False, False
        acute_missing = False
        for index, point in enumerate(core):
            distance_mm = float(ShapelyPoint(point[0], point[1]).distance(emitted_boundary))
            if distance_mm <= max_allowed_distance_mm:
                continue
            prev_pt = core[index - 1]
            next_pt = core[(index + 1) % len(core)]
            acute_missing = _vertex_angle_deg(prev_pt, point, next_pt) <= 35.0
            return True, acute_missing
        return False, False

    def append_ring_fallback(ring: Any, *, ring_role: str, acute_missing: bool) -> None:
        points = pipeline_core.simplify_segment_points(
            [Point(float(x), float(y)) for x, y in ring.coords],
            simplify_mm,
            True,
        )
        if len(points) < 4:
            return
        metadata = {
            "boundary_kind": "fallback",
            "ring_role": ring_role,
            "component_id": int(component_id),
            "source_region_id": f"component_{int(component_id):03d}",
            "path_role": "FINAL_OUTLINE_FALLBACK",
            "generated_from": "source_component_boundary_fallback",
            "outline_generation_source": "boundary_fallback",
            "outline_fallback_mode": "acute_cusp_preserved" if acute_missing else "collapsed_offset_boundary",
            "outline_debug_label": "acute-cusp preserved" if acute_missing else "collapsed-offset fallback",
            "force_minimum_printable_stroke": True,
            "actual_outline_centerline": False,
            "offset_mm": 0.0,
            "outline_centerline_offset_mm": 0.0,
            "outline_offset_mm": 0.0,
            "simplify_tolerance_mm": float(simplify_mm),
        }
        if ring_role == "hole":
            metadata["is_hole"] = True
        fallback_paths.append(Toolpath(
            points=points,
            kind="outline",
            closed=True,
            source="coverage_boundary_fallback",
            region_id=int(component_id),
            metadata=metadata,
        ))

    for polygon in _geometry_parts(component_geometry):
        needs_fallback, acute_missing = ring_needs_fallback(polygon.exterior)
        if needs_fallback:
            append_ring_fallback(polygon.exterior, ring_role="outer", acute_missing=acute_missing)
        for ring in polygon.interiors:
            needs_fallback, acute_missing = ring_needs_fallback(ring)
            if needs_fallback:
                append_ring_fallback(ring, ring_role="hole", acute_missing=acute_missing)
    return fallback_paths


def _path_points_to_mask(
    paths: Iterable[Toolpath],
    *,
    shape: tuple[int, int],
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_px: int,
) -> np.ndarray:
    height, width = shape
    mask = np.zeros((height, width), dtype=np.uint8)
    for path in paths:
        if len(path.points) == 0:
            continue
        mapped = [pipeline_core.apply_svg_matrix(point, current_to_source_matrix) for point in path.points]
        if len(mapped) == 1:
            cv2.circle(mask, (int(round(mapped[0].x)), int(round(mapped[0].y))), pen_radius_px, 255, -1)
            continue
        for start, end in zip(mapped, mapped[1:]):
            cv2.line(
                mask,
                (int(round(start.x)), int(round(start.y))),
                (int(round(end.x)), int(round(end.y))),
                255,
                max(1, pen_radius_px * 2),
                lineType=cv2.LINE_AA,
            )
            cv2.circle(mask, (int(round(start.x)), int(round(start.y))), pen_radius_px, 255, -1)
            cv2.circle(mask, (int(round(end.x)), int(round(end.y))), pen_radius_px, 255, -1)
    return mask


def _paths_footprint_union(paths: Iterable[Toolpath], *, pen_radius_mm: float) -> Any:
    geometries: list[Any] = []
    for path in paths:
        if len(path.points) < 2:
            continue
        line = LineString([(point.x, point.y) for point in path.points])
        if line.is_empty or line.length <= 1e-9:
            continue
        geometries.append(line.buffer(max(0.01, pen_radius_mm), cap_style=1, join_style=1))
    if not geometries:
        return Polygon()
    return unary_union(geometries)


def _is_actual_outline_centerline_path(path: Toolpath) -> bool:
    metadata = path.metadata or {}
    if path.kind == "outline":
        return True
    if bool(metadata.get("actual_outline_centerline", False)):
        return True
    return str(metadata.get("path_role", "")) == "FINAL_OUTLINE_FALLBACK"


def _actual_outline_centerline_paths(paths: Iterable[Toolpath]) -> list[Toolpath]:
    return [path for path in paths if _is_actual_outline_centerline_path(path)]


def _infill_beyond_outline_area_mm2(
    paths: Iterable[Toolpath],
    *,
    allowed_geom: Any,
    pen_radius_mm: float,
) -> float:
    if allowed_geom is None or getattr(allowed_geom, "is_empty", True):
        return 0.0
    infill_footprint = _paths_footprint_union((path for path in paths if path.kind == "fill-infill"), pen_radius_mm=pen_radius_mm)
    if infill_footprint.is_empty:
        return 0.0
    return float(infill_footprint.difference(allowed_geom).area)


def _mask_to_overlay(mask: np.ndarray, color: tuple[int, int, int]) -> np.ndarray:
    image = np.full((mask.shape[0], mask.shape[1], 3), 255, dtype=np.uint8)
    image[mask > 0] = np.array(color, dtype=np.uint8)
    return image


def _render_paths_overlay(
    shape: tuple[int, int],
    paths: Iterable[Toolpath],
    *,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    colors: dict[str, tuple[int, int, int]],
) -> np.ndarray:
    canvas = np.full((shape[0], shape[1], 3), 255, dtype=np.uint8)
    for path in paths:
        color = colors.get(path.kind, (0, 0, 0))
        if len(path.points) == 1:
            mapped = pipeline_core.apply_svg_matrix(path.points[0], current_to_source_matrix)
            cv2.circle(canvas, (int(round(mapped.x)), int(round(mapped.y))), 2, color, -1)
            continue
        mapped = [pipeline_core.apply_svg_matrix(point, current_to_source_matrix) for point in path.points]
        coords = np.asarray([(int(round(point.x)), int(round(point.y))) for point in mapped], dtype=np.int32).reshape(-1, 1, 2)
        if len(coords) >= 2:
            cv2.polylines(canvas, [coords], bool(path.closed), color, 1, lineType=cv2.LINE_AA)
    return canvas


def _chain_pen_down_infill_connectors(paths: list[Toolpath], *, printable_geometry: Any | None = None) -> list[Toolpath]:
    if not paths:
        return paths
    del printable_geometry

    result: list[Toolpath] = []
    i = 0
    while i < len(paths):
        path = paths[i]
        if path.kind == "fill-infill" and len(path.points) >= 2:
            chain_points = list(path.points)
            chain_meta = dict(path.metadata or {})
            j = i + 1
            merged = False
            while j < len(paths):
                nxt = paths[j]
                if nxt.kind == "fill-infill-travel" and nxt.metadata.get("connector_pen_down_allowed") and len(nxt.points) >= 2:
                    if pipeline_core.nearly_same_point(chain_points[-1], nxt.points[0]):
                        chain_points = chain_points + nxt.points[1:]
                        merged = True
                        j += 1
                        continue
                    break
                break
            if merged:
                chain_meta["chained_infill"] = True
                result.append(Toolpath(points=chain_points, kind="fill-infill", closed=False, source="coverage_chained_fill", region_id=path.region_id, metadata=chain_meta))
                i = j
                continue
        result.append(path)
        i += 1
    return result


def _painted_metrics(
    *,
    target_mask: np.ndarray,
    painted_mask: np.ndarray,
    allowed_mask: np.ndarray,
    px_per_mm: float,
) -> dict[str, float]:
    target = target_mask > 0
    painted = painted_mask > 0
    allowed = allowed_mask > 0
    covered_inside = int(np.count_nonzero(target & painted))
    missed_inside = int(np.count_nonzero(target & ~painted))
    overflow = int(np.count_nonzero(painted & ~allowed))
    target_area_px = int(np.count_nonzero(target))
    painted_inside_mm2 = float(covered_inside) / max(1e-9, px_per_mm * px_per_mm)
    missed_area_mm2 = float(missed_inside) / max(1e-9, px_per_mm * px_per_mm)
    overflow_area_mm2 = float(overflow) / max(1e-9, px_per_mm * px_per_mm)
    coverage_percent = (100.0 * covered_inside / target_area_px) if target_area_px > 0 else 100.0
    return {
        "target_area_mm2": float(target_area_px) / max(1e-9, px_per_mm * px_per_mm),
        "painted_inside_area_mm2": painted_inside_mm2,
        "missed_area_mm2": missed_area_mm2,
        "overflow_area_mm2": overflow_area_mm2,
        "coverage_percent": float(coverage_percent),
        "missed_inside_px": float(missed_inside),
        "overflow_px": float(overflow),
        "covered_inside_px": float(covered_inside),
    }


def _equivalent_diameter_mm(area_mm2: float) -> float:
    if area_mm2 <= 1e-9:
        return 0.0
    return float(2.0 * math.sqrt(max(0.0, area_mm2) / math.pi))


def _detail_coverage_paths(paths: Iterable[Toolpath]) -> list[Toolpath]:
    allowed_kinds = {
        "fill-infill",
        REPAIR_PATH_KIND,
        "detail-trace",
        "coverage_centerline",
        "coverage_offset_line",
        "coverage_rectilinear",
        "coverage_contour",
        "coverage_connector",
        "coverage_tiny_mark",
    }
    return [path for path in paths if path.kind in allowed_kinds and len(path.points) >= 2]


def _connected_mask_blob_rows(
    component_mask: np.ndarray,
    *,
    px_per_mm: float,
    min_blob_area_mm2: float,
) -> list[dict[str, Any]]:
    if component_mask is None or component_mask.size == 0 or not np.any(component_mask > 0):
        return []
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats((component_mask > 0).astype(np.uint8), connectivity=8)
    blob_rows: list[dict[str, Any]] = []
    px_to_mm2 = 1.0 / max(1e-9, px_per_mm * px_per_mm)
    for blob_id in range(1, int(component_count)):
        area_px = int(stats[blob_id, cv2.CC_STAT_AREA])
        area_mm2 = float(area_px) * px_to_mm2
        if area_mm2 < min_blob_area_mm2:
            continue
        blob_rows.append({
            "blob_id": int(blob_id),
            "mask": (labels == blob_id).astype(np.uint8),
            "area_px": int(area_px),
            "area_mm2": float(area_mm2),
            "left_px": int(stats[blob_id, cv2.CC_STAT_LEFT]),
            "top_px": int(stats[blob_id, cv2.CC_STAT_TOP]),
            "width_px": int(stats[blob_id, cv2.CC_STAT_WIDTH]),
            "height_px": int(stats[blob_id, cv2.CC_STAT_HEIGHT]),
        })
    blob_rows.sort(key=lambda row: float(row["area_mm2"]), reverse=True)
    return blob_rows


def _blob_summary(blob_rows: list[dict[str, Any]]) -> dict[str, float | int]:
    if not blob_rows:
        return {
            "missed_blob_count": 0,
            "largest_missed_blob_area_mm2": 0.0,
            "largest_missed_blob_equivalent_diameter_mm": 0.0,
        }
    largest_area_mm2 = float(max(float(row.get("area_mm2", 0.0)) for row in blob_rows))
    return {
        "missed_blob_count": int(len(blob_rows)),
        "largest_missed_blob_area_mm2": float(largest_area_mm2),
        "largest_missed_blob_equivalent_diameter_mm": float(_equivalent_diameter_mm(largest_area_mm2)),
    }


def _bbox_mm_from_blob_row(blob_row: dict[str, Any], *, px_per_mm: float, origin_x: float, origin_y: float) -> dict[str, float]:
    left_px = int(blob_row.get("left_px", 0))
    top_px = int(blob_row.get("top_px", 0))
    width_px = int(blob_row.get("width_px", 0))
    height_px = int(blob_row.get("height_px", 0))
    return {
        "min_x": float(origin_x + (left_px / max(px_per_mm, 1e-9))),
        "min_y": float(origin_y + (top_px / max(px_per_mm, 1e-9))),
        "max_x": float(origin_x + ((left_px + width_px) / max(px_per_mm, 1e-9))),
        "max_y": float(origin_y + ((top_px + height_px) / max(px_per_mm, 1e-9))),
    }


def _centroid_from_mask(mask: np.ndarray, *, px_per_mm: float, origin_x: float, origin_y: float) -> tuple[dict[str, float], dict[str, float]]:
    ys, xs = np.nonzero(mask > 0)
    if xs.size == 0:
        return {"x": 0.0, "y": 0.0}, {"x": float(origin_x), "y": float(origin_y)}
    cx_px = float(np.mean(xs))
    cy_px = float(np.mean(ys))
    return (
        {"x": float(cx_px), "y": float(cy_px)},
        {"x": float(origin_x + (cx_px / max(px_per_mm, 1e-9))), "y": float(origin_y + (cy_px / max(px_per_mm, 1e-9)))},
    )


def _nearest_path_debug_row(paths: list[Toolpath], centroid_mm: dict[str, float]) -> dict[str, Any]:
    query = ShapelyPoint(float(centroid_mm["x"]), float(centroid_mm["y"]))
    nearest: dict[str, Any] = {
        "nearest_existing_path_id": "",
        "nearest_existing_path_kind": "",
        "nearest_existing_path_distance_mm": 0.0,
    }
    best_distance = float("inf")
    for path in paths:
        if len(path.points) < 2 or path.kind == "fill-infill-travel":
            continue
        line = LineString([(point.x, point.y) for point in path.points])
        if line.is_empty:
            continue
        distance = float(query.distance(line))
        if distance < best_distance:
            best_distance = distance
            nearest = {
                "nearest_existing_path_id": str(path.path_id or ""),
                "nearest_existing_path_kind": str(path.kind),
                "nearest_existing_path_distance_mm": float(distance),
            }
    return nearest


def _render_labeled_blob_overlay(mask: np.ndarray, blob_rows: list[dict[str, Any]]) -> np.ndarray:
    canvas = cv2.cvtColor((mask > 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    for index, blob_row in enumerate(blob_rows, start=1):
        left = int(blob_row.get("left_px", 0))
        top = int(blob_row.get("top_px", 0))
        width = int(blob_row.get("width_px", 0))
        height = int(blob_row.get("height_px", 0))
        cv2.rectangle(canvas, (left, top), (left + width, top + height), (0, 0, 255), 1)
        cv2.putText(canvas, str(index), (left, max(10, top + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1, cv2.LINE_AA)
    return canvas


def _longest_mask_centerline_candidate(
    component_mask: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    component_id: int,
) -> Toolpath | None:
    skeleton = _skeletonize_mask(component_mask)
    segments = _skeleton_to_segments(skeleton)
    if not segments:
        return None
    longest = max(segments, key=lambda item: len(item))
    points_px = [(float(x), float(y)) for y, x in longest]
    return _pixel_path_to_toolpath(
        points_px,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        kind="fill-infill",
        source="detail_repair_fill",
        component_id=component_id,
        metadata={
            "repair_candidate": True,
            "repair_candidate_type": "centerline_fallback",
        },
        closed=False,
    )


def _convert_candidate_to_detail_repair_fill(
    candidate: Toolpath,
    *,
    component_id: int,
    region_component_id: int,
    strategy_key: str,
    strategy_label: str,
    region_area_mm2: float,
    region_max_width_mm: float,
    region_equivalent_diameter_mm: float,
) -> Toolpath:
    metadata = {
        **(candidate.metadata or {}),
        "repair_candidate": True,
        "fill_mode": f"detail_repair_{strategy_key}",
        "fill_strategy": strategy_label,
        "source_region_id": f"detail_region_{int(region_component_id):03d}",
        "detail_region_component_id": int(region_component_id),
        "detail_repair_component_id": int(component_id),
        "detail_region_area_mm2": float(region_area_mm2),
        "detail_region_max_width_mm": float(region_max_width_mm),
        "detail_region_equivalent_diameter_mm": float(region_equivalent_diameter_mm),
        "detail_repair_strategy": strategy_key,
    }
    return Toolpath(
        points=list(candidate.points),
        kind=REPAIR_PATH_KIND,
        closed=bool(candidate.closed),
        source=REPAIR_PATH_SOURCE,
        region_id=region_component_id,
        metadata=metadata,
    )


def _detail_repair_candidates_for_blob(
    blob_mask: np.ndarray,
    *,
    blob_geometry: Any | None,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    line_width_mm: float,
    simplify_tolerance_mm: float,
    detail_fill_spacing_mm: float,
    region_component_id: int,
    region_area_mm2: float,
    region_max_width_mm: float,
    region_equivalent_diameter_mm: float,
    main_angle_deg: float,
) -> list[tuple[str, Toolpath]]:
    erosion_px = max(1, int(round(px_per_mm * max(0.08, line_width_mm * 0.22))))
    safe_mask = cv2.erode(
        (blob_mask > 0).astype(np.uint8),
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erosion_px * 2 + 1, erosion_px * 2 + 1)),
        iterations=1,
    )
    working_mask = safe_mask if np.any(safe_mask > 0) else (blob_mask > 0).astype(np.uint8)
    working_geometry = blob_geometry
    if np.any(safe_mask > 0):
        safe_geometry = _component_mask_to_geometry(
            (safe_mask > 0).astype(np.uint8) * 255,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
        )
        if safe_geometry is not None and not getattr(safe_geometry, "is_empty", True):
            working_geometry = safe_geometry

    ys, xs = np.nonzero(working_mask > 0)
    if xs.size == 0:
        return []

    ordered_candidates: list[tuple[str, Toolpath]] = []
    raw_candidates = _candidate_paths_for_missed_component(
        working_mask,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        line_width_mm=line_width_mm,
        component_id=region_component_id,
        main_angle_deg=main_angle_deg,
        component_geometry=working_geometry,
    )
    preferred_parallel_angle = _normalize_angle(main_angle_deg)
    preferred_perpendicular_angle = _normalize_angle(main_angle_deg + 90.0)

    if working_geometry is not None and not getattr(working_geometry, "is_empty", True):
        serpentine, _stats = _scanline_fill_paths(
            working_geometry,
            angle_deg=preferred_parallel_angle,
            spacing_mm=detail_fill_spacing_mm,
            line_width_mm=line_width_mm,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            component_id=region_component_id,
            allow_connectors=False,
            max_overflow_mm=0.05,
            fill_mode_label="detail_repair_serpentine",
        )
        for candidate in serpentine[:4]:
            ordered_candidates.append((
                "serpentine",
                _convert_candidate_to_detail_repair_fill(
                    candidate,
                    component_id=region_component_id,
                    region_component_id=region_component_id,
                    strategy_key="serpentine",
                    strategy_label="DETAIL_REPAIR_SERPENTINE",
                    region_area_mm2=region_area_mm2,
                    region_max_width_mm=region_max_width_mm,
                    region_equivalent_diameter_mm=region_equivalent_diameter_mm,
                ),
            ))

        contour_paths = _boundary_paths_for_geometry(
            working_geometry,
            simplify_tolerance_mm=max(0.0, simplify_tolerance_mm),
            line_width_mm=line_width_mm,
            centerline_offset_mm=line_width_mm * 0.5,
        )
        for candidate in contour_paths[:2]:
            ordered_candidates.append((
                "contour_offset",
                _convert_candidate_to_detail_repair_fill(
                    candidate,
                    component_id=region_component_id,
                    region_component_id=region_component_id,
                    strategy_key="contour_offset",
                    strategy_label="DETAIL_REPAIR_CONTOUR_OFFSET",
                    region_area_mm2=region_area_mm2,
                    region_max_width_mm=region_max_width_mm,
                    region_equivalent_diameter_mm=region_equivalent_diameter_mm,
                ),
            ))

    for candidate in raw_candidates:
        angle_deg = float((candidate.metadata or {}).get("repair_candidate_angle_deg", preferred_parallel_angle))
        normalized_angle = _normalize_angle(angle_deg)
        if abs(normalized_angle - preferred_parallel_angle) < 1e-6:
            ordered_candidates.append((
                "parallel_stroke",
                _convert_candidate_to_detail_repair_fill(
                    candidate,
                    component_id=region_component_id,
                    region_component_id=region_component_id,
                    strategy_key="parallel_stroke",
                    strategy_label="DETAIL_REPAIR_PARALLEL_STROKE",
                    region_area_mm2=region_area_mm2,
                    region_max_width_mm=region_max_width_mm,
                    region_equivalent_diameter_mm=region_equivalent_diameter_mm,
                ),
            ))
        elif abs(normalized_angle - preferred_perpendicular_angle) < 1e-6:
            ordered_candidates.append((
                "perpendicular_stroke",
                _convert_candidate_to_detail_repair_fill(
                    candidate,
                    component_id=region_component_id,
                    region_component_id=region_component_id,
                    strategy_key="perpendicular_stroke",
                    strategy_label="DETAIL_REPAIR_PERPENDICULAR_STROKE",
                    region_area_mm2=region_area_mm2,
                    region_max_width_mm=region_max_width_mm,
                    region_equivalent_diameter_mm=region_equivalent_diameter_mm,
                ),
            ))

    centerline = _longest_mask_centerline_candidate(
        working_mask,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        component_id=region_component_id,
    )
    if centerline is not None:
        ordered_candidates.append((
            "centerline_fallback",
            _convert_candidate_to_detail_repair_fill(
                centerline,
                component_id=region_component_id,
                region_component_id=region_component_id,
                strategy_key="centerline_fallback",
                strategy_label="DETAIL_REPAIR_CENTERLINE_FALLBACK",
                region_area_mm2=region_area_mm2,
                region_max_width_mm=region_max_width_mm,
                region_equivalent_diameter_mm=region_equivalent_diameter_mm,
            ),
        ))
    return ordered_candidates


def _line_from_candidate(
    component_mask: np.ndarray,
    *,
    angle_deg: float,
    origin_px: tuple[float, float],
    extend_px: float,
    min_inside_px: float,
) -> list[tuple[float, float]] | None:
    height, width = component_mask.shape[:2]
    origin_x, origin_y = origin_px
    angle_rad = math.radians(angle_deg)
    ux = math.cos(angle_rad)
    uy = math.sin(angle_rad)
    px = -uy
    py = ux
    x0 = origin_x - ux * extend_px
    y0 = origin_y - uy * extend_px
    x1 = origin_x + ux * extend_px
    y1 = origin_y + uy * extend_px
    steps = max(12, int(round(extend_px * 3.0)))
    samples: list[tuple[float, float, bool]] = []
    for index in range(steps + 1):
        t = index / max(1, steps)
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        xi = int(round(x))
        yi = int(round(y))
        inside = 0 <= xi < width and 0 <= yi < height and bool(component_mask[yi, xi])
        samples.append((x, y, inside))
    best_start = -1
    best_end = -1
    run_start = -1
    for index, (_x, _y, inside) in enumerate(samples):
        if inside and run_start < 0:
            run_start = index
        if (not inside or index == len(samples) - 1) and run_start >= 0:
            run_end = index if inside and index == len(samples) - 1 else index - 1
            if run_end - run_start > best_end - best_start:
                best_start, best_end = run_start, run_end
            run_start = -1
    if best_start < 0 or best_end <= best_start:
        return None
    a = samples[best_start]
    b = samples[best_end]
    if math.hypot(b[0] - a[0], b[1] - a[1]) < min_inside_px:
        return None
    return [(a[0], a[1]), (b[0], b[1])]


def _candidate_paths_for_missed_component(
    component_mask: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    line_width_mm: float,
    component_id: int,
    main_angle_deg: float,
    component_geometry: Any | None,
) -> list[Toolpath]:
    ys, xs = np.nonzero(component_mask > 0)
    if xs.size == 0:
        return []
    width_px = float(xs.max() - xs.min() + 1)
    height_px = float(ys.max() - ys.min() + 1)
    centroid_x = float(np.mean(xs))
    centroid_y = float(np.mean(ys))
    diagonal_px = max(6.0, math.hypot(width_px, height_px) * 1.25)
    min_inside_px = max(2.0, px_per_mm * line_width_mm * 0.2)
    angles = [
        main_angle_deg,
        _normalize_angle(main_angle_deg + 90.0),
        _normalize_angle(main_angle_deg + 45.0),
        _normalize_angle(main_angle_deg - 45.0),
    ]
    if component_geometry is not None and not getattr(component_geometry, "is_empty", True):
        boundary_angle = _boundary_tangent_angle(component_geometry)
        angles.extend([boundary_angle, _normalize_angle(boundary_angle + 90.0)])
    candidates: list[Toolpath] = []
    for angle in angles:
        segment = _line_from_candidate(
            component_mask,
            angle_deg=angle,
            origin_px=(centroid_x, centroid_y),
            extend_px=diagonal_px,
            min_inside_px=min_inside_px,
        )
        if segment is None:
            continue
        candidate = _pixel_path_to_toolpath(
            segment,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            kind="coverage_centerline",
            source="coverage_repair_candidate",
            component_id=component_id,
            metadata={
                "repair_candidate": True,
                "repair_candidate_angle_deg": float(angle),
                "repair_candidate_type": "short_straight",
                "component_id": int(component_id),
            },
        )
        if candidate is not None:
            candidates.append(candidate)

    serpentine_spacing = max(0.22, line_width_mm * 0.55)
    if component_geometry is not None and not getattr(component_geometry, "is_empty", True):
        serpentine, _stats = _scanline_fill_paths(
            component_geometry,
            angle_deg=_normalize_angle(main_angle_deg),
            spacing_mm=serpentine_spacing,
            line_width_mm=line_width_mm,
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            component_id=component_id,
            allow_connectors=False,
            max_overflow_mm=0.15,
        )
        for candidate in serpentine[:3]:
            candidate.metadata = {**candidate.metadata, "repair_candidate": True, "repair_candidate_type": "local_serpentine"}
        candidates.extend(serpentine[:3])

    boundary = _boundary_paths_for_component(
        component_geometry,
        component_id=component_id,
        simplify_tolerance_mm=0.03,
        line_width_mm=line_width_mm,
        centerline_offset_mm=line_width_mm * 0.5,
    )
    for candidate in boundary[:2]:
        candidate.metadata = {**candidate.metadata, "repair_candidate": True, "repair_candidate_type": "boundary_stroke"}
    candidates.extend(boundary[:2])
    return candidates


def _boundary_tangent_angle(component_geometry: Any) -> float:
    parts = _geometry_parts(component_geometry)
    if not parts:
        return 0.0
    polygon = max(parts, key=lambda item: float(item.area))
    coords = list(polygon.exterior.coords)
    if len(coords) < 3:
        return 0.0
    p0 = coords[0]
    p1 = coords[1]
    return _normalize_angle(math.degrees(math.atan2(float(p1[1] - p0[1]), float(p1[0] - p0[0]))))


def _score_candidate(
    *,
    current_paths: list[Toolpath],
    candidate: Toolpath,
    target_mask: np.ndarray,
    allowed_mask: np.ndarray,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_mm: float,
    sample_step_mm: float,
) -> tuple[float, dict[str, float]]:
    px_per_mm = max(abs(float(current_to_source_matrix[0])), abs(float(current_to_source_matrix[3])), 1e-6)
    pen_radius_px = max(1, int(round(pen_radius_mm * px_per_mm)))
    trial = list(current_paths) + [candidate]
    trial_metrics = pipeline_core.compute_toolpath_mask_coverage_metrics(
        trial,
        mask=target_mask,
        current_to_source_matrix=current_to_source_matrix,
        pen_radius_mm=pen_radius_mm,
        sample_step_mm=sample_step_mm,
        include_kinds={
            REPAIR_PATH_KIND,
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_contour",
            "coverage_connector",
            "coverage_tiny_mark",
            "fill-infill-travel",
        },
    )
    if trial_metrics is None:
        return -1e12, {"covered_missed_px": 0.0, "overflow_px": 0.0, "added_length_mm": 0.0, "pen_lift_penalty": 1.0}
    current_metrics = pipeline_core.compute_toolpath_mask_coverage_metrics(
        current_paths,
        mask=target_mask,
        current_to_source_matrix=current_to_source_matrix,
        pen_radius_mm=pen_radius_mm,
        sample_step_mm=sample_step_mm,
        include_kinds={
            REPAIR_PATH_KIND,
            "coverage_centerline",
            "coverage_offset_line",
            "coverage_rectilinear",
            "coverage_contour",
            "coverage_connector",
            "coverage_tiny_mark",
            "fill-infill-travel",
        },
    )
    if current_metrics is None:
        return -1e12, {"covered_missed_px": 0.0, "overflow_px": 0.0, "added_length_mm": 0.0, "pen_lift_penalty": 1.0}
    current_painted = _path_points_to_mask(current_paths, shape=target_mask.shape, current_to_source_matrix=current_to_source_matrix, pen_radius_px=pen_radius_px)
    trial_painted = _path_points_to_mask(trial, shape=target_mask.shape, current_to_source_matrix=current_to_source_matrix, pen_radius_px=pen_radius_px)
    target = target_mask > 0
    allowed = allowed_mask > 0
    current_missed = target & ~(current_painted > 0)
    trial_missed = target & ~(trial_painted > 0)
    current_over = (current_painted > 0) & ~allowed
    trial_over = (trial_painted > 0) & ~allowed
    covered_missed = int(np.count_nonzero(current_missed)) - int(np.count_nonzero(trial_missed))
    overflow_px = int(np.count_nonzero(trial_over)) - int(np.count_nonzero(current_over))
    added_length = max(0.0, pipeline_core.segment_length(candidate.points))
    pen_lift_penalty = 0.0 if current_paths and pipeline_core.nearly_same_point(current_paths[-1].points[-1], candidate.points[0]) else 1.0
    score = (covered_missed * 1000.0) - (overflow_px * 20.0) - added_length - (pen_lift_penalty * 500.0)
    return score, {
        "covered_missed_px": float(max(0, covered_missed)),
        "overflow_px": float(max(0, overflow_px)),
        "added_length_mm": float(added_length),
        "pen_lift_penalty": float(pen_lift_penalty),
        "coverage_percent": float(trial_metrics.raw_coverage_percent),
    }


def _path_centerline_mask(
    path: Toolpath,
    *,
    shape: tuple[int, int],
    current_to_source_matrix: tuple[float, float, float, float, float, float],
) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    if len(path.points) < 2:
        return mask
    mapped = [pipeline_core.apply_svg_matrix(point, current_to_source_matrix) for point in path.points]
    for start, end in zip(mapped, mapped[1:]):
        cv2.line(
            mask,
            (int(round(start.x)), int(round(start.y))),
            (int(round(end.x)), int(round(end.y))),
            255,
            1,
            lineType=cv2.LINE_AA,
        )
    return mask


def _candidate_centeredness_against_geometry(path: Toolpath, geometry: Any) -> float:
    if geometry is None or getattr(geometry, "is_empty", True) or len(path.points) < 2:
        return 0.0
    try:
        boundary = geometry.boundary
    except Exception:
        return 0.0
    if boundary is None or getattr(boundary, "is_empty", True):
        return 0.0
    samples = path.points
    if len(samples) > 24:
        step = max(1, int(round(len(samples) / 24)))
        samples = [samples[index] for index in range(0, len(samples), step)]
    distances: list[float] = []
    for pt in samples:
        try:
            distances.append(float(ShapelyPoint(float(pt.x), float(pt.y)).distance(boundary)))
        except Exception:
            continue
    if not distances:
        return 0.0
    return float(sum(distances) / max(1, len(distances)))


def _candidate_medial_quality(path: Toolpath, geometry: Any) -> float:
    if geometry is None or getattr(geometry, "is_empty", True) or len(path.points) < 2:
        return 0.0
    boundary = getattr(geometry, "boundary", None)
    if boundary is None or getattr(boundary, "is_empty", True):
        return 0.0
    samples = path.points
    if len(samples) > 24:
        step = max(1, int(round(len(samples) / 24)))
        samples = [samples[index] for index in range(0, len(samples), step)]
    distances: list[float] = []
    for pt in samples:
        try:
            distances.append(float(ShapelyPoint(float(pt.x), float(pt.y)).distance(boundary)))
        except Exception:
            continue
    if not distances:
        return 0.0
    mean_distance = float(sum(distances) / max(1, len(distances)))
    variance = float(sum((distance - mean_distance) ** 2 for distance in distances) / max(1, len(distances)))
    return max(0.0, mean_distance - math.sqrt(max(0.0, variance)))


def _distance_transform_max_radius_mm(mask: np.ndarray, *, px_per_mm: float) -> float:
    binary = (mask > 0).astype(np.uint8)
    if not np.any(binary):
        return 0.0
    distance = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    return float(np.max(distance)) / max(px_per_mm, 1e-9)


def _safe_intersection(a: Any, b: Any) -> Any:
    if a is None or b is None:
        return Polygon()
    if getattr(a, "is_empty", True) or getattr(b, "is_empty", True):
        return Polygon()
    try:
        return a.intersection(b)
    except Exception:
        return a.buffer(0).intersection(b.buffer(0))


def _safe_buffer(geometry: Any, distance_mm: float) -> Any:
    if geometry is None or getattr(geometry, "is_empty", True):
        return Polygon()
    try:
        return geometry.buffer(float(distance_mm), join_style=1)
    except Exception:
        return geometry.buffer(0).buffer(float(distance_mm), join_style=1)


def _classify_missed_component(
    printable_geometry: Any,
    safe_geometry: Any,
    missed_component: Any,
    *,
    missed_mask: np.ndarray,
    px_per_mm: float,
    pen_radius_mm: float,
) -> tuple[str, dict[str, Any]]:
    local_region = _safe_intersection(printable_geometry, _safe_buffer(missed_component, 2.0 * pen_radius_mm))
    local_safe = _strict_safe_centerline_geometry(local_region, inset_mm=pen_radius_mm)
    center_domain = _safe_intersection(safe_geometry, _safe_buffer(missed_component, pen_radius_mm))
    if local_region is not None and not getattr(local_region, "is_empty", True):
        local_mask, _origin_x, _origin_y, local_px_per_mm = _rasterize_geometry(
            local_region,
            resolution_mm=max(0.03, pen_radius_mm * 0.12),
            pad_mm=max(0.6, pen_radius_mm * 2.0),
        )
        max_radius_mm = _distance_transform_max_radius_mm(local_mask, px_per_mm=local_px_per_mm)
    else:
        max_radius_mm = _distance_transform_max_radius_mm(missed_mask, px_per_mm=px_per_mm)
    local_safe_area_mm2 = 0.0 if local_safe is None or getattr(local_safe, "is_empty", True) else float(local_safe.area)
    center_domain_area_mm2 = 0.0 if center_domain is None or getattr(center_domain, "is_empty", True) else float(center_domain.area)

    if (
        center_domain is not None
        and not getattr(center_domain, "is_empty", True)
        and center_domain_area_mm2 > MIN_CENTER_DOMAIN_AREA_MM2
        and max_radius_mm >= pen_radius_mm
    ):
        return "safe_repair", {
            "local_region": local_region,
            "local_safe": local_safe,
            "center_domain": center_domain,
            "max_radius_mm": float(max_radius_mm),
        }

    if (
        local_safe is None
        or getattr(local_safe, "is_empty", True)
        or local_safe_area_mm2 < MIN_SAFE_AREA_MM2
        or max_radius_mm < (pen_radius_mm + THIN_TOL_MM)
    ):
        return "thin_centerline", {
            "local_region": local_region,
            "local_safe": local_safe,
            "center_domain": center_domain,
            "max_radius_mm": float(max_radius_mm),
        }

    return "mixed", {
        "local_region": local_region,
        "local_safe": local_safe,
        "center_domain": center_domain,
        "max_radius_mm": float(max_radius_mm),
    }


def _is_candidate_overflow_symmetric(
    candidate_mask: np.ndarray,
    *,
    allowed_mask: np.ndarray,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    candidate: Toolpath,
) -> bool:
    overflow_mask = ((candidate_mask > 0) & ~(allowed_mask > 0)).astype(np.uint8)
    if not np.any(overflow_mask > 0) or len(candidate.points) < 2:
        return True
    ys, xs = np.nonzero(overflow_mask > 0)
    if xs.size <= 1:
        return False
    mapped = [pipeline_core.apply_svg_matrix(point, current_to_source_matrix) for point in candidate.points]
    start = mapped[0]
    end = mapped[-1]
    dx = float(end.x - start.x)
    dy = float(end.y - start.y)
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return False
    nx = -dy / length
    ny = dx / length
    midpoint_x = (float(start.x) + float(end.x)) * 0.5
    midpoint_y = (float(start.y) + float(end.y)) * 0.5
    signed = [((float(x) - midpoint_x) * nx) + ((float(y) - midpoint_y) * ny) for y, x in zip(ys, xs)]
    if not signed:
        return False
    has_pos = any(value > 0.5 for value in signed)
    has_neg = any(value < -0.5 for value in signed)
    if not (has_pos and has_neg):
        return False
    return abs(float(np.mean(signed))) <= max(1.0, float(np.std(signed)) * 0.35)


def _repair_candidate_audit(
    candidate: Toolpath,
    *,
    current_painted: np.ndarray,
    target_mask: np.ndarray,
    allowed_mask: np.ndarray,
    safe_centerline_mask: np.ndarray,
    boundary_band_mask: np.ndarray,
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    pen_radius_px: int,
    component_geometry: Any | None,
    source: str,
    repair_mode: str,
    component_id: int | None,
) -> dict[str, Any]:
    candidate_mask = _path_points_to_mask(
        [candidate],
        shape=target_mask.shape,
        current_to_source_matrix=current_to_source_matrix,
        pen_radius_px=pen_radius_px,
    )
    candidate_bool = candidate_mask > 0
    current_bool = current_painted > 0
    target_bool = target_mask > 0
    allowed_bool = allowed_mask > 0
    safe_centerline_bool = safe_centerline_mask > 0
    boundary_bool = boundary_band_mask > 0
    centerline_mask = _path_centerline_mask(
        candidate,
        shape=target_mask.shape,
        current_to_source_matrix=current_to_source_matrix,
    )
    centerline_bool = centerline_mask > 0
    new_expected_pixels = int(np.count_nonzero(candidate_bool & target_bool & ~current_bool))
    redundant_pixels = int(np.count_nonzero(candidate_bool & target_bool & current_bool))
    overflow_pixels = int(np.count_nonzero(candidate_bool & ~allowed_bool))
    boundary_overflow_pixels = int(np.count_nonzero(candidate_bool & ~allowed_bool & boundary_bool))
    centerline_violation_pixels = int(np.count_nonzero(centerline_bool & ~safe_centerline_bool))
    centeredness_mm = float(_candidate_centeredness_against_geometry(candidate, component_geometry))
    path_length_mm = float(pipeline_core.segment_length(candidate.points))
    is_symmetric_overflow = _is_candidate_overflow_symmetric(
        candidate_mask,
        allowed_mask=allowed_mask,
        current_to_source_matrix=current_to_source_matrix,
        candidate=candidate,
    )
    boundary_hug_penalty = float(boundary_overflow_pixels + max(0, centerline_violation_pixels))
    new_expected_area = float(new_expected_pixels)
    redundant_area = float(redundant_pixels)
    overflow_area = float(overflow_pixels)
    overflow_weight = 3.0 if repair_mode != "normal-safe-repair" and is_symmetric_overflow else 8.0
    score = (
        (4.0 * new_expected_area)
        - (overflow_weight * overflow_area)
        - (1.5 * redundant_area)
        - (0.15 * path_length_mm)
        - (5.0 * boundary_hug_penalty)
    )
    classification = "reject-useless-or-overflowing"
    if repair_mode == "normal-safe-repair":
        if (
            new_expected_pixels > 0
            and overflow_pixels == 0
            and boundary_overflow_pixels == 0
            and centerline_violation_pixels == 0
        ):
            classification = "normal-safe-repair"
    else:
        if (
            new_expected_pixels > 0
            and centeredness_mm >= 0.06
            and boundary_overflow_pixels <= max(4, int(round(new_expected_pixels * 0.10)))
            and centerline_violation_pixels <= max(6, int(round(new_expected_pixels * 0.20)))
            and (overflow_pixels == 0 or is_symmetric_overflow)
        ):
            classification = "thin-collapsed-detail-repair"
    return {
        "candidate": candidate,
        "source": str(source),
        "repair_mode": str(repair_mode),
        "component_id": int(component_id) if component_id is not None else None,
        "new_expected_pixels": int(new_expected_pixels),
        "redundant_pixels": int(redundant_pixels),
        "overflow_pixels": int(overflow_pixels),
        "boundary_overflow_pixels": int(boundary_overflow_pixels),
        "centerline_violation_pixels": int(centerline_violation_pixels),
        "centeredness_mm": float(centeredness_mm),
        "path_length_mm": float(path_length_mm),
        "boundary_hug_penalty": float(boundary_hug_penalty),
        "is_symmetric_overflow": bool(is_symmetric_overflow),
        "score": float(score),
        "classification": classification,
    }


def _medial_repair_candidates_for_component(
    component_mask: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    component_id: int,
) -> list[Toolpath]:
    skeleton = _skeletonize_mask(component_mask)
    segments = sorted(_skeleton_to_segments(skeleton), key=len, reverse=True)
    candidates: list[Toolpath] = []
    for segment in segments[:8]:
        candidate = _pixel_path_to_toolpath(
            [(float(x), float(y)) for y, x in segment],
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            kind="coverage_centerline",
            source="coverage_repair_candidate",
            component_id=component_id,
            metadata={
                "repair_candidate": True,
                "repair_candidate_type": "medial_skeleton",
                "centerline_generation_mode": "medial-skeleton",
                "generated_from_safe_mask": False,
                "safe_centerline_inset_mm": 0.0,
            },
            closed=False,
        )
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def _medial_repair_candidates_for_region_blob(
    component_mask: np.ndarray,
    focus_blob_mask: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    component_id: int,
) -> list[Toolpath]:
    if component_mask is None or component_mask.size == 0 or not np.any(component_mask > 0):
        return []
    if focus_blob_mask is None or focus_blob_mask.size == 0 or not np.any(focus_blob_mask > 0):
        return []
    skeleton = _skeletonize_mask(component_mask)
    segments = sorted(_skeleton_to_segments(skeleton), key=len, reverse=True)
    focus_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    expanded_focus = cv2.dilate((focus_blob_mask > 0).astype(np.uint8), focus_kernel, iterations=1) > 0
    candidates: list[tuple[int, int, Toolpath]] = []
    for segment in segments:
        overlap = 0
        for y, x in segment:
            if 0 <= y < expanded_focus.shape[0] and 0 <= x < expanded_focus.shape[1] and expanded_focus[y, x]:
                overlap += 1
        if overlap <= 0:
            continue
        candidate = _pixel_path_to_toolpath(
            [(float(x), float(y)) for y, x in segment],
            origin_x=origin_x,
            origin_y=origin_y,
            px_per_mm=px_per_mm,
            kind="coverage_centerline",
            source="coverage_repair_candidate",
            component_id=component_id,
            metadata={
                "repair_candidate": True,
                "repair_candidate_type": "medial_region_skeleton",
                "centerline_generation_mode": "medial-region-skeleton",
                "generated_from_safe_mask": False,
                "safe_centerline_inset_mm": 0.0,
            },
            closed=False,
        )
        if candidate is not None:
            candidates.append((int(overlap), int(len(segment)), candidate))
    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    return [candidate for _overlap, _length, candidate in candidates[:8]]


def _optimized_final_repair_paths(
    *,
    all_paths: list[Toolpath],
    target_mask: np.ndarray,
    allowed_mask: np.ndarray,
    printable_geometry: Any,
    current_to_source: tuple[float, float, float, float, float, float],
    origin_x: float,
    origin_y: float,
    px_per_mm: float,
    line_width_mm: float,
    simplify_tolerance_mm: float,
) -> tuple[list[Toolpath], list[Toolpath], list[dict[str, Any]]]:
    base_paths = [path for path in all_paths if not _is_repair_path(path)]
    existing_repairs = [path for path in all_paths if _is_repair_path(path)]
    pen_radius_mm = float(line_width_mm) * 0.5
    pen_radius_px = max(1, int(round(pen_radius_mm * px_per_mm)))
    safe_centerline_mask = cv2.erode(
        target_mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (pen_radius_px * 2 + 1, pen_radius_px * 2 + 1)),
        iterations=1,
    )
    boundary_band_px = max(1, pen_radius_px + 1)
    boundary_band_mask = cv2.morphologyEx(
        target_mask,
        cv2.MORPH_GRADIENT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (boundary_band_px * 2 + 1, boundary_band_px * 2 + 1)),
    )
    base_painted = _path_points_to_mask(
        base_paths,
        shape=target_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_px=pen_radius_px,
    )
    audit_rows: list[dict[str, Any]] = []
    for repair_path in existing_repairs:
        audit_rows.append(_repair_candidate_audit(
            repair_path,
            current_painted=base_painted,
            target_mask=target_mask,
            allowed_mask=allowed_mask,
            safe_centerline_mask=safe_centerline_mask,
            boundary_band_mask=boundary_band_mask,
            current_to_source_matrix=current_to_source,
            pen_radius_px=pen_radius_px,
            component_geometry=None,
            source=str(repair_path.source or ""),
            repair_mode=str((repair_path.metadata or {}).get("repair_mode", "unknown")),
            component_id=(repair_path.metadata or {}).get("component_id"),
        ))

    rebuilt_repairs: list[Toolpath] = []
    current_painted = np.asarray(base_painted).copy()
    target_geometry = printable_geometry if printable_geometry is not None else _component_mask_to_geometry(
        target_mask,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
    )
    safe_printable_geometry = _strict_safe_centerline_geometry(target_geometry, inset_mm=pen_radius_mm)
    candidate_rows: list[dict[str, Any]] = []
    for _repair_iteration in range(REPAIR_MAX_ITERATIONS):
        residual_mask = ((target_mask > 0) & ~(current_painted > 0)).astype(np.uint8)
        if not np.any(residual_mask):
            break
        current_metrics = _painted_metrics(
            target_mask=target_mask,
            painted_mask=current_painted,
            allowed_mask=allowed_mask,
            px_per_mm=px_per_mm,
        )
        coverage_ratio = float(current_metrics["coverage_percent"]) / 100.0
        overflow_ratio = (
            float(current_metrics["overflow_px"]) / max(1.0, float(np.count_nonzero(target_mask > 0)))
        )
        if coverage_ratio >= REPAIR_TARGET_COVERAGE_RATIO and overflow_ratio <= REPAIR_MAX_OVERFLOW_RATIO:
            break

        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(residual_mask, connectivity=8)
        missed_ids = [index for index in range(1, int(component_count)) if int(stats[index, cv2.CC_STAT_AREA]) > 0]
        missed_ids.sort(key=lambda idx: int(stats[idx, cv2.CC_STAT_AREA]), reverse=True)
        accepted_this_iteration = False

        for missed_id in missed_ids[:80]:
            component_mask = (labels == missed_id).astype(np.uint8)
            if not np.any(component_mask > 0):
                continue
            component_geometry = _component_mask_to_geometry(
                component_mask * 255,
                origin_x=origin_x,
                origin_y=origin_y,
                px_per_mm=px_per_mm,
            )
            if component_geometry is None or getattr(component_geometry, "is_empty", True):
                continue
            classification, class_info = _classify_missed_component(
                target_geometry,
                safe_printable_geometry,
                component_geometry,
                missed_mask=component_mask,
                px_per_mm=px_per_mm,
                pen_radius_mm=pen_radius_mm,
            )
            local_region = class_info["local_region"]
            center_domain = class_info["center_domain"]
            component_candidates: list[tuple[str, str, Toolpath, Any | None]] = []
            angle_deg = _component_fill_angle(component_mask)

            if classification in {"safe_repair", "mixed"} and center_domain is not None and not getattr(center_domain, "is_empty", True):
                safe_paths, _stats = _scanline_fill_paths(
                    center_domain,
                    angle_deg=angle_deg,
                    spacing_mm=max(0.25, line_width_mm),
                    line_width_mm=line_width_mm,
                    origin_x=origin_x,
                    origin_y=origin_y,
                    px_per_mm=px_per_mm,
                    component_id=missed_id,
                    allow_connectors=False,
                    max_overflow_mm=0.0,
                    fill_mode_label="repair_safe_fill",
                    intersection_geometry=center_domain,
                    endpoint_limit_geometry=center_domain,
                    centerline_generation_mode="safe-mask",
                    safe_centerline_inset_mm=pen_radius_mm,
                )
                for index, candidate in enumerate(safe_paths[:8]):
                    candidate.metadata = {
                        **dict(candidate.metadata or {}),
                        "repair_mode": "normal-safe-repair",
                        "component_id": int(missed_id),
                        "generated_from_safe_mask": True,
                        "safe_centerline_inset_mm": float(pen_radius_mm),
                        "missed_component_classification": classification,
                    }
                    component_candidates.append(("normal-safe-repair", f"safe_scanline_{index}", candidate, center_domain))

            if classification in {"thin_centerline", "mixed"} and local_region is not None and not getattr(local_region, "is_empty", True):
                local_mask, local_origin_x, local_origin_y, local_px_per_mm = _rasterize_geometry(
                    local_region,
                    resolution_mm=max(0.03, line_width_mm * 0.08),
                    pad_mm=max(line_width_mm, 0.6),
                )
                if np.any(local_mask > 0):
                    focus_geometry = _safe_intersection(local_region, _safe_buffer(component_geometry, pen_radius_mm))
                    focus_mask, _fx, _fy, _fpx = _rasterize_geometry(
                        focus_geometry if focus_geometry is not None and not getattr(focus_geometry, "is_empty", True) else component_geometry,
                        resolution_mm=max(0.03, line_width_mm * 0.08),
                        pad_mm=max(line_width_mm, 0.6),
                    )
                    for index, candidate in enumerate(_medial_repair_candidates_for_region_blob(
                        local_mask,
                        focus_mask,
                        origin_x=local_origin_x,
                        origin_y=local_origin_y,
                        px_per_mm=local_px_per_mm,
                        component_id=missed_id,
                    )[:8]):
                        candidate.metadata = {
                            **dict(candidate.metadata or {}),
                            "repair_mode": "thin-collapsed-detail-repair",
                            "component_id": int(missed_id),
                            "generated_from_safe_mask": False,
                            "safe_centerline_inset_mm": 0.0,
                            "missed_component_classification": classification,
                        }
                        component_candidates.append(("thin-collapsed-detail-repair", f"thin_medial_{index}", candidate, local_region))

            viable_candidate_rows: list[dict[str, Any]] = []
            for repair_mode, strategy_key, candidate, candidate_geometry in component_candidates:
                audit_row = _repair_candidate_audit(
                    candidate,
                    current_painted=current_painted,
                    target_mask=target_mask,
                    allowed_mask=allowed_mask,
                    safe_centerline_mask=safe_centerline_mask,
                    boundary_band_mask=boundary_band_mask,
                    current_to_source_matrix=current_to_source,
                    pen_radius_px=pen_radius_px,
                    component_geometry=candidate_geometry,
                    source=REPAIR_PATH_SOURCE,
                    repair_mode=repair_mode,
                    component_id=missed_id,
                )
                audit_row["missed_blob_id"] = int(missed_id)
                audit_row["missed_blob_classification"] = classification
                audit_row["strategy_key"] = strategy_key
                candidate_rows.append(audit_row)
                if audit_row["classification"] == "reject-useless-or-overflowing":
                    continue
                if audit_row["new_expected_pixels"] <= max(12, audit_row["redundant_pixels"]):
                    continue
                if float(audit_row["score"]) <= 0.0:
                    continue
                viable_candidate_rows.append(audit_row)

            if not viable_candidate_rows:
                continue
            viable_candidate_rows.sort(key=lambda row: float(row["score"]), reverse=True)
            accepted_for_blob = 0
            for viable_row in viable_candidate_rows[:3]:
                refreshed_row = _repair_candidate_audit(
                    viable_row["candidate"],
                    current_painted=current_painted,
                    target_mask=target_mask,
                    allowed_mask=allowed_mask,
                    safe_centerline_mask=safe_centerline_mask,
                    boundary_band_mask=boundary_band_mask,
                    current_to_source_matrix=current_to_source,
                    pen_radius_px=pen_radius_px,
                    component_geometry=local_region if viable_row["classification"] == "thin-collapsed-detail-repair" else center_domain,
                    source=REPAIR_PATH_SOURCE,
                    repair_mode=str(viable_row["classification"]),
                    component_id=missed_id,
                )
                if refreshed_row["classification"] == "reject-useless-or-overflowing":
                    continue
                if refreshed_row["new_expected_pixels"] <= max(8, refreshed_row["redundant_pixels"]):
                    continue
                if float(refreshed_row["score"]) <= 0.0:
                    continue
                exported_candidate = _export_repair_toolpath(
                    viable_row["candidate"],
                    strategy_key=str(viable_row.get("strategy_key", "")),
                    blob_id=int(missed_id),
                )
                exported_candidate.metadata = {
                    **dict(exported_candidate.metadata or {}),
                    "repair_mode": str(refreshed_row["classification"]),
                    "component_id": int(missed_id),
                    "missed_blob_classification": classification,
                }
                candidate_mask = _path_points_to_mask(
                    [exported_candidate],
                    shape=target_mask.shape,
                    current_to_source_matrix=current_to_source,
                    pen_radius_px=pen_radius_px,
                )
                current_painted = np.maximum(current_painted, candidate_mask)
                rebuilt_repairs.append(exported_candidate)
                accepted_this_iteration = True
                accepted_for_blob += 1
                if accepted_for_blob >= 2:
                    break
        if not accepted_this_iteration:
            break

    repaired_paths = base_paths + rebuilt_repairs
    repaired_mask = _path_points_to_mask(
        repaired_paths,
        shape=target_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_px=pen_radius_px,
    )
    original_mask = _path_points_to_mask(
        base_paths + existing_repairs,
        shape=target_mask.shape,
        current_to_source_matrix=current_to_source,
        pen_radius_px=pen_radius_px,
    )
    repaired_metrics = _painted_metrics(
        target_mask=target_mask,
        painted_mask=repaired_mask,
        allowed_mask=allowed_mask,
        px_per_mm=px_per_mm,
    )
    original_metrics = _painted_metrics(
        target_mask=target_mask,
        painted_mask=original_mask,
        allowed_mask=allowed_mask,
        px_per_mm=px_per_mm,
    )
    if float(repaired_metrics["painted_inside_area_mm2"]) + 1e-9 < float(original_metrics["painted_inside_area_mm2"]):
        return existing_repairs, audit_rows, candidate_rows
    if (
        float(repaired_metrics["overflow_area_mm2"]) > float(original_metrics["overflow_area_mm2"]) + 0.02
        and float(repaired_metrics["coverage_percent"]) <= float(original_metrics["coverage_percent"]) + 0.05
    ):
        return existing_repairs, audit_rows, candidate_rows
    return rebuilt_repairs, audit_rows, candidate_rows


def _source_thin_region_centerline_pass(
    *,
    thin_region_infos: list[dict[str, Any]],
    current_paths: list[Toolpath],
    line_width_mm: float,
    simplify_tolerance_mm: float,
) -> tuple[list[Toolpath], dict[str, Any], list[dict[str, Any]]]:
    if not thin_region_infos:
        return [], {
            "thin_source_region_detection_ran": False,
            "thin_source_region_count": 0,
            "thin_centerline_candidate_count": 0,
            "thin_centerline_accepted_count": 0,
            "thin_centerline_total_length_mm": 0.0,
            "thin_centerline_paths_exported": False,
            "thin_skeleton_component_count": 0,
            "useful_thin_skeleton_length_mm": 0.0,
            "accepted_thin_centerline_length_mm": 0.0,
            "accepted_thin_centerline_fraction": 0.0,
        }, []

    accepted_paths: list[Toolpath] = []
    candidate_rows: list[dict[str, Any]] = []
    candidate_count = 0
    total_length_mm = 0.0
    useful_skeleton_length_mm = 0.0
    useful_skeleton_component_count = 0
    pen_radius_mm = float(line_width_mm) * 0.5
    thin_radius_limit_mm = pen_radius_mm + THIN_TOL_MM
    bridge_radius_limit_mm = max(thin_radius_limit_mm + THIN_TOL_MM, pen_radius_mm * 1.75)

    for info in thin_region_infos:
        region_geometry = info.get("geometry")
        region_mask = np.asarray(info.get("mask"), dtype=np.uint8)
        if region_geometry is None or getattr(region_geometry, "is_empty", True) or not np.any(region_mask > 0):
            continue
        region_px_per_mm = float(info["px_per_mm"])
        region_pen_radius_px = max(1, int(round((line_width_mm * 0.5) * region_px_per_mm)))
        region_current_to_source = (
            region_px_per_mm,
            0.0,
            0.0,
            region_px_per_mm,
            -float(info["origin_x"]) * region_px_per_mm,
            -float(info["origin_y"]) * region_px_per_mm,
        )
        current_region_painted = _path_points_to_mask(
            _detail_coverage_paths([*current_paths, *accepted_paths]),
            shape=region_mask.shape,
            current_to_source_matrix=region_current_to_source,
            pen_radius_px=region_pen_radius_px,
        )
        missed_mask = ((region_mask > 0) & ~(current_region_painted > 0)).astype(np.uint8) * 255
        if not np.any(missed_mask > 0):
            continue
        skeleton = _skeletonize_mask((region_mask > 0).astype(np.uint8))
        neighbors = _skeleton_neighbors(skeleton)
        if not neighbors:
            continue
        distance = cv2.distanceTransform((region_mask > 0).astype(np.uint8), cv2.DIST_L2, 3)
        missed_proximity = cv2.dilate(
            (missed_mask > 0).astype(np.uint8),
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (max(1, region_pen_radius_px) * 2 + 1, max(1, region_pen_radius_px) * 2 + 1)),
            iterations=1,
        ) > 0
        component_index = 0
        for component_nodes in _connected_skeleton_components(neighbors):
            thin_nodes = {
                node for node in component_nodes
                if (float(distance[node[0], node[1]]) / max(region_px_per_mm, 1e-9)) <= thin_radius_limit_mm
            }
            if not thin_nodes:
                continue
            seed_nodes = {
                node for node in thin_nodes
                if 0 <= node[0] < missed_proximity.shape[0]
                and 0 <= node[1] < missed_proximity.shape[1]
                and bool(missed_proximity[node[0], node[1]])
            }
            if not seed_nodes:
                continue
            selected_nodes: set[tuple[int, int]] = set(seed_nodes)
            stack = list(seed_nodes)
            while stack:
                current = stack.pop()
                for neighbor in neighbors.get(current, []):
                    if neighbor not in component_nodes or neighbor in selected_nodes:
                        continue
                    neighbor_radius_mm = float(distance[neighbor[0], neighbor[1]]) / max(region_px_per_mm, 1e-9)
                    if neighbor_radius_mm <= bridge_radius_limit_mm:
                        selected_nodes.add(neighbor)
                        stack.append(neighbor)
            if len(selected_nodes) < 2:
                continue
            component_routes, component_length_mm = _cover_skeleton_subgraph_with_routes(
                nodes=selected_nodes,
                neighbors=neighbors,
                px_per_mm=region_px_per_mm,
                origin_x=float(info["origin_x"]),
                origin_y=float(info["origin_y"]),
                simplify_tolerance_mm=max(0.015, simplify_tolerance_mm),
                kind="detail-trace",
                min_route_length_mm=max(0.12, line_width_mm * 0.5),
            )
            if not component_routes or component_length_mm <= 1e-9:
                continue
            useful_skeleton_component_count += 1
            useful_skeleton_length_mm += float(component_length_mm)
            component_index += 1
            component_routes.sort(key=lambda route: float(pipeline_core.segment_length(route.points)), reverse=True)
            component_accepted_length_mm = 0.0
            component_target_fraction = 0.7
            long_axis_mm = float(max(info.get("max_width_mm", 0.0), info.get("equivalent_diameter_mm", 0.0), component_length_mm))
            for route_index, candidate in enumerate(component_routes, start=1):
                candidate.metadata = {
                    **dict(candidate.metadata or {}),
                    "source_region_id": f"detail_region_{int(info['component_id']):03d}",
                    "detail_component_id": int(info["component_id"]),
                    "detail_region_component_id": int(info["component_id"]),
                    "detail_region_classification": "thin-source",
                    "small_detail_fill_style": "thin_region_centerline",
                    "thin_source_region_centerline": True,
                    "thin_source_region": True,
                    "thin_skeleton_component_index": int(component_index),
                    "thin_skeleton_component_useful_length_mm": float(component_length_mm),
                    "thin_skeleton_route_index": int(route_index),
                }
                candidate_count += 1
                before_blob_rows = _connected_mask_blob_rows(
                    missed_mask.astype(np.uint8),
                    px_per_mm=region_px_per_mm,
                    min_blob_area_mm2=0.01,
                )
                before_blob_count = sum(
                    1 for blob in before_blob_rows
                    if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > 0.10
                )
                candidate_mask = _path_points_to_mask(
                    [candidate],
                    shape=region_mask.shape,
                    current_to_source_matrix=region_current_to_source,
                    pen_radius_px=region_pen_radius_px,
                )
                candidate_bool = candidate_mask > 0
                trial_painted = np.maximum(current_region_painted, candidate_mask)
                after_missed_mask = ((region_mask > 0) & ~(trial_painted > 0)).astype(np.uint8)
                after_blob_rows = _connected_mask_blob_rows(
                    after_missed_mask,
                    px_per_mm=region_px_per_mm,
                    min_blob_area_mm2=0.01,
                )
                after_blob_count = sum(
                    1 for blob in after_blob_rows
                    if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > 0.10
                )
                overlap_ratio = float(np.count_nonzero(candidate_bool & (current_region_painted > 0)) / max(1, np.count_nonzero(candidate_bool)))
                length_mm = float(pipeline_core.segment_length(candidate.points))
                length_coverage_ratio = length_mm / max(component_length_mm, 1e-6)
                component_coverage_ratio_before = component_accepted_length_mm / max(component_length_mm, 1e-9)
                medial_quality = _candidate_medial_quality(candidate, region_geometry)
                audit_row = _repair_candidate_audit(
                    candidate,
                    current_painted=current_region_painted,
                    target_mask=(region_mask > 0).astype(np.uint8) * 255,
                    allowed_mask=(region_mask > 0).astype(np.uint8) * 255,
                    safe_centerline_mask=(region_mask > 0).astype(np.uint8) * 255,
                    boundary_band_mask=cv2.morphologyEx(
                        (region_mask > 0).astype(np.uint8) * 255,
                        cv2.MORPH_GRADIENT,
                        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (region_pen_radius_px * 2 + 1, region_pen_radius_px * 2 + 1)),
                    ),
                    current_to_source_matrix=region_current_to_source,
                    pen_radius_px=region_pen_radius_px,
                    component_geometry=region_geometry,
                    source="thin_source_region_centerline",
                    repair_mode="thin-collapsed-detail-centerline",
                    component_id=int(info["component_id"]),
                )
                audit_row.update({
                    "overlap_ratio": float(overlap_ratio),
                    "length_coverage_ratio": float(length_coverage_ratio),
                    "medial_axis_distance_quality": float(medial_quality),
                    "missed_blob_count_before": int(before_blob_count),
                    "missed_blob_count_after": int(after_blob_count),
                    "missed_blob_count_reduced": int(max(0, before_blob_count - after_blob_count)),
                    "useful_thin_skeleton_length_mm": float(component_length_mm),
                    "thin_skeleton_component_index": int(component_index),
                    "thin_feature_length_coverage_ratio_before": float(component_coverage_ratio_before),
                })
                candidate_rows.append(audit_row)
                if audit_row["new_expected_pixels"] <= max(12, audit_row["redundant_pixels"]):
                    continue
                if overlap_ratio >= 0.96:
                    continue
                if length_coverage_ratio < 0.08 and length_mm < max(1.5, line_width_mm * 2.0):
                    continue
                if medial_quality < max(0.005, line_width_mm * 0.01):
                    continue
                overflow_ratio = float(audit_row["overflow_pixels"]) / max(1.0, float(audit_row["new_expected_pixels"]))
                if overflow_ratio > 0.45:
                    continue
                route_expands_component = (
                    component_coverage_ratio_before < component_target_fraction
                    and length_mm >= max(1.0, line_width_mm * 1.5)
                    and overflow_ratio <= 0.35
                )
                if (
                    int(audit_row["missed_blob_count_reduced"]) <= 0
                    and audit_row["new_expected_pixels"] <= max(60, audit_row["redundant_pixels"])
                    and not route_expands_component
                ):
                    continue
                accepted = Toolpath(
                    points=[Point(float(point.x), float(point.y)) for point in candidate.points],
                    kind="detail-trace",
                    closed=False,
                    source="thin_source_region_centerline",
                    region_id=int(info["component_id"]),
                    metadata={
                        **dict(candidate.metadata or {}),
                        "path_role": "PRINT_DETAIL",
                        "detail_source": "thin_source_region_centerline",
                        "repair_mode": "thin-collapsed-detail-centerline",
                        "thin_feature_length_coverage_ratio_before": float(component_coverage_ratio_before),
                        "thin_feature_length_coverage_ratio_after": float((component_accepted_length_mm + length_mm) / max(component_length_mm, 1e-9)),
                    },
                )
                accepted_paths.append(accepted)
                total_length_mm += length_mm
                component_accepted_length_mm += length_mm
                current_region_painted = trial_painted
                missed_mask = ((region_mask > 0) & ~(current_region_painted > 0)).astype(np.uint8) * 255

    return accepted_paths, {
        "thin_source_region_detection_ran": True,
        "thin_source_region_count": int(useful_skeleton_component_count),
        "thin_centerline_candidate_count": int(candidate_count),
        "thin_centerline_accepted_count": int(len(accepted_paths)),
        "thin_centerline_total_length_mm": float(total_length_mm),
        "thin_centerline_paths_exported": bool(accepted_paths),
        "thin_skeleton_component_count": int(useful_skeleton_component_count),
        "useful_thin_skeleton_length_mm": float(useful_skeleton_length_mm),
        "accepted_thin_centerline_length_mm": float(total_length_mm),
        "accepted_thin_centerline_fraction": float(total_length_mm / max(useful_skeleton_length_mm, 1e-9)) if useful_skeleton_length_mm > 0.0 else 0.0,
    }, candidate_rows


def _generate_debug_artifacts(
    *,
    output_dir: Path,
    shape: tuple[int, int],
    target_mask: np.ndarray,
    components_mask: np.ndarray,
    initial_paths: list[Toolpath],
    skeleton_paths: list[Toolpath],
    boundary_paths: list[Toolpath],
    final_paths: list[Toolpath],
    repair_candidates: list[dict[str, Any]],
    accepted_repair_paths: list[Toolpath],
    current_to_source_matrix: tuple[float, float, float, float, float, float],
    line_width_mm: float,
    pen_radius_px: int,
    px_per_mm: float,
    allowed_mask: np.ndarray,
    debug: dict[str, Any] | None = None,
) -> None:
    debug = debug or {}
    output_dir.mkdir(parents=True, exist_ok=True)
    origin_x = -float(current_to_source_matrix[4]) / max(1e-9, float(current_to_source_matrix[0]))
    origin_y = -float(current_to_source_matrix[5]) / max(1e-9, float(current_to_source_matrix[3]))
    cv2.imwrite(str(output_dir / "01_target_mask.png"), (target_mask > 0).astype(np.uint8) * 255)
    cv2.imwrite(str(output_dir / "02_components.png"), components_mask)
    cv2.imwrite(str(output_dir / "03_width_analysis.png"), _mask_to_overlay(np.maximum(components_mask, target_mask), (120, 180, 255)))
    cv2.imwrite(str(output_dir / "04_initial_serpentine_fill.png"), _render_paths_overlay(shape, initial_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (0, 128, 255), "fill-infill-travel": (255, 0, 255)}))
    cv2.imwrite(str(output_dir / "05_skeleton_paths.png"), _render_paths_overlay(shape, skeleton_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (255, 128, 0), "detail-trace": (255, 128, 0)}))
    cv2.imwrite(str(output_dir / "06_boundary_paths.png"), _render_paths_overlay(shape, boundary_paths, current_to_source_matrix=current_to_source_matrix, colors={"outline": (0, 180, 0)}))
    painted = _path_points_to_mask(final_paths, shape=shape, current_to_source_matrix=current_to_source_matrix, pen_radius_px=pen_radius_px)
    cv2.imwrite(str(output_dir / "07_simulated_painted_area.png"), _mask_to_overlay(painted, (80, 80, 80)))
    missed = ((target_mask > 0) & ~(painted > 0)).astype(np.uint8) * 255
    overflow = ((painted > 0) & ~(allowed_mask > 0)).astype(np.uint8) * 255
    cv2.imwrite(str(output_dir / "08_missed_pixels.png"), _mask_to_overlay(missed, (0, 0, 255)))
    cv2.imwrite(str(output_dir / "09_overflow_pixels.png"), _mask_to_overlay(overflow, (255, 0, 0)))
    candidate_paths = [item["candidate"] for item in repair_candidates if isinstance(item, dict) and item.get("candidate") is not None]
    cv2.imwrite(str(output_dir / "10_repair_candidates.png"), _render_paths_overlay(shape, candidate_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (255, 220, 0), "outline": (0, 200, 120), "coverage_centerline": (255, 160, 0), "detail-trace": (255, 160, 0)}))
    cv2.imwrite(str(output_dir / "11_accepted_repair_strokes.png"), _render_paths_overlay(shape, accepted_repair_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (220, 0, 0), "coverage_centerline": (220, 0, 0), "outline": (220, 0, 0), "detail-trace": (220, 0, 0)}))
    cv2.imwrite(str(output_dir / "12_final_paths.png"), _render_paths_overlay(shape, final_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (255, 128, 0), "coverage_centerline": (255, 128, 0), "outline": (0, 180, 0), "fill-infill-travel": (255, 0, 255), "detail-trace": (255, 128, 0)}))
    final_check = np.zeros((shape[0], shape[1], 3), dtype=np.uint8)
    final_check[target_mask > 0] = (0, 60, 0)
    final_check[overflow > 0] = (180, 0, 0)
    final_check[missed > 0] = (0, 0, 200)
    final_check[(painted > 0) & (target_mask > 0)] = (50, 180, 50)
    cv2.imwrite(str(output_dir / "13_final_coverage_check.png"), final_check)

    accepted_ids = {id(path) for path in accepted_repair_paths}
    before_repair_paths = [path for path in final_paths if id(path) not in accepted_ids]
    before_repair_painted = _path_points_to_mask(before_repair_paths, shape=shape, current_to_source_matrix=current_to_source_matrix, pen_radius_px=pen_radius_px)
    outline_only = [path for path in final_paths if path.kind == "outline"]
    outline_painted = _path_points_to_mask(outline_only, shape=shape, current_to_source_matrix=current_to_source_matrix, pen_radius_px=pen_radius_px)
    combined_after = painted
    missed_before = ((target_mask > 0) & ~(before_repair_painted > 0)).astype(np.uint8) * 255
    missed_after = ((target_mask > 0) & ~(combined_after > 0)).astype(np.uint8) * 255
    cv2.imwrite(str(output_dir / "target_mask.png"), (target_mask > 0).astype(np.uint8) * 255)
    cv2.imwrite(str(output_dir / "selected_color_mask.png"), (target_mask > 0).astype(np.uint8) * 255)
    cv2.imwrite(str(output_dir / "fill_coverage_before_repair.png"), _mask_to_overlay(before_repair_painted, (80, 120, 220)))
    cv2.imwrite(str(output_dir / "outline_coverage.png"), _mask_to_overlay(outline_painted, (0, 180, 0)))
    cv2.imwrite(str(output_dir / "combined_coverage_after_repair.png"), _mask_to_overlay(combined_after, (60, 160, 80)))
    cv2.imwrite(str(output_dir / "missed_blobs_before_repair.png"), _mask_to_overlay(missed_before, (0, 0, 255)))
    cv2.imwrite(str(output_dir / "missed_blobs_after_repair.png"), _mask_to_overlay(missed_after, (0, 0, 255)))
    cv2.imwrite(str(output_dir / "repair_strokes.png"), _render_paths_overlay(shape, accepted_repair_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (220, 0, 0), "coverage_centerline": (220, 0, 0), "outline": (220, 0, 0), "detail-trace": (220, 0, 0)}))
    cv2.imwrite(str(output_dir / "overflow_pixels.png"), _mask_to_overlay(overflow, (255, 0, 0)))
    cv2.imwrite(str(output_dir / "swept_coverage.png"), _mask_to_overlay(combined_after, (60, 160, 80)))
    cv2.imwrite(str(output_dir / "missed_blobs.png"), _mask_to_overlay(missed_after, (0, 0, 255)))
    final_blob_rows = _connected_mask_blob_rows((missed_after > 0).astype(np.uint8), px_per_mm=px_per_mm, min_blob_area_mm2=0.01)
    cv2.imwrite(str(output_dir / "missed_blobs_labeled.png"), _render_labeled_blob_overlay((missed_after > 0).astype(np.uint8), final_blob_rows))
    cv2.imwrite(str(output_dir / "repair_candidates.png"), _render_paths_overlay(shape, candidate_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (255, 220, 0), REPAIR_PATH_KIND: (255, 220, 0), "coverage_centerline": (255, 160, 0)}))
    rejected_candidate_paths = [item["candidate"] for item in repair_candidates if isinstance(item, dict) and item.get("candidate") is not None and not bool(item.get("accepted", False))]
    cv2.imwrite(str(output_dir / "rejected_repair_candidates.png"), _render_paths_overlay(shape, rejected_candidate_paths, current_to_source_matrix=current_to_source_matrix, colors={"fill-infill": (255, 80, 80), REPAIR_PATH_KIND: (255, 80, 80), "coverage_centerline": (255, 80, 80)}))
    cv2.imwrite(str(output_dir / "exported_repair_paths.png"), _render_paths_overlay(shape, accepted_repair_paths, current_to_source_matrix=current_to_source_matrix, colors={REPAIR_PATH_KIND: (220, 0, 0), "fill-infill": (220, 0, 0), "coverage_centerline": (220, 0, 0)}))

    final_blob_diagnostics: list[dict[str, Any]] = []
    repair_row_lookup = {
        int(row.get("component_id", -1)): row
        for row in list(debug.get("detail_region_repair_rows", []))
        if isinstance(row, dict) and row.get("component_id") is not None
    }
    for blob_row in final_blob_rows:
        centroid_px, centroid_mm = _centroid_from_mask(np.asarray(blob_row["mask"], dtype=np.uint8), px_per_mm=px_per_mm, origin_x=origin_x, origin_y=origin_y)
        nearest_path = _nearest_path_debug_row(final_paths, centroid_mm)
        nearest_region_row = None
        nearest_region_id = ""
        if nearest_path["nearest_existing_path_kind"]:
            for path in final_paths:
                if str(path.path_id or "") == str(nearest_path["nearest_existing_path_id"]):
                    nearest_region_id = str(path.region_id or (path.metadata or {}).get("source_region_id", ""))
                    component_id = (path.metadata or {}).get("detail_region_component_id") or path.region_id
                    try:
                        nearest_region_row = repair_row_lookup.get(int(component_id))
                    except Exception:
                        nearest_region_row = None
                    break
        bbox_mm = _bbox_mm_from_blob_row(blob_row, px_per_mm=px_per_mm, origin_x=origin_x, origin_y=origin_y)
        rejection_reason = str((nearest_region_row or {}).get("failure_reason_if_still_below_90", "") or "")
        if rejection_reason:
            root_cause_category = "E"
        elif not bool(debug.get("preview_and_gcode_share_same_projected_paths", True)):
            root_cause_category = "G"
        else:
            root_cause_category = "B"
        final_blob_diagnostics.append({
            "missed_blob_id": int(blob_row.get("blob_id", 0)),
            "area_mm2": float(blob_row.get("area_mm2", 0.0)),
            "equivalent_diameter_mm": float(_equivalent_diameter_mm(float(blob_row.get("area_mm2", 0.0)))),
            "bbox_px": {
                "left": int(blob_row.get("left_px", 0)),
                "top": int(blob_row.get("top_px", 0)),
                "width": int(blob_row.get("width_px", 0)),
                "height": int(blob_row.get("height_px", 0)),
            },
            "bbox_mm": bbox_mm,
            "centroid_px": centroid_px,
            "centroid_mm": centroid_mm,
            **nearest_path,
            "nearest_region_id": str(nearest_region_id),
            "region_classification": str((nearest_region_row or {}).get("classification", (nearest_region_row or {}).get("region_classification", "wide"))),
            "region_strategy_used": list((nearest_region_row or {}).get("strategy_used", [])),
            "was_inside_selected_color_mask": True,
            "was_inside_outline_area": True,
            "was_inside_inset_outline_area": True,
            "was_excluded_by_noise_filter": False,
            "was_excluded_by_detail_filter": False,
            "was_excluded_by_outline_clip": rejection_reason == "outside_selected_mask_overflow",
            "was_excluded_by_hole_detection": False,
            "was_excluded_by_projection_bounds": False,
            "repair_candidate_generated": bool((nearest_region_row or {}).get("detail_repair_iterations", 0) > 0),
            "repair_candidate_rejected": bool(rejection_reason),
            "repair_rejection_reason": rejection_reason,
            "repair_exported": False,
            "root_cause_category": root_cause_category,
        })
    with open(output_dir / "missed_blob_diagnostics.json", "w", encoding="utf-8") as handle:
        json.dump(final_blob_diagnostics, handle, indent=2)

    coverage_report = {"pen_radius_px": int(pen_radius_px), **_painted_metrics(target_mask=target_mask, painted_mask=painted, allowed_mask=allowed_mask, px_per_mm=px_per_mm)}
    coverage_report.update({
        "number_of_components": int(sum(1 for path in final_paths if path.kind != "detail-trace")),
        "number_of_serpentine_strokes": int(sum(1 for path in final_paths if str((path.metadata or {}).get("fill_mode", "")) == "serpentine")),
        "number_of_skeleton_strokes": int(sum(1 for path in final_paths if str((path.metadata or {}).get("fill_mode", "")) == "skeleton")),
        "number_of_boundary_strokes": int(sum(1 for path in final_paths if path.kind == "outline")),
        "number_of_repair_strokes": int(len(accepted_repair_paths)),
        "total_draw_length_mm": float(sum(pipeline_core.segment_length(path.points) for path in final_paths if len(path.points) >= 2 and path.kind != "fill-infill-travel")),
        "total_travel_length_mm": float(sum(pipeline_core.segment_length(path.points) for path in final_paths if path.kind == "fill-infill-travel")),
    })
    # Additional diagnostics and metadata
    coverage_report["pen_diameter_mm"] = float(line_width_mm)
    coverage_report["effective_infill_spacing_mm"] = float(debug.get("infill_debug", {}).get("spacing_mm", 0.0))
    coverage_report["number_of_raw_infill_segments"] = int(sum(1 for path in initial_paths if path.kind == "fill-infill"))
    coverage_report["number_of_chained_infill_paths"] = int(sum(1 for path in final_paths if path.kind == "fill-infill" and bool((path.metadata or {}).get("chained_infill", False))))
    coverage_report["number_of_pen_lifts_before_optimization"] = int(debug.get("pen_lifts_before_optimization", 0))
    coverage_report["number_of_pen_lifts_after_optimization"] = int(debug.get("pen_lifts_after_optimization", 0) or 0)
    coverage_report["outline_added_after_fill"] = bool(debug.get("infill_debug", {}).get("diagnostics", {}).get("outline_after_fill", False))
    coverage_report["coverage_before_outline_percent"] = float(debug.get("coverage_before_outline_percent", 0.0))
    coverage_report["missed_area_before_outline_mm2"] = float(debug.get("missed_area_before_outline_mm2", 0.0))
    coverage_report["largest_missed_blob_before_outline_mm"] = float(debug.get("largest_missed_blob_before_outline_mm", 0.0))
    coverage_report["endpoint_extension_mm"] = float(debug.get("infill_debug", {}).get("endpoint_extension_mm", line_width_mm * 0.5))
    coverage_report["endpoint_extensions_added"] = int(debug.get("endpoint_extensions_added", 0))
    coverage_report["endpoint_extensions_clipped"] = int(debug.get("endpoint_extensions_clipped", 0))
    coverage_report["outline_overlap_allowed"] = bool(debug.get("outline_overlap_allowed", True))
    coverage_report["fill_uses_outline_clearance"] = bool(debug.get("fill_uses_outline_clearance", False))
    coverage_report["outside_overflow_mm2"] = float(debug.get("outside_overflow_mm2", coverage_report.get("overflow_area_mm2", 0.0)))
    coverage_report["coverage_after_outline_percent"] = float(debug.get("coverage_after_outline_percent", coverage_report.get("coverage_percent", 0.0)))
    coverage_report["endpoint_clamp_mode"] = str(debug.get("endpoint_clamp_mode", "postprocess_only"))
    coverage_report["line_generation_changed"] = bool(debug.get("line_generation_changed", False))
    coverage_report["global_fill_mask_changed"] = bool(debug.get("global_fill_mask_changed", False))
    coverage_report["infill_spacing_mm"] = float(debug.get("infill_debug", {}).get("spacing_mm", 0.0))
    coverage_report["line_width_mm"] = float(line_width_mm)
    coverage_report["endpoints_checked"] = int(debug.get("endpoints_checked", 0))
    coverage_report["endpoints_clamped"] = int(debug.get("endpoints_clamped", 0))
    coverage_report["max_endpoint_retract_mm"] = float(debug.get("max_endpoint_retract_mm", 0.0))
    coverage_report["infill_beyond_outline_before_mm2"] = float(debug.get("infill_beyond_outline_before_mm2", 0.0))
    coverage_report["infill_beyond_outline_after_mm2"] = float(debug.get("infill_beyond_outline_after_mm2", 0.0))
    coverage_report["coverage_before_endpoint_clamp_percent"] = float(debug.get("coverage_report", {}).get("coverage_before_endpoint_clamp_percent", 0.0))
    coverage_report["coverage_after_endpoint_clamp_percent"] = float(debug.get("coverage_report", {}).get("coverage_after_endpoint_clamp_percent", 0.0))
    coverage_report["missed_blob_debug_enabled"] = bool(debug.get("missed_blob_debug_enabled", True))
    coverage_report["coverage_target"] = str(debug.get("coverage_validation_target", "selected_color_mask"))
    coverage_report["repair_clipping"] = str(debug.get("repair_clipped_against", "selected_color_mask"))
    coverage_report["fill_allowed_to_overlap_outline"] = bool(debug.get("fill_allowed_to_overlap_outline", True))
    coverage_report["missed_blob_count_before_repair"] = int(debug.get("missed_blob_count_before_repair", 0))
    coverage_report["missed_blob_count_after_repair"] = int(debug.get("missed_blob_count_after_repair", 0))
    coverage_report["largest_missed_blob_area_mm2_before"] = float(debug.get("largest_missed_blob_area_mm2_before", 0.0))
    coverage_report["largest_missed_blob_area_mm2_after"] = float(debug.get("largest_missed_blob_area_mm2_after", 0.0))
    coverage_report["largest_missed_blob_equivalent_diameter_mm_before"] = float(debug.get("largest_missed_blob_equivalent_diameter_mm_before", 0.0))
    coverage_report["largest_missed_blob_equivalent_diameter_mm_after"] = float(debug.get("largest_missed_blob_equivalent_diameter_mm_after", 0.0))
    coverage_report["repair_candidates_generated"] = int(debug.get("repair_candidates_generated", 0))
    coverage_report["repair_candidates_accepted"] = int(debug.get("repair_candidates_accepted", 0))
    coverage_report["repair_candidates_rejected"] = int(debug.get("repair_candidates_rejected", 0))
    coverage_report["top_repair_rejection_reasons"] = dict(debug.get("top_repair_rejection_reasons", {}))
    coverage_report["repair_paths_exported"] = bool(debug.get("repair_paths_exported", False))
    coverage_report["coverage_preview_gcode_consistent"] = bool(debug.get("coverage_preview_gcode_consistent", True))
    coverage_report["root_cause_category"] = str(debug.get("root_cause_category", ""))
    with open(output_dir / "coverage_report.json", "w", encoding="utf-8") as handle:
        json.dump(coverage_report, handle, indent=2)
    with open(output_dir / "repair_candidates.json", "w", encoding="utf-8") as handle:
        json.dump([
            {key: value for key, value in item.items() if key != "candidate"} for item in repair_candidates
        ], handle, indent=2)
    with open(output_dir / "path_stats.json", "w", encoding="utf-8") as handle:
        stats = Counter(path.kind for path in final_paths)
        json.dump({
            "total_paths": len(final_paths),
            "paths_by_kind": dict(stats),
            "frontend_generate_request_received": bool(debug.get("frontend_generate_request_received", False)),
            "lineWidthMm": float(debug.get("lineWidthMm", line_width_mm)),
            "rotationDeg": float(debug.get("rotationDeg", 0.0)),
            "source_mask_component_count": int(debug.get("source_mask_component_count", 0)),
            "normal_safe_infill_enabled": bool(debug.get("normal_safe_infill_enabled", True)),
            "thin_source_region_pass_enabled": bool(debug.get("thin_source_region_pass_enabled", False)),
            "thin_source_region_pass_ran": bool(debug.get("thin_source_region_pass_ran", False)),
            "thin_source_region_count": int(debug.get("thin_source_region_count", 0)),
            "thin_centerline_candidate_count": int(debug.get("thin_centerline_candidate_count", 0)),
            "thin_centerline_accepted_count": int(debug.get("thin_centerline_accepted_count", 0)),
            "thin_centerline_exported_count": int(debug.get("thin_centerline_exported_count", 0)),
            "thin_skeleton_component_count": int(debug.get("thin_skeleton_component_count", 0)),
            "useful_thin_skeleton_length_mm": float(debug.get("useful_thin_skeleton_length_mm", 0.0)),
            "accepted_thin_centerline_length_mm": float(debug.get("accepted_thin_centerline_length_mm", 0.0)),
            "accepted_thin_centerline_fraction": float(debug.get("accepted_thin_centerline_fraction", 0.0)),
            "final_path_count_before_thin_centerlines": int(debug.get("final_path_count_before_thin_centerlines", 0)),
            "final_path_count_after_thin_centerlines": int(debug.get("final_path_count_after_thin_centerlines", 0)),
            "total_draw_length_mm": coverage_report["total_draw_length_mm"],
            "total_travel_length_mm": coverage_report["total_travel_length_mm"],
            "pen_diameter_mm": float(line_width_mm),
            "infill_overlap_percent": float(debug.get("infill_debug", {}).get("infill_overlap_percent", 0.0)),
            "effective_infill_spacing_mm": float(debug.get("infill_debug", {}).get("spacing_mm", 0.0)),
            "number_of_raw_infill_segments": int(sum(1 for path in initial_paths if path.kind == "fill-infill")),
            "number_of_chained_infill_paths": int(sum(1 for path in final_paths if path.kind == "fill-infill" and bool((path.metadata or {}).get("chained_infill", False)))),
            "number_of_pen_lifts_before_optimization": int(debug.get("pen_lifts_before_optimization", 0)),
            "number_of_pen_lifts_after_optimization": int(debug.get("pen_lifts_after_optimization", 0)),
            "coverage_before_outline_percent": float(debug.get("coverage_before_outline_percent", 0.0)),
            "missed_area_before_outline_mm2": float(debug.get("missed_area_before_outline_mm2", 0.0)),
            "largest_missed_blob_before_outline_mm": float(debug.get("largest_missed_blob_before_outline_mm", 0.0)),
            "endpoint_extension_mm": float(debug.get("infill_debug", {}).get("endpoint_extension_mm", line_width_mm * 0.5)),
            "endpoint_extensions_added": int(debug.get("endpoint_extensions_added", 0)),
            "endpoint_extensions_clipped": int(debug.get("endpoint_extensions_clipped", 0)),
            "outline_overlap_allowed": bool(debug.get("outline_overlap_allowed", True)),
            "fill_uses_outline_clearance": bool(debug.get("fill_uses_outline_clearance", False)),
            "endpoint_clamp_mode": str(debug.get("endpoint_clamp_mode", "postprocess_only")),
            "line_generation_changed": bool(debug.get("line_generation_changed", False)),
            "global_fill_mask_changed": bool(debug.get("global_fill_mask_changed", False)),
            "infill_spacing_mm": float(debug.get("infill_debug", {}).get("spacing_mm", 0.0)),
            "line_width_mm": float(line_width_mm),
            "endpoints_checked": int(debug.get("endpoints_checked", 0)),
            "endpoints_clamped": int(debug.get("endpoints_clamped", 0)),
            "max_endpoint_retract_mm": float(debug.get("max_endpoint_retract_mm", 0.0)),
            "infill_beyond_outline_before_mm2": float(debug.get("infill_beyond_outline_before_mm2", 0.0)),
            "infill_beyond_outline_after_mm2": float(debug.get("infill_beyond_outline_after_mm2", 0.0)),
            "coverage_before_endpoint_clamp_percent": float(debug.get("coverage_report", {}).get("coverage_before_endpoint_clamp_percent", 0.0)),
            "coverage_after_endpoint_clamp_percent": float(debug.get("coverage_report", {}).get("coverage_after_endpoint_clamp_percent", 0.0)),
            "coverage_after_outline_percent": float(debug.get("coverage_after_outline_percent", coverage_report.get("coverage_percent", 0.0))),
            "missed_blob_debug_enabled": bool(debug.get("missed_blob_debug_enabled", True)),
            "coverage_target": str(debug.get("coverage_validation_target", "selected_color_mask")),
            "repair_clipping": str(debug.get("repair_clipped_against", "selected_color_mask")),
            "fill_allowed_to_overlap_outline": bool(debug.get("fill_allowed_to_overlap_outline", True)),
            "missed_blob_count_before_repair": int(debug.get("missed_blob_count_before_repair", 0)),
            "missed_blob_count_after_repair": int(debug.get("missed_blob_count_after_repair", 0)),
            "largest_missed_blob_area_mm2_before": float(debug.get("largest_missed_blob_area_mm2_before", 0.0)),
            "largest_missed_blob_area_mm2_after": float(debug.get("largest_missed_blob_area_mm2_after", 0.0)),
            "largest_missed_blob_equivalent_diameter_mm_before": float(debug.get("largest_missed_blob_equivalent_diameter_mm_before", 0.0)),
            "largest_missed_blob_equivalent_diameter_mm_after": float(debug.get("largest_missed_blob_equivalent_diameter_mm_after", 0.0)),
            "repair_candidates_generated": int(debug.get("repair_candidates_generated", 0)),
            "repair_candidates_accepted": int(debug.get("repair_candidates_accepted", 0)),
            "repair_candidates_rejected": int(debug.get("repair_candidates_rejected", 0)),
            "top_repair_rejection_reasons": dict(debug.get("top_repair_rejection_reasons", {})),
            "repair_paths_exported": bool(debug.get("repair_paths_exported", False)),
            "coverage_preview_gcode_consistent": bool(debug.get("coverage_preview_gcode_consistent", True)),
            "root_cause_category": str(debug.get("root_cause_category", "")),
            "accepted_connectors": int(sum(1 for path in final_paths if path.kind == "fill-infill-travel")),
            "travel_optimization_mode": str(debug.get("travel_optimization_mode", "")),
            "optimizer_runs_on_final_export_paths": bool(debug.get("optimizer_runs_on_final_export_paths", False)),
            "uses_surface_mm_for_ordering": bool(debug.get("uses_surface_mm_for_ordering", False)),
            "geometry_changed": bool(debug.get("geometry_changed", False)),
            "path_points_moved": bool(debug.get("path_points_moved", False)),
            "detail_filter_mode": str(debug.get("detail_filter_mode", pipeline_core.DETAIL_FILTER_MODE)),
            "detail_paths_generated_raw": int(debug.get("detail_paths_generated_raw", 0)),
            "detail_paths_kept": int(debug.get("detail_paths_kept", 0)),
            "detail_paths_dropped": int(debug.get("detail_paths_dropped", 0)),
            "detail_drop_reasons": dict(debug.get("detail_drop_reasons", {})),
            "detail_paths_dropped_as_travel_or_debug": int(debug.get("detail_paths_dropped_as_travel_or_debug", 0)),
            "detail_paths_dropped_as_redundant_overlap": int(debug.get("detail_paths_dropped_as_redundant_overlap", 0)),
            "detail_new_coverage_area_mm2_total": float(debug.get("detail_new_coverage_area_mm2_total", 0.0)),
            "detail_already_covered_ratio_avg": float(debug.get("detail_already_covered_ratio_avg", 0.0)),
            "detail_source_whitelist_enforced": bool(debug.get("detail_source_whitelist_enforced", True)),
            "travel_geometry_allowed_as_detail": bool(debug.get("travel_geometry_allowed_as_detail", False)),
            "infill_geometry_changed": bool(debug.get("infill_geometry_changed", False)),
            "outline_geometry_changed": bool(debug.get("outline_geometry_changed", False)),
            "detail_dropped_path_records": list(debug.get("detail_dropped_path_records", [])),
            "paths_reordered_count": int(debug.get("paths_reordered_count", 0)),
            "open_paths_reversed_count": int(debug.get("open_paths_reversed_count", 0)),
            "raw_travel_length_mm": float(debug.get("raw_travel_length_mm", 0.0)),
            "optimized_travel_length_mm": float(debug.get("optimized_travel_length_mm", 0.0)),
            "travel_length_reduction_percent": float(debug.get("travel_length_reduction_percent", 0.0)),
            "raw_longest_travel_mm": float(debug.get("raw_longest_travel_mm", 0.0)),
            "optimized_longest_travel_mm": float(debug.get("optimized_longest_travel_mm", 0.0)),
            "raw_travel_crossing_count": int(debug.get("raw_travel_crossing_count", 0)),
            "optimized_travel_crossing_count": int(debug.get("optimized_travel_crossing_count", 0)),
            "bad_choice_count_after_optimization": int(debug.get("bad_choice_count_after_optimization", 0)),
            "top_longest_travels_before": list(debug.get("top_longest_travels_before", [])),
            "top_longest_travels_after": list(debug.get("top_longest_travels_after", [])),
            "top_longest_travels": list(debug.get("top_longest_travels_after", []))[:25],
        }, handle, indent=2)


def plan_coverage_first_toolpaths(
    bundle: Any,
    *,
    enable_fill: bool,
    line_width_mm: float,
    wall_count: int,
    infill_density: float,
    infill_spacing_mm: float,
    infill_angle_deg: float,
    outline_after_fill: bool,
    min_fill_area_mm2: float,
    min_fill_width_mm: float,
    simplify_tolerance_mm: float,
    remove_duplicate_paths: bool,
    small_shape_mode: str,
    fill_strategy: str = "horizontal_scanline",
    alternate_fill_angle_deg: float = -45.0,
    thin_detail_mode: bool = pipeline_core.DEFAULT_THIN_DETAIL_MODE,
    thin_detail_min_area_mm2: float = pipeline_core.DEFAULT_THIN_DETAIL_MIN_AREA_MM2,
    thin_detail_simplify_mm: float = pipeline_core.DEFAULT_THIN_DETAIL_SIMPLIFY_MM,
    thin_detail_overlap: bool = pipeline_core.DEFAULT_THIN_DETAIL_OVERLAP,
    min_segment_length_mm: float = pipeline_core.DEFAULT_MIN_SEGMENT_LENGTH_MM,
    travel_optimization: str = pipeline_core.DEFAULT_TRAVEL_OPTIMIZATION,
    allow_pen_down_infill_connectors: bool = pipeline_core.DEFAULT_ALLOW_PEN_DOWN_INFILL_CONNECTORS,
    infill_path_mode: str = pipeline_core.DEFAULT_INFILL_PATH_MODE,
    infill_overlap_percent: float = pipeline_core.DEFAULT_INFILL_OVERLAP_PERCENT,
    expensive_coverage_repair: bool = True,
    debug: dict[str, Any] | None = None,
) -> list[Toolpath]:
    detail_tolerance_mm = max(simplify_tolerance_mm, thin_detail_simplify_mm)
    del wall_count, infill_density, min_fill_area_mm2, min_fill_width_mm, alternate_fill_angle_deg, thin_detail_min_area_mm2, thin_detail_overlap
    printable_geometry = getattr(bundle, "printable_geometry", None)
    if not enable_fill or printable_geometry is None or getattr(printable_geometry, "is_empty", True):
        outline_segments = list(getattr(bundle, "outline_segments", []))
        if not enable_fill:
            outline_segments.extend(getattr(bundle, "fill_boundary_segments", []))
        toolpaths: list[Toolpath] = []
        for segment in outline_segments:
            simplified = pipeline_core.simplify_segment_points(list(segment.points), simplify_tolerance_mm, bool(segment.closed))
            if len(simplified) < 2:
                continue
            toolpaths.append(Toolpath(points=simplified, kind="outline", closed=segment.closed, source="mask_contour", metadata={"simplify_tolerance_mm": float(simplify_tolerance_mm), "pen_width_mm": float(line_width_mm)}))
        return pipeline_core.assign_stable_path_ids(pipeline_core.merge_connected_toolpaths(toolpaths))

    pad_mm = max(0.75, line_width_mm * 2.0)
    resolution_mm = max(0.03, min(0.06, max(0.03, line_width_mm * 0.08)))
    target_mask, origin_x, origin_y, px_per_mm = _rasterize_geometry(printable_geometry, resolution_mm=resolution_mm, pad_mm=pad_mm)
    current_to_source = (px_per_mm, 0.0, 0.0, px_per_mm, -origin_x * px_per_mm, -origin_y * px_per_mm)
    allowed_mask = cv2.dilate(target_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (max(1, int(round((0.2 * px_per_mm) * 2)) + 1), max(1, int(round((0.2 * px_per_mm) * 2)) + 1))), iterations=1)
    connector_validation = dict((getattr(bundle, "metadata", {}) or {}).get("connector_validation", {}) or {})

    component_count, labels, stats, _centroids = cv2.connectedComponentsWithStats((target_mask > 0).astype(np.uint8), connectivity=8)
    component_ids = [index for index in range(1, int(component_count)) if int(stats[index, cv2.CC_STAT_AREA]) > 0]
    component_ids.sort(key=lambda idx: int(stats[idx, cv2.CC_STAT_AREA]), reverse=True)

    all_paths: list[Toolpath] = []
    initial_paths: list[Toolpath] = []
    skeleton_paths: list[Toolpath] = []
    boundary_paths: list[Toolpath] = []
    component_debug: list[dict[str, Any]] = []
    repair_candidate_rows: list[dict[str, Any]] = []
    accepted_repairs: list[Toolpath] = []
    pre_endpoint_clamp_paths: list[Toolpath] = []
    travel_distance_mm = 0.0
    detail_paths: list[Toolpath] = []
    fill_uses_outline_clearance = False
    outline_overlap_allowed = True
    pen_radius_mm = float(line_width_mm) * 0.5
    full_coverage_outline_offset_mm = pen_radius_mm * (
        0.0 if float(infill_spacing_mm if infill_spacing_mm > 0 else line_width_mm) >= (float(line_width_mm) * 0.995) else 1.0
    )
    source_outline_paths = _boundary_paths_for_geometry(
        printable_geometry,
        simplify_tolerance_mm=simplify_tolerance_mm,
        line_width_mm=line_width_mm,
        centerline_offset_mm=full_coverage_outline_offset_mm,
    )
    source_printable_parts = _geometry_parts(printable_geometry)
    detail_region_count = 0
    detail_regions_classified_wide = 0
    detail_regions_classified_thin = 0
    detail_regions_serpentine_filled = 0
    detail_regions_centerline_traced = 0
    detail_classification_threshold_width_mm = max(1.0, line_width_mm * 1.65)
    detail_classification_threshold_area_mm2 = 0.5
    arsenal_detail_serpentine_paths_generated = 0
    arsenal_detail_centerline_paths_generated = 0
    detail_region_total_area_mm2 = 0.0
    detail_region_covered_area_mm2 = 0.0
    classified_thin_component_ids: set[int] = set()
    detail_region_rows: list[dict[str, Any]] = []
    detail_repair_pass_enabled = False
    required_detail_coverage_percent = 98.0
    largest_allowed_missed_blob_equivalent_diameter_mm = 0.10
    min_repair_blob_area_mm2 = 0.01
    outside_region_overflow_tolerance_mm2 = 0.03
    max_detail_repair_iterations_per_region = 30
    detail_coverage_before_repair_percent = 0.0
    detail_coverage_after_repair_percent = 0.0
    detail_fillable_regions_total = 0
    detail_fillable_regions_failing_before_repair = 0
    detail_fillable_regions_failing_after_repair = 0
    detail_repair_regions_processed = 0
    detail_repair_strokes_added = 0
    detail_repair_iterations_total = 0
    detail_repair_new_coverage_mm2 = 0.0
    detail_repair_remaining_missed_area_mm2 = 0.0
    detail_repair_outside_overflow_mm2 = 0.0
    detail_regions_still_below_90: list[dict[str, Any]] = []
    detail_region_infos: list[dict[str, Any]] = []
    thin_source_region_infos: list[dict[str, Any]] = []
    preferred_centerline_outline_regions: list[Any] = []
    local_coverage_validation_enabled = False
    coverage_validation_target = "selected_color_mask"
    repair_clipped_against = "selected_color_mask"
    fill_allowed_to_overlap_outline = True
    regions_total = 0
    region_coverage_min_percent = 0.0
    missed_blob_count_before_repair = 0
    missed_blob_count_after_repair = 0
    largest_missed_blob_area_mm2_before = 0.0
    largest_missed_blob_area_mm2_after = 0.0
    largest_missed_blob_equivalent_diameter_mm_after = 0.0
    missed_blob_diagnostic_rows: list[dict[str, Any]] = []
    repair_rejection_reason_counts: Counter[str] = Counter()
    repair_candidates_generated = 0
    repair_candidates_accepted = 0
    repair_candidates_rejected = 0
    legacy_preview_only_repair_kind_detected = False
    thin_source_stats: dict[str, Any] = {
        "thin_source_region_detection_ran": False,
        "thin_source_region_count": 0,
        "thin_centerline_candidate_count": 0,
        "thin_centerline_accepted_count": 0,
        "thin_centerline_total_length_mm": 0.0,
        "thin_centerline_paths_exported": False,
    }
    final_path_count_before_thin_centerlines = 0
    final_path_count_after_thin_centerlines = 0

    for component_id in component_ids:
        component_mask = (labels == component_id).astype(np.uint8)
        ys, xs = np.nonzero(component_mask > 0)
        if xs.size == 0:
            continue
        comp_x0 = float(xs.min())
        comp_y0 = float(ys.min())
        component_geometry = _component_mask_to_geometry(component_mask * 255, origin_x=origin_x, origin_y=origin_y, px_per_mm=px_per_mm)
        if printable_geometry is not None and not getattr(printable_geometry, "is_empty", True) and component_geometry is not None and not getattr(component_geometry, "is_empty", True):
            component_geometry = component_geometry.intersection(printable_geometry)
        dt = cv2.distanceTransform(component_mask.astype(np.uint8), cv2.DIST_L2, 3)
        width_mm = float(np.max(dt) * 2.0 / max(1e-9, px_per_mm)) if np.any(dt > 0) else 0.0
        min_dim_mm = float(min(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1) / px_per_mm)
        thin_region = bool(width_mm <= line_width_mm * 1.5 or min_dim_mm <= line_width_mm * 1.5)
        prefer_connected_centerline = bool(thin_region and width_mm <= line_width_mm * 1.5)
        angle_deg = _component_fill_angle(component_mask)
        component_boundary_paths = _boundary_paths_for_component(
            component_geometry,
            component_id=component_id,
            simplify_tolerance_mm=simplify_tolerance_mm,
            line_width_mm=line_width_mm,
            centerline_offset_mm=full_coverage_outline_offset_mm,
        )
        if prefer_connected_centerline:
            component_boundary_paths = []
            if component_geometry is not None and not getattr(component_geometry, "is_empty", True):
                preferred_centerline_outline_regions.append(component_geometry)
        component_outline_footprint = _paths_footprint_union(component_boundary_paths, pen_radius_mm=pen_radius_mm)
        component_endpoint_limit = component_geometry.union(component_outline_footprint) if component_geometry is not None and not getattr(component_geometry, "is_empty", True) else component_outline_footprint
        if thin_region and thin_detail_mode:
            detail_style = "single_stroke_detail" if width_mm <= line_width_mm * 1.35 else "contour_following"
            fallback_spacing = max(0.0, line_width_mm * (1.0 - float(infill_overlap_percent) / 100.0))
            detail_fill_spacing_mm = min(max(0.25, infill_spacing_mm if infill_spacing_mm > 0 else fallback_spacing), max(0.35, line_width_mm))
            thin_paths, thin_stats = _thin_detail_fill_paths_for_component(
                component_mask,
                component_geometry=component_geometry,
                component_endpoint_limit=component_endpoint_limit,
                mark_outline_fallback=not bool(component_boundary_paths),
                prefer_connected_centerline=prefer_connected_centerline,
                origin_x=origin_x,
                origin_y=origin_y,
                px_per_mm=px_per_mm,
                component_id=component_id,
                line_width_mm=line_width_mm,
                simplify_tolerance_mm=simplify_tolerance_mm,
                detail_fill_spacing_mm=detail_fill_spacing_mm,
                angle_deg=angle_deg if math.isfinite(angle_deg) else infill_angle_deg,
                small_detail_fill_style=detail_style,
            )
            skeleton_paths.extend(thin_paths)
            all_paths.extend(thin_paths)
            pre_endpoint_clamp_paths.extend(_clone_toolpath(path) for path in thin_paths)
            component_debug.append({
                "component_id": int(component_id),
                "mode": "thin",
                "area_px": int(stats[component_id, cv2.CC_STAT_AREA]),
                "estimated_width_mm": float(width_mm),
                "skeleton_segment_count": int(thin_stats.get("skeleton_segment_count", 0)),
                "path_count": len(thin_paths),
                "thin_detail_initial_coverage_percent": float(thin_stats.get("thin_detail_initial_coverage_percent", 0.0)),
                "thin_detail_final_coverage_percent": float(thin_stats.get("thin_detail_final_coverage_percent", 0.0)),
                "thin_detail_initial_largest_missed_blob_mm": float(thin_stats.get("thin_detail_initial_largest_missed_blob_mm", 0.0)),
                "thin_detail_final_largest_missed_blob_mm": float(thin_stats.get("thin_detail_final_largest_missed_blob_mm", 0.0)),
                "thin_detail_supplemental_paths_added": int(thin_stats.get("thin_detail_supplemental_paths_added", 0)),
                "thin_detail_repair_paths_added": int(thin_stats.get("thin_detail_repair_paths_added", 0)),
                "thin_detail_repair_iterations": int(thin_stats.get("thin_detail_repair_iterations", 0)),
                "thin_detail_has_holes": bool(thin_stats.get("thin_detail_has_holes", False)),
            })
        else:
            fill_geometry = component_geometry
            fill_spacing_mm = min(max(0.25, infill_spacing_mm if infill_spacing_mm > 0 else max(0.0, line_width_mm * (1.0 - float(infill_overlap_percent) / 100.0))), max(0.35, line_width_mm))
            use_safe_centerline_fill = True
            safe_fill_geometry = _strict_safe_centerline_geometry(fill_geometry, inset_mm=pen_radius_mm)
            safe_fill_geometry_empty = bool(
                safe_fill_geometry is None or getattr(safe_fill_geometry, "is_empty", True)
            )
            has_holes = bool(component_geometry is not None and getattr(component_geometry, "interiors", None) and len(getattr(component_geometry, "interiors", [])) > 0)
            fill_mode_label = "detail_contour_cell" if has_holes and width_mm <= max(line_width_mm * 4.0, infill_spacing_mm * 4.0) else "serpentine"
            fill_paths, fill_stats = _scanline_fill_paths(
                fill_geometry,
                angle_deg=angle_deg if math.isfinite(angle_deg) else infill_angle_deg,
                spacing_mm=fill_spacing_mm,
                line_width_mm=line_width_mm,
                origin_x=origin_x,
                origin_y=origin_y,
                px_per_mm=px_per_mm,
                component_id=component_id,
                allow_connectors=allow_pen_down_infill_connectors,
                max_overflow_mm=0.0 if use_safe_centerline_fill else 0.05,
                fill_mode_label=fill_mode_label,
                intersection_geometry=safe_fill_geometry if use_safe_centerline_fill else None,
                endpoint_limit_geometry=safe_fill_geometry if use_safe_centerline_fill else None,
                centerline_generation_mode="safe-mask" if use_safe_centerline_fill else "raw-mask",
                safe_centerline_inset_mm=pen_radius_mm if use_safe_centerline_fill else 0.0,
            )
            if use_safe_centerline_fill:
                raw_preview_paths, _raw_preview_stats = _scanline_fill_paths(
                    fill_geometry,
                    angle_deg=angle_deg if math.isfinite(angle_deg) else infill_angle_deg,
                    spacing_mm=fill_spacing_mm,
                    line_width_mm=line_width_mm,
                    origin_x=origin_x,
                    origin_y=origin_y,
                    px_per_mm=px_per_mm,
                    component_id=component_id,
                    allow_connectors=allow_pen_down_infill_connectors,
                    max_overflow_mm=0.05,
                    fill_mode_label=fill_mode_label,
                    centerline_generation_mode="raw-mask",
                    safe_centerline_inset_mm=0.0,
                )
                unclamped_fill_paths = [_clone_toolpath(path) for path in raw_preview_paths]
            else:
                unclamped_fill_paths = [_clone_toolpath(path) for path in fill_paths]
            fill_paths, endpoint_clamp_stats = _clamp_infill_endpoints_to_outline_limit(
                fill_paths,
                allowed_geom=safe_fill_geometry if use_safe_centerline_fill and not safe_fill_geometry_empty else component_endpoint_limit,
                pen_radius_mm=pen_radius_mm,
                max_retract_mm=pen_radius_mm,
                precision_mm=0.02,
            )
            initial_paths.extend(fill_paths)
            all_paths.extend(fill_paths)
            pre_endpoint_clamp_paths.extend(unclamped_fill_paths)
            component_debug.append({
                "component_id": int(component_id),
                "mode": "wide",
                "area_px": int(stats[component_id, cv2.CC_STAT_AREA]),
                "estimated_width_mm": float(width_mm),
                "segment_count": int(fill_stats.get("segment_count", 0)),
                "row_count": int(fill_stats.get("row_count", 0)),
                "path_count": len(fill_paths),
                "endpoint_extension_mm": float(fill_stats.get("endpoint_extension_mm", line_width_mm * 0.5)),
                "endpoint_extensions_added": int(fill_stats.get("endpoint_extensions_added", 0)),
                "endpoint_extensions_clipped": int(fill_stats.get("endpoint_extensions_clipped", 0)),
                "endpoint_clamp_mode": str(endpoint_clamp_stats.get("endpoint_clamp_mode", "postprocess_only")),
                "endpoints_checked": int(endpoint_clamp_stats.get("endpoints_checked", 0)),
                "endpoints_clamped": int(endpoint_clamp_stats.get("endpoints_clamped", 0)),
                "max_endpoint_retract_mm": float(endpoint_clamp_stats.get("max_endpoint_retract_mm", 0.0)),
                "centerline_generation_mode": str(fill_stats.get("centerline_generation_mode", "raw-mask")),
                "safe_centerline_inset_mm": float(fill_stats.get("safe_centerline_inset_mm", 0.0)),
                "safe_centerline_geometry_empty": bool(use_safe_centerline_fill and safe_fill_geometry_empty),
                "infill_beyond_outline_before_mm2": float(_infill_beyond_outline_area_mm2(unclamped_fill_paths, allowed_geom=component_endpoint_limit, pen_radius_mm=pen_radius_mm)),
                "infill_beyond_outline_after_mm2": float(_infill_beyond_outline_area_mm2(fill_paths, allowed_geom=component_endpoint_limit, pen_radius_mm=pen_radius_mm)),
            })

        boundary = [] if prefer_connected_centerline else _boundary_paths_for_component(
            component_geometry,
            component_id=component_id,
            simplify_tolerance_mm=simplify_tolerance_mm,
            line_width_mm=line_width_mm,
            centerline_offset_mm=pen_radius_mm,
        )
        boundary_fallback = [] if prefer_connected_centerline else _boundary_fallback_paths_for_component(
            component_geometry,
            component_id=component_id,
            existing_boundary_paths=boundary,
            simplify_tolerance_mm=simplify_tolerance_mm,
            line_width_mm=line_width_mm,
        )
        boundary.extend(boundary_fallback)
        boundary_paths.extend(boundary)
        if outline_after_fill:
            all_paths.extend(boundary)
            pre_endpoint_clamp_paths.extend(_clone_toolpath(path) for path in boundary)
        else:
            all_paths = boundary + all_paths
            pre_endpoint_clamp_paths = [_clone_toolpath(path) for path in boundary] + pre_endpoint_clamp_paths

    use_source_geometry_outlines = len(component_ids) < len(source_printable_parts)
    if use_source_geometry_outlines:
        existing_fill_area = _paths_footprint_union(
            [path for path in all_paths if path.kind == "fill-infill"],
            pen_radius_mm=pen_radius_mm,
        )
        fallback_spacing = max(0.0, line_width_mm * (1.0 - float(infill_overlap_percent) / 100.0))
        detail_fill_spacing_mm = min(max(0.25, infill_spacing_mm if infill_spacing_mm > 0 else fallback_spacing), max(0.35, line_width_mm))
        for component_id, component_geometry in enumerate(source_printable_parts, start=1):
            if component_geometry is None or getattr(component_geometry, "is_empty", True):
                continue
            component_area_mm2 = float(component_geometry.area)
            if component_area_mm2 <= 1e-9:
                continue
            detail_region_count += 1
            detail_region_total_area_mm2 += component_area_mm2
            component_mask, comp_origin_x, comp_origin_y, comp_px_per_mm = _rasterize_geometry(
                component_geometry,
                resolution_mm=resolution_mm,
                pad_mm=max(0.25, line_width_mm),
            )
            dt = cv2.distanceTransform((component_mask > 0).astype(np.uint8), cv2.DIST_L2, 3)
            dt_positive = dt[dt > 0]
            max_width_mm = float(np.max(dt_positive) * 2.0 / max(1e-9, comp_px_per_mm)) if dt_positive.size else 0.0
            median_width_mm = float(np.median(dt_positive) * 2.0 / max(1e-9, comp_px_per_mm)) if dt_positive.size else 0.0
            equivalent_diameter_mm = _equivalent_diameter_mm(component_area_mm2)
            region_angle_deg = _component_fill_angle(component_mask)
            has_holes = bool(any(len(poly.interiors) > 0 for poly in _geometry_parts(component_geometry)))
            hole_driven_detail_fill = bool(
                has_holes
                and component_area_mm2 >= max(line_width_mm * line_width_mm * 4.0, 1.0)
                and (
                    max_width_mm >= (line_width_mm * 1.45)
                    or median_width_mm >= (line_width_mm * 0.55)
                    or equivalent_diameter_mm >= (line_width_mm * 3.5)
                )
            )
            detail_region_infos.append({
                "component_id": int(component_id),
                "geometry": component_geometry,
                "mask": component_mask,
                "origin_x": float(comp_origin_x),
                "origin_y": float(comp_origin_y),
                "px_per_mm": float(comp_px_per_mm),
                "area_mm2": float(component_area_mm2),
                "max_width_mm": float(max_width_mm),
                "median_width_mm": float(median_width_mm),
                "equivalent_diameter_mm": float(equivalent_diameter_mm),
                "angle_deg": float(region_angle_deg),
                "has_holes": bool(has_holes),
                "hole_driven_detail_fill": bool(hole_driven_detail_fill),
            })
            is_wide_detail_region = bool(
                max_width_mm >= (line_width_mm * 2.0)
                or median_width_mm >= (line_width_mm * 1.4)
                or (max_width_mm >= detail_classification_threshold_width_mm and component_area_mm2 >= detail_classification_threshold_area_mm2)
                or hole_driven_detail_fill
            )

            covered_area_before_mm2 = 0.0
            if existing_fill_area is not None and not getattr(existing_fill_area, "is_empty", True):
                try:
                    covered_area_before_mm2 = float(component_geometry.intersection(existing_fill_area).area)
                except Exception:
                    covered_area_before_mm2 = float(component_geometry.buffer(0).intersection(existing_fill_area.buffer(0)).area)
            coverage_ratio_before = covered_area_before_mm2 / max(component_area_mm2, 1e-9)

            if not is_wide_detail_region:
                detail_regions_classified_thin += 1
                classified_thin_component_ids.add(component_id)
                thin_source_region_infos.append({
                    "component_id": int(component_id),
                    "geometry": component_geometry,
                    "mask": component_mask,
                    "origin_x": float(comp_origin_x),
                    "origin_y": float(comp_origin_y),
                    "px_per_mm": float(comp_px_per_mm),
                    "area_mm2": float(component_area_mm2),
                    "max_width_mm": float(max_width_mm),
                    "median_width_mm": float(median_width_mm),
                    "equivalent_diameter_mm": float(equivalent_diameter_mm),
                    "angle_deg": float(region_angle_deg),
                    "has_holes": bool(has_holes),
                })
                continue

            detail_regions_classified_wide += 1
            if coverage_ratio_before >= 0.90:
                detail_region_covered_area_mm2 += covered_area_before_mm2
                continue

            safe_detail_geometry = _strict_safe_centerline_geometry(component_geometry, inset_mm=pen_radius_mm)
            if safe_detail_geometry is None or getattr(safe_detail_geometry, "is_empty", True):
                detail_regions_classified_thin += 1
                classified_thin_component_ids.add(component_id)
                detail_regions_classified_wide = max(0, detail_regions_classified_wide - 1)
                thin_source_region_infos.append({
                    "component_id": int(component_id),
                    "geometry": component_geometry,
                    "mask": component_mask,
                    "origin_x": float(comp_origin_x),
                    "origin_y": float(comp_origin_y),
                    "px_per_mm": float(comp_px_per_mm),
                    "area_mm2": float(component_area_mm2),
                    "max_width_mm": float(max_width_mm),
                    "median_width_mm": float(median_width_mm),
                    "equivalent_diameter_mm": float(equivalent_diameter_mm),
                    "angle_deg": float(region_angle_deg),
                    "has_holes": bool(has_holes),
                })
                component_debug.append({
                    "component_id": int(component_id),
                    "mode": "detail-wide-collapsed-to-thin",
                    "area_px": int(np.count_nonzero(component_mask > 0)),
                    "estimated_width_mm": float(max_width_mm),
                    "median_width_mm": float(median_width_mm),
                    "detail_region_augmented": False,
                    "safe_centerline_geometry_empty": True,
                })
                continue

            fill_paths, fill_stats = _scanline_fill_paths(
                component_geometry,
                angle_deg=region_angle_deg if math.isfinite(region_angle_deg) else infill_angle_deg,
                spacing_mm=detail_fill_spacing_mm,
                line_width_mm=line_width_mm,
                origin_x=comp_origin_x,
                origin_y=comp_origin_y,
                px_per_mm=comp_px_per_mm,
                component_id=component_id,
                allow_connectors=allow_pen_down_infill_connectors,
                max_overflow_mm=0.0,
                fill_mode_label="detail_serpentine_fill",
                intersection_geometry=safe_detail_geometry,
                endpoint_limit_geometry=safe_detail_geometry,
                centerline_generation_mode="safe-mask",
                safe_centerline_inset_mm=pen_radius_mm,
            )
            component_boundary_paths = _boundary_paths_for_geometry(
                component_geometry,
                simplify_tolerance_mm=simplify_tolerance_mm,
                line_width_mm=line_width_mm,
                centerline_offset_mm=pen_radius_mm,
            )
            component_outline_footprint = _paths_footprint_union(component_boundary_paths, pen_radius_mm=pen_radius_mm)
            component_endpoint_limit = component_geometry.union(component_outline_footprint) if component_outline_footprint is not None and not getattr(component_outline_footprint, "is_empty", True) else component_geometry
            unclamped_fill_paths = [_clone_toolpath(path) for path in fill_paths]
            fill_paths, endpoint_clamp_stats = _clamp_infill_endpoints_to_outline_limit(
                fill_paths,
                allowed_geom=safe_detail_geometry,
                pen_radius_mm=pen_radius_mm,
                max_retract_mm=pen_radius_mm,
                precision_mm=0.02,
            )
            if not fill_paths:
                detail_regions_classified_thin += 1
                classified_thin_component_ids.add(component_id)
                detail_regions_classified_wide = max(0, detail_regions_classified_wide - 1)
                thin_source_region_infos.append({
                    "component_id": int(component_id),
                    "geometry": component_geometry,
                    "mask": component_mask,
                    "origin_x": float(comp_origin_x),
                    "origin_y": float(comp_origin_y),
                    "px_per_mm": float(comp_px_per_mm),
                    "area_mm2": float(component_area_mm2),
                    "max_width_mm": float(max_width_mm),
                    "median_width_mm": float(median_width_mm),
                    "equivalent_diameter_mm": float(equivalent_diameter_mm),
                    "angle_deg": float(region_angle_deg),
                    "has_holes": bool(has_holes),
                })
                continue
            for path in fill_paths:
                path.source = "detail_serpentine_fill"
                path.kind = "fill-infill"
                path.metadata = {
                    **(path.metadata or {}),
                    "fill_mode": "detail_serpentine_fill",
                    "fill_strategy": "DETAIL_SERPENTINE_FILL",
                    "source_region_id": f"detail_region_{component_id:03d}",
                    "detail_region_component_id": int(component_id),
                    "detail_region_classification": "wide",
                    "detail_region_max_width_mm": float(max_width_mm),
                    "detail_region_median_width_mm": float(median_width_mm),
                    "detail_region_area_mm2": float(component_area_mm2),
                    "detail_region_has_holes": bool(has_holes),
                    "detail_region_hole_driven_fill": bool(hole_driven_detail_fill),
                }
            all_paths.extend(fill_paths)
            pre_endpoint_clamp_paths.extend(unclamped_fill_paths)
            detail_regions_serpentine_filled += 1
            arsenal_detail_serpentine_paths_generated += len(fill_paths)
            component_debug.append({
                "component_id": int(component_id),
                "mode": "detail-wide",
                "area_px": int(np.count_nonzero(component_mask > 0)),
                "estimated_width_mm": float(max_width_mm),
                "median_width_mm": float(median_width_mm),
                "segment_count": int(fill_stats.get("segment_count", 0)),
                "row_count": int(fill_stats.get("row_count", 0)),
                "path_count": len(fill_paths),
                "endpoint_extension_mm": float(fill_stats.get("endpoint_extension_mm", line_width_mm * 0.5)),
                "endpoint_extensions_added": int(fill_stats.get("endpoint_extensions_added", 0)),
                "endpoint_extensions_clipped": int(fill_stats.get("endpoint_extensions_clipped", 0)),
                "endpoint_clamp_mode": str(endpoint_clamp_stats.get("endpoint_clamp_mode", "postprocess_only")),
                "endpoints_checked": int(endpoint_clamp_stats.get("endpoints_checked", 0)),
                "endpoints_clamped": int(endpoint_clamp_stats.get("endpoints_clamped", 0)),
                "max_endpoint_retract_mm": float(endpoint_clamp_stats.get("max_endpoint_retract_mm", 0.0)),
                "centerline_generation_mode": str(fill_stats.get("centerline_generation_mode", "raw-mask")),
                "safe_centerline_inset_mm": float(fill_stats.get("safe_centerline_inset_mm", 0.0)),
                "safe_centerline_geometry_empty": False,
                "infill_beyond_outline_before_mm2": float(_infill_beyond_outline_area_mm2(unclamped_fill_paths, allowed_geom=component_endpoint_limit, pen_radius_mm=pen_radius_mm)),
                "infill_beyond_outline_after_mm2": float(_infill_beyond_outline_area_mm2(fill_paths, allowed_geom=component_endpoint_limit, pen_radius_mm=pen_radius_mm)),
                "detail_region_augmented": True,
                "has_holes": bool(has_holes),
                "hole_driven_detail_fill": bool(hole_driven_detail_fill),
            })
            existing_fill_area = _paths_footprint_union(
                [path for path in all_paths if path.kind == "fill-infill"],
                pen_radius_mm=pen_radius_mm,
            )
            covered_area_after_mm2 = 0.0
            if existing_fill_area is not None and not getattr(existing_fill_area, "is_empty", True):
                try:
                    covered_area_after_mm2 = float(component_geometry.intersection(existing_fill_area).area)
                except Exception:
                    covered_area_after_mm2 = float(component_geometry.buffer(0).intersection(existing_fill_area.buffer(0)).area)
            detail_region_covered_area_mm2 += max(covered_area_before_mm2, covered_area_after_mm2)

        final_path_count_before_thin_centerlines = int(len(all_paths))
        source_thin_centerlines, thin_source_stats, thin_source_candidate_rows = _source_thin_region_centerline_pass(
            thin_region_infos=detail_region_infos,
            current_paths=all_paths,
            line_width_mm=line_width_mm,
            simplify_tolerance_mm=simplify_tolerance_mm,
        )
        if source_thin_centerlines:
            all_paths.extend(source_thin_centerlines)
            pre_endpoint_clamp_paths.extend(_clone_toolpath(path) for path in source_thin_centerlines)
            for path in source_thin_centerlines:
                component_id = int((path.metadata or {}).get("detail_component_id", 0) or 0)
                if component_id > 0:
                    classified_thin_component_ids.add(component_id)
            arsenal_detail_centerline_paths_generated += len(source_thin_centerlines)
        final_path_count_after_thin_centerlines = int(len(all_paths))
        repair_candidate_rows.extend(
            {
                "candidate_type": "thin_source_region_centerline",
                "component_id": int(row.get("component_id", 0) or 0),
                "score": float(row.get("score", 0.0) or 0.0),
                "accepted": bool(
                    str(row.get("classification", "")) != "reject-useless-or-overflowing"
                    and float(row.get("new_expected_pixels", 0.0) or 0.0) > max(12.0, float(row.get("redundant_pixels", 0.0) or 0.0))
                    and float(row.get("overflow_pixels", 0.0) or 0.0) <= max(1.0, float(row.get("new_expected_pixels", 0.0) or 0.0)) * 0.35
                ),
                "candidate": row.get("candidate"),
                "stats": {
                    "covered_missed_px": float(row.get("new_expected_pixels", 0.0) or 0.0),
                    "overflow_px": float(row.get("overflow_pixels", 0.0) or 0.0),
                    "added_length_mm": float(row.get("path_length_mm", 0.0) or 0.0),
                    "pen_lift_penalty": 1.0,
                    "coverage_percent": float(row.get("length_coverage_ratio", 0.0) or 0.0) * 100.0,
                },
            }
            for row in thin_source_candidate_rows
        )

        detail_repair_pass_enabled = bool(detail_region_infos)
        if detail_repair_pass_enabled:
            local_coverage_validation_enabled = True
            total_fillable_area_mm2 = 0.0
            total_covered_before_mm2 = 0.0
            total_covered_after_mm2 = 0.0
            pen_radius_px_by_region: dict[int, int] = {}

            for info in detail_region_infos:
                region_component_id = int(info["component_id"])
                region_mask = np.asarray(info["mask"]) > 0
                region_px_per_mm = float(info["px_per_mm"])
                region_pen_radius_px = max(1, int(round(pen_radius_mm * region_px_per_mm)))
                pen_radius_px_by_region[region_component_id] = region_pen_radius_px
                region_geometry = info["geometry"]
                region_outline_paths = _boundary_paths_for_geometry(
                    region_geometry,
                    simplify_tolerance_mm=simplify_tolerance_mm,
                    line_width_mm=line_width_mm,
                    centerline_offset_mm=full_coverage_outline_offset_mm,
                )
                region_current_to_source = (
                    region_px_per_mm,
                    0.0,
                    0.0,
                    region_px_per_mm,
                    -float(info["origin_x"]) * region_px_per_mm,
                    -float(info["origin_y"]) * region_px_per_mm,
                )
                painted_before = _path_points_to_mask(
                    [*_detail_coverage_paths(all_paths), *region_outline_paths],
                    shape=region_mask.shape,
                    current_to_source_matrix=region_current_to_source,
                    pen_radius_px=region_pen_radius_px,
                )
                metrics_before_region = _painted_metrics(
                    target_mask=region_mask,
                    painted_mask=painted_before,
                    allowed_mask=region_mask,
                    px_per_mm=region_px_per_mm,
                )
                blob_rows_before = _connected_mask_blob_rows(
                    ((region_mask > 0) & ~(painted_before > 0)).astype(np.uint8),
                    px_per_mm=region_px_per_mm,
                    min_blob_area_mm2=min_repair_blob_area_mm2,
                )
                blob_summary_before = _blob_summary(blob_rows_before)
                visible_blob_count_before = sum(
                    1 for blob in blob_rows_before
                    if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > largest_allowed_missed_blob_equivalent_diameter_mm
                )
                coverage_before_percent = float(metrics_before_region["coverage_percent"])
                area_mm2 = float(info["area_mm2"])
                max_width_mm = float(info["max_width_mm"])
                equivalent_diameter_mm = float(info["equivalent_diameter_mm"])
                fillable_region = bool(
                    max_width_mm >= line_width_mm
                    or equivalent_diameter_mm >= line_width_mm
                )
                row: dict[str, Any] = {
                    "region_id": f"detail_region_{region_component_id:03d}",
                    "component_id": int(region_component_id),
                    "area_mm2": float(area_mm2),
                    "max_width_mm": float(max_width_mm),
                    "median_width_mm": float(info["median_width_mm"]),
                    "equivalent_diameter_mm": float(equivalent_diameter_mm),
                    "strategy_used": [],
                    "coverage_before_percent": float(coverage_before_percent),
                    "coverage_after_percent": float(coverage_before_percent),
                    "coverage_percent": float(coverage_before_percent),
                    "missed_area_before_mm2": float(metrics_before_region["missed_area_mm2"]),
                    "missed_area_after_mm2": float(metrics_before_region["missed_area_mm2"]),
                    "missed_area_mm2": float(metrics_before_region["missed_area_mm2"]),
                    "missed_blob_count": int(visible_blob_count_before),
                    "missed_blob_count_before_repair": int(visible_blob_count_before),
                    "missed_blob_count_after_repair": int(visible_blob_count_before),
                    "largest_missed_blob_mm2": float(blob_summary_before["largest_missed_blob_area_mm2"]),
                    "largest_missed_blob_area_mm2_before": float(blob_summary_before["largest_missed_blob_area_mm2"]),
                    "largest_missed_blob_area_mm2_after": float(blob_summary_before["largest_missed_blob_area_mm2"]),
                    "largest_missed_blob_diameter_before_mm": float(blob_summary_before["largest_missed_blob_equivalent_diameter_mm"]),
                    "largest_missed_blob_diameter_after_mm": float(blob_summary_before["largest_missed_blob_equivalent_diameter_mm"]),
                    "fillable": bool(fillable_region),
                    "failure_reason_if_still_below_90": "",
                    "repair_strokes_added": 0,
                    "detail_repair_iterations": 0,
                }
                detail_region_rows.append(row)
                regions_total += 1
                missed_blob_count_before_repair += int(visible_blob_count_before)
                largest_missed_blob_area_mm2_before = max(largest_missed_blob_area_mm2_before, float(blob_summary_before["largest_missed_blob_area_mm2"]))
                if not fillable_region:
                    row["failure_reason_if_still_below_90"] = "non_fillable_below_pen_diameter"
                    continue
                detail_fillable_regions_total += 1
                total_fillable_area_mm2 += area_mm2
                total_covered_before_mm2 += float(metrics_before_region["painted_inside_area_mm2"])
                total_covered_after_mm2 += float(metrics_before_region["painted_inside_area_mm2"])
                if (
                    coverage_before_percent < required_detail_coverage_percent
                    or float(blob_summary_before["largest_missed_blob_equivalent_diameter_mm"]) > largest_allowed_missed_blob_equivalent_diameter_mm
                ):
                    detail_fillable_regions_failing_before_repair += 1

            if total_fillable_area_mm2 > 0.0:
                detail_coverage_before_repair_percent = 100.0 * total_covered_before_mm2 / total_fillable_area_mm2
                detail_coverage_after_repair_percent = detail_coverage_before_repair_percent

            for info, row in zip(detail_region_infos, detail_region_rows):
                if not row.get("fillable", False):
                    continue
                region_component_id = int(info["component_id"])
                region_mask = np.asarray(info["mask"]) > 0
                region_px_per_mm = float(info["px_per_mm"])
                region_geometry = info["geometry"]
                region_outline_paths = _boundary_paths_for_geometry(
                    region_geometry,
                    simplify_tolerance_mm=simplify_tolerance_mm,
                    line_width_mm=line_width_mm,
                    centerline_offset_mm=full_coverage_outline_offset_mm,
                )
                region_outline_footprint = _paths_footprint_union(region_outline_paths, pen_radius_mm=pen_radius_mm)
                region_allowed_geom = (
                    region_geometry.union(region_outline_footprint)
                    if region_outline_footprint is not None and not getattr(region_outline_footprint, "is_empty", True)
                    else region_geometry
                )
                region_pen_radius_px = pen_radius_px_by_region[region_component_id]
                region_current_to_source = (
                    region_px_per_mm,
                    0.0,
                    0.0,
                    region_px_per_mm,
                    -float(info["origin_x"]) * region_px_per_mm,
                    -float(info["origin_y"]) * region_px_per_mm,
                )
                current_detail_paths = [*_detail_coverage_paths(all_paths), *region_outline_paths]
                current_painted = _path_points_to_mask(
                    current_detail_paths,
                    shape=region_mask.shape,
                    current_to_source_matrix=region_current_to_source,
                    pen_radius_px=region_pen_radius_px,
                )
                current_metrics = _painted_metrics(
                    target_mask=region_mask,
                    painted_mask=current_painted,
                    allowed_mask=region_mask,
                    px_per_mm=region_px_per_mm,
                )
                current_blob_rows = _connected_mask_blob_rows(
                    ((region_mask > 0) & ~(current_painted > 0)).astype(np.uint8),
                    px_per_mm=region_px_per_mm,
                    min_blob_area_mm2=min_repair_blob_area_mm2,
                )
                current_blob_summary = _blob_summary(current_blob_rows)
                if (
                    float(current_metrics["coverage_percent"]) >= required_detail_coverage_percent
                    and float(current_blob_summary["largest_missed_blob_equivalent_diameter_mm"]) <= largest_allowed_missed_blob_equivalent_diameter_mm
                ):
                    row["coverage_after_percent"] = float(current_metrics["coverage_percent"])
                    row["coverage_percent"] = float(current_metrics["coverage_percent"])
                    row["missed_area_after_mm2"] = float(current_metrics["missed_area_mm2"])
                    row["missed_area_mm2"] = float(current_metrics["missed_area_mm2"])
                    row["missed_blob_count_after_repair"] = int(sum(
                        1 for blob in current_blob_rows
                        if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > largest_allowed_missed_blob_equivalent_diameter_mm
                    ))
                    row["largest_missed_blob_area_mm2_after"] = float(current_blob_summary["largest_missed_blob_area_mm2"])
                    row["largest_missed_blob_diameter_after_mm"] = float(current_blob_summary["largest_missed_blob_equivalent_diameter_mm"])
                    continue

                detail_repair_regions_processed += 1
                region_strategy_used: list[str] = []
                no_improvement_reason = "no_valid_repair_candidate"
                previous_covered_mm2 = float(current_metrics["painted_inside_area_mm2"])

                for _iteration in range(max_detail_repair_iterations_per_region):
                    detail_repair_iterations_total += 1
                    row["detail_repair_iterations"] = int(row.get("detail_repair_iterations", 0)) + 1
                    missed_mask_region = ((region_mask > 0) & ~(current_painted > 0)).astype(np.uint8)
                    blob_rows = _connected_mask_blob_rows(
                        missed_mask_region,
                        px_per_mm=region_px_per_mm,
                        min_blob_area_mm2=min_repair_blob_area_mm2,
                    )
                    if not blob_rows:
                        no_improvement_reason = "only_subminimum_missed_blobs_remain"
                        break

                    best_choice: dict[str, Any] | None = None
                    for blob_row in blob_rows:
                        blob_id = int(blob_row.get("blob_id", 0))
                        blob_mask = np.asarray(blob_row["mask"], dtype=np.uint8)
                        blob_geometry = _component_mask_to_geometry(
                            blob_mask * 255,
                            origin_x=float(info["origin_x"]),
                            origin_y=float(info["origin_y"]),
                            px_per_mm=region_px_per_mm,
                        )
                        candidate_groups = defaultdict(list)
                        for strategy_key, candidate in _detail_repair_candidates_for_blob(
                            blob_mask,
                            blob_geometry=blob_geometry,
                            origin_x=float(info["origin_x"]),
                            origin_y=float(info["origin_y"]),
                            px_per_mm=region_px_per_mm,
                            line_width_mm=line_width_mm,
                            simplify_tolerance_mm=simplify_tolerance_mm,
                            detail_fill_spacing_mm=detail_fill_spacing_mm,
                            region_component_id=region_component_id,
                            region_area_mm2=float(info["area_mm2"]),
                            region_max_width_mm=float(info["max_width_mm"]),
                            region_equivalent_diameter_mm=float(info["equivalent_diameter_mm"]),
                            main_angle_deg=float(info["angle_deg"]),
                        ):
                            repair_candidates_generated += 1
                            candidate_groups[strategy_key].append(candidate)

                        for strategy_key in [
                            "serpentine",
                            "contour_offset",
                            "parallel_stroke",
                            "perpendicular_stroke",
                            "centerline_fallback",
                        ]:
                            strategy_candidates = candidate_groups.get(strategy_key, [])
                            if not strategy_candidates:
                                continue
                            valid_rows: list[dict[str, Any]] = []
                            for candidate in strategy_candidates:
                                clipped_to_region = False
                                while True:
                                    candidate_mask = _path_points_to_mask(
                                        [candidate],
                                        shape=region_mask.shape,
                                        current_to_source_matrix=region_current_to_source,
                                        pen_radius_px=region_pen_radius_px,
                                    )
                                    trial_painted = np.maximum(current_painted, candidate_mask)
                                    trial_metrics = _painted_metrics(
                                        target_mask=region_mask,
                                        painted_mask=trial_painted,
                                        allowed_mask=region_mask,
                                        px_per_mm=region_px_per_mm,
                                    )
                                    candidate_footprint = _paths_footprint_union([candidate], pen_radius_mm=pen_radius_mm)
                                    newly_covered_target_area_mm2 = max(
                                        0.0,
                                        float(trial_metrics["painted_inside_area_mm2"]) - float(current_metrics["painted_inside_area_mm2"]),
                                    )
                                    outside_region_overflow_mm2 = (
                                        float(candidate_footprint.difference(region_allowed_geom).area)
                                        if candidate_footprint is not None and not getattr(candidate_footprint, "is_empty", True)
                                        else 0.0
                                    )
                                    candidate_inside_px = int(np.count_nonzero((candidate_mask > 0) & region_mask))
                                    already_covered_px = int(np.count_nonzero((candidate_mask > 0) & (current_painted > 0) & region_mask))
                                    already_covered_ratio = float(already_covered_px / max(1, candidate_inside_px))
                                    rejection_reason = ""
                                    if newly_covered_target_area_mm2 <= 0.005:
                                        rejection_reason = "new_coverage_below_threshold"
                                        repair_candidates_rejected += 1
                                        repair_rejection_reason_counts[rejection_reason] += 1
                                        break
                                    if outside_region_overflow_mm2 > outside_region_overflow_tolerance_mm2 and not clipped_to_region:
                                        candidate_line = LineString([(point.x, point.y) for point in candidate.points])
                                        clipped_geom = candidate_line.intersection(region_allowed_geom)
                                        clipped_parts = [
                                            part
                                            for part in pipeline_core.extract_lines(clipped_geom)
                                            if not getattr(part, "is_empty", True) and float(part.length) > 1e-9
                                        ]
                                        if clipped_parts:
                                            best_part = max(clipped_parts, key=lambda part: float(part.length))
                                            clipped_points = pipeline_core.simplify_segment_points(
                                                [Point(float(x), float(y)) for x, y in best_part.coords],
                                                simplify_tolerance_mm,
                                                False,
                                            )
                                            if len(clipped_points) >= 2:
                                                candidate = Toolpath(
                                                    points=clipped_points,
                                                    kind=candidate.kind,
                                                    closed=False,
                                                    source=candidate.source,
                                                    region_id=candidate.region_id,
                                                    path_id=candidate.path_id,
                                                    metadata=dict(candidate.metadata or {}),
                                                )
                                                clipped_to_region = True
                                                continue
                                        rejection_reason = "outside_selected_mask_overflow"
                                        repair_candidates_rejected += 1
                                        repair_rejection_reason_counts[rejection_reason] += 1
                                        break
                                    if outside_region_overflow_mm2 > outside_region_overflow_tolerance_mm2:
                                        rejection_reason = "outside_selected_mask_overflow"
                                        repair_candidates_rejected += 1
                                        repair_rejection_reason_counts[rejection_reason] += 1
                                        break
                                    if already_covered_ratio > 0.95 and newly_covered_target_area_mm2 < 0.01:
                                        rejection_reason = "already_covered_ratio_too_high"
                                        repair_candidates_rejected += 1
                                        repair_rejection_reason_counts[rejection_reason] += 1
                                        break
                                    break
                                if rejection_reason:
                                    continue
                                valid_rows.append({
                                    "blob_id": int(blob_id),
                                    "strategy_key": strategy_key,
                                    "candidate": candidate,
                                    "newly_covered_target_area_mm2": float(newly_covered_target_area_mm2),
                                    "outside_region_overflow_mm2": float(outside_region_overflow_mm2),
                                    "already_covered_ratio": float(already_covered_ratio),
                                    "trial_painted": trial_painted,
                                    "trial_metrics": trial_metrics,
                                    "candidate_length_mm": float(pipeline_core.segment_length(candidate.points)),
                                })
                            if valid_rows:
                                valid_rows.sort(
                                    key=lambda item: (
                                        float(item["newly_covered_target_area_mm2"]),
                                        -float(item["outside_region_overflow_mm2"]),
                                        -float(item["candidate_length_mm"]),
                                    ),
                                    reverse=True,
                                )
                                top_row = valid_rows[0]
                                if best_choice is None or float(top_row["newly_covered_target_area_mm2"]) > float(best_choice["newly_covered_target_area_mm2"]):
                                    best_choice = top_row
                                break

                    if best_choice is None:
                        no_improvement_reason = "no_valid_repair_candidate"
                        break

                    legacy_preview_only_repair_kind_detected = legacy_preview_only_repair_kind_detected or str(best_choice["candidate"].kind) in {"coverage_centerline", "coverage_offset_line", "coverage_rectilinear", "coverage_contour"}
                    best_candidate = _export_repair_toolpath(
                        best_choice["candidate"],
                        strategy_key=str(best_choice["strategy_key"]),
                        blob_id=int(best_choice.get("blob_id", 0)) or None,
                    )
                    all_paths.append(best_candidate)
                    pre_endpoint_clamp_paths.append(_clone_toolpath(best_candidate))
                    accepted_repairs.append(best_candidate)
                    repair_candidates_accepted += 1
                    detail_repair_strokes_added += 1
                    row["repair_strokes_added"] = int(row.get("repair_strokes_added", 0)) + 1
                    region_strategy_used.append(str(best_choice["strategy_key"]))
                    current_painted = np.asarray(best_choice["trial_painted"])
                    current_metrics = dict(best_choice["trial_metrics"])
                    detail_repair_new_coverage_mm2 += float(best_choice["newly_covered_target_area_mm2"])
                    detail_repair_outside_overflow_mm2 = max(
                        float(detail_repair_outside_overflow_mm2),
                        float(best_choice["outside_region_overflow_mm2"]),
                    )
                    current_blob_rows = _connected_mask_blob_rows(
                        ((region_mask > 0) & ~(current_painted > 0)).astype(np.uint8),
                        px_per_mm=region_px_per_mm,
                        min_blob_area_mm2=min_repair_blob_area_mm2,
                    )
                    current_blob_summary = _blob_summary(current_blob_rows)
                    if (
                        float(current_metrics["coverage_percent"]) >= required_detail_coverage_percent
                        and float(current_blob_summary["largest_missed_blob_equivalent_diameter_mm"]) <= largest_allowed_missed_blob_equivalent_diameter_mm
                    ):
                        no_improvement_reason = ""
                        break
                    if float(current_metrics["painted_inside_area_mm2"]) <= previous_covered_mm2 + 1e-9:
                        no_improvement_reason = "no_improvement_from_repair_candidates"
                        break
                    previous_covered_mm2 = float(current_metrics["painted_inside_area_mm2"])

                row["strategy_used"] = region_strategy_used
                row["coverage_after_percent"] = float(current_metrics["coverage_percent"])
                row["coverage_percent"] = float(current_metrics["coverage_percent"])
                row["missed_area_after_mm2"] = float(current_metrics["missed_area_mm2"])
                row["missed_area_mm2"] = float(current_metrics["missed_area_mm2"])
                blob_rows_after = _connected_mask_blob_rows(
                    ((region_mask > 0) & ~(current_painted > 0)).astype(np.uint8),
                    px_per_mm=region_px_per_mm,
                    min_blob_area_mm2=min_repair_blob_area_mm2,
                )
                blob_summary_after = _blob_summary(blob_rows_after)
                visible_blob_count_after = sum(
                    1 for blob in blob_rows_after
                    if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > largest_allowed_missed_blob_equivalent_diameter_mm
                )
                row["missed_blob_count"] = int(visible_blob_count_after)
                row["missed_blob_count_after_repair"] = int(visible_blob_count_after)
                row["largest_missed_blob_mm2"] = float(blob_summary_after["largest_missed_blob_area_mm2"])
                row["largest_missed_blob_area_mm2_after"] = float(blob_summary_after["largest_missed_blob_area_mm2"])
                row["largest_missed_blob_diameter_after_mm"] = float(blob_summary_after["largest_missed_blob_equivalent_diameter_mm"])
                if (
                    float(current_metrics["coverage_percent"]) < required_detail_coverage_percent
                    or float(blob_summary_after["largest_missed_blob_equivalent_diameter_mm"]) > largest_allowed_missed_blob_equivalent_diameter_mm
                ):
                    if row["detail_repair_iterations"] >= max_detail_repair_iterations_per_region and not no_improvement_reason:
                        no_improvement_reason = "max_iterations_reached"
                    row["failure_reason_if_still_below_90"] = no_improvement_reason or "coverage_below_threshold_after_repair"

            total_covered_after_mm2 = 0.0
            detail_fillable_regions_failing_after_repair = 0
            detail_repair_remaining_missed_area_mm2 = 0.0
            detail_regions_still_below_90 = []
            missed_blob_count_after_repair = 0
            largest_missed_blob_area_mm2_after = 0.0
            largest_missed_blob_equivalent_diameter_mm_after = 0.0
            for info, row in zip(detail_region_infos, detail_region_rows):
                if not row.get("fillable", False):
                    continue
                region_component_id = int(info["component_id"])
                region_mask = np.asarray(info["mask"]) > 0
                region_px_per_mm = float(info["px_per_mm"])
                region_geometry = info["geometry"]
                region_outline_paths = _boundary_paths_for_geometry(
                    region_geometry,
                    simplify_tolerance_mm=simplify_tolerance_mm,
                    line_width_mm=line_width_mm,
                    centerline_offset_mm=pen_radius_mm,
                )
                region_current_to_source = (
                    region_px_per_mm,
                    0.0,
                    0.0,
                    region_px_per_mm,
                    -float(info["origin_x"]) * region_px_per_mm,
                    -float(info["origin_y"]) * region_px_per_mm,
                )
                final_painted = _path_points_to_mask(
                    [*_detail_coverage_paths(all_paths), *region_outline_paths],
                    shape=region_mask.shape,
                    current_to_source_matrix=region_current_to_source,
                    pen_radius_px=pen_radius_px_by_region[region_component_id],
                )
                final_metrics = _painted_metrics(
                    target_mask=region_mask,
                    painted_mask=final_painted,
                    allowed_mask=region_mask,
                    px_per_mm=region_px_per_mm,
                )
                row["coverage_after_percent"] = float(final_metrics["coverage_percent"])
                row["coverage_percent"] = float(final_metrics["coverage_percent"])
                row["missed_area_after_mm2"] = float(final_metrics["missed_area_mm2"])
                row["missed_area_mm2"] = float(final_metrics["missed_area_mm2"])
                final_blob_rows = _connected_mask_blob_rows(
                    ((region_mask > 0) & ~(final_painted > 0)).astype(np.uint8),
                    px_per_mm=region_px_per_mm,
                    min_blob_area_mm2=min_repair_blob_area_mm2,
                )
                final_blob_summary = _blob_summary(final_blob_rows)
                row["missed_blob_count_after_repair"] = int(final_blob_summary["missed_blob_count"])
                row["largest_missed_blob_area_mm2_after"] = float(final_blob_summary["largest_missed_blob_area_mm2"])
                row["largest_missed_blob_diameter_after_mm"] = float(final_blob_summary["largest_missed_blob_equivalent_diameter_mm"])
                total_covered_after_mm2 += float(final_metrics["painted_inside_area_mm2"])
                detail_repair_remaining_missed_area_mm2 += float(final_metrics["missed_area_mm2"])
                missed_blob_count_after_repair += int(sum(
                    1 for blob in final_blob_rows
                    if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > largest_allowed_missed_blob_equivalent_diameter_mm
                ))
                largest_missed_blob_area_mm2_after = max(largest_missed_blob_area_mm2_after, float(final_blob_summary["largest_missed_blob_area_mm2"]))
                largest_missed_blob_equivalent_diameter_mm_after = max(largest_missed_blob_equivalent_diameter_mm_after, float(final_blob_summary["largest_missed_blob_equivalent_diameter_mm"]))
                if (
                    float(final_metrics["coverage_percent"]) < required_detail_coverage_percent
                    or float(final_blob_summary["largest_missed_blob_equivalent_diameter_mm"]) > largest_allowed_missed_blob_equivalent_diameter_mm
                ):
                    detail_fillable_regions_failing_after_repair += 1
                    detail_regions_still_below_90.append({
                        "region_id": row["region_id"],
                        "coverage_after_percent": float(final_metrics["coverage_percent"]),
                        "missed_area_after_mm2": float(final_metrics["missed_area_mm2"]),
                        "largest_missed_blob_equivalent_diameter_mm": float(final_blob_summary["largest_missed_blob_equivalent_diameter_mm"]),
                        "failure_reason": row.get("failure_reason_if_still_below_90", "") or "coverage_below_threshold_after_repair",
                    })

            if total_fillable_area_mm2 > 0.0:
                detail_coverage_after_repair_percent = 100.0 * total_covered_after_mm2 / total_fillable_area_mm2
                detail_region_total_area_mm2 = total_fillable_area_mm2
                detail_region_covered_area_mm2 = total_covered_after_mm2
                region_coverage_min_percent = min((float(row.get("coverage_after_percent", 100.0)) for row in detail_region_rows if bool(row.get("fillable", False))), default=100.0)

    detail_clip_region = None
    if enable_fill and printable_geometry is not None and not getattr(printable_geometry, "is_empty", True):
        detail_clip_region = printable_geometry.buffer(-(line_width_mm * 0.5), join_style=1)
        if detail_clip_region is None or getattr(detail_clip_region, "is_empty", True):
            detail_clip_region = printable_geometry.buffer(-(line_width_mm * 0.25), join_style=1)
        if detail_clip_region is None or getattr(detail_clip_region, "is_empty", True):
            detail_clip_region = printable_geometry

    for index, segment in enumerate(getattr(bundle, "detail_segments", [])):
        simplified = pipeline_core.simplify_segment_points(list(segment.points), detail_tolerance_mm, bool(segment.closed))
        if len(simplified) < 2:
            continue
        metadata = {
            "simplify_tolerance_mm": float(detail_tolerance_mm),
            "pen_width_mm": float(line_width_mm),
            "source_region_id": f"detail_{index + 1:03d}",
            "expected_relation_to_fill": "detail_overlay",
            "coordinate_space_at_creation": "surface_mm",
            "coordinate_space_before_offset": "surface_mm",
            "offset_space": "none",
            "coordinate_space_before_simplify": "surface_mm",
            "simplify_space": "surface_mm",
            "detail_centerline_clipped_to_printable_offset": bool(detail_clip_region is not None),
        }
        if detail_clip_region is None:
            path = Toolpath(points=simplified, kind="detail-trace", closed=bool(segment.closed), source="detail_trace", metadata=metadata)
            detail_paths.append(path)
            continue
        line = LineString([(point.x, point.y) for point in simplified])
        clipped = line.intersection(detail_clip_region)
        for part in pipeline_core.extract_lines(clipped):
            clipped_points = pipeline_core.simplify_segment_points([Point(float(x), float(y)) for x, y in part.coords], detail_tolerance_mm, False)
            if len(clipped_points) < 2:
                continue
            detail_paths.append(Toolpath(points=clipped_points, kind="detail-trace", closed=False, source="detail_trace", metadata=metadata))

    detail_filter_stats: dict[str, Any] = {
        "detail_filter_mode": pipeline_core.DETAIL_FILTER_MODE,
        "detail_paths_generated_raw": 0,
        "detail_paths_kept": 0,
        "detail_paths_dropped": 0,
        "detail_drop_reasons": {},
        "detail_paths_dropped_as_travel_or_debug": 0,
        "detail_paths_dropped_as_redundant_overlap": 0,
        "detail_new_coverage_area_mm2_total": 0.0,
        "detail_already_covered_ratio_avg": 0.0,
        "detail_source_whitelist_enforced": True,
        "travel_geometry_allowed_as_detail": False,
        "infill_geometry_changed": False,
        "outline_geometry_changed": False,
        "detail_dropped_path_records": [],
    }
    printable_components: list[Any] = []
    use_source_geometry_outlines = len(component_ids) < len(source_printable_parts)
    if detail_paths:
        outline_paths_for_detail = _actual_outline_centerline_paths(
            source_outline_paths if use_source_geometry_outlines else all_paths
        )
        existing_painted_area = _paths_footprint_union(
            [*([path for path in all_paths if path.kind in {"fill-infill", "fill-infill-travel"}]), *outline_paths_for_detail],
            pen_radius_mm=pen_radius_mm,
        )
        residual_target = printable_geometry
        if existing_painted_area is not None and not getattr(existing_painted_area, "is_empty", True):
            try:
                residual_target = printable_geometry.difference(existing_painted_area)
            except Exception:
                residual_target = printable_geometry.buffer(0).difference(existing_painted_area.buffer(0))
        if residual_target is not None and not getattr(residual_target, "is_empty", True):
            try:
                residual_target = residual_target.buffer(0)
            except Exception:
                pass
        printable_components = []
        if residual_target is not None and not getattr(residual_target, "is_empty", True):
            for geom in _geometry_parts(residual_target):
                if geom is None or getattr(geom, "is_empty", True):
                    continue
                min_x, min_y, max_x, max_y = geom.bounds
                local_width = min(float(max_x - min_x), float(max_y - min_y))
                if local_width > (line_width_mm * 1.6):
                    continue
                printable_components.append(geom)
        printable_component_boundaries = [component.boundary for component in printable_components]

        def _candidate_component_index(path: Toolpath) -> int | None:
            if len(path.points) < 2:
                return None
            shp = LineString([(point.x, point.y) for point in path.points])
            if shp.is_empty:
                return None
            best_idx: int | None = None
            best_overlap = 0.0
            for idx, component in enumerate(printable_components):
                try:
                    overlap = float(shp.intersection(component).length)
                except Exception:
                    overlap = 0.0
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_idx = idx
            return best_idx

        def _candidate_centeredness(path: Toolpath, component_idx: int | None) -> float:
            if component_idx is None or component_idx < 0 or component_idx >= len(printable_component_boundaries):
                return 0.0
            boundary = printable_component_boundaries[component_idx]
            if boundary is None or boundary.is_empty or len(path.points) < 2:
                return 0.0
            samples = path.points
            if len(samples) > 16:
                step = max(1, int(len(samples) / 16))
                samples = [samples[index] for index in range(0, len(samples), step)]
            distances: list[float] = []
            for pt in samples:
                try:
                    distances.append(float(ShapelyPoint(pt.x, pt.y).distance(boundary)))
                except Exception:
                    continue
            if not distances:
                return 0.0
            return float(sum(distances) / max(1, len(distances)))

        def _candidate_component_metrics(path: Toolpath, component_idx: int | None) -> dict[str, Any]:
            if component_idx is None or component_idx < 0 or component_idx >= len(printable_components):
                return {}
            component = printable_components[component_idx]
            if component is None or component.is_empty:
                return {}
            min_x, min_y, max_x, max_y = component.bounds
            return {
                "component_id": component_idx + 1,
                "area_mm2": float(component.area),
                "bbox_mm": (float(max_x - min_x), float(max_y - min_y)),
                "estimated_width_mm": float(min(max_x - min_x, max_y - min_y)),
            }
        detail_filter_stats = pipeline_core._filter_detail_trace_candidates_for_export(
            detail_paths,
            target_geometry=printable_geometry,
            existing_painted_area=existing_painted_area,
            line_width_mm=line_width_mm,
            allow_detail_overlap_outline=True,
            validate_detail_with_pen_footprint=True,
            max_detail_overspill_mm=min(0.05, line_width_mm * 0.10, max(0.0, float(os.getenv("MAX_DETAIL_OVERSPILL_MM", "0.05")))),
            max_detail_overspill_area_ratio=max(0.0, float(os.getenv("MAX_DETAIL_OVERSPILL_AREA_RATIO", "0.03"))),
            min_detail_new_coverage_mm2=max(0.0, float(os.getenv("MIN_DETAIL_NEW_COVERAGE_MM2", "0.02"))),
            max_already_covered_ratio=max(0.0, min(1.0, float(os.getenv("MAX_ALREADY_COVERED_RATIO", "0.90")))),
            candidate_component_index_fn=_candidate_component_index,
            candidate_centeredness_fn=_candidate_centeredness,
            candidate_component_metrics_fn=_candidate_component_metrics,
        )
        kept_detail_paths = list(detail_filter_stats["accepted_detail_paths"])
        all_paths.extend(kept_detail_paths)
        pre_endpoint_clamp_paths.extend(_clone_toolpath(path) for path in kept_detail_paths)

    if debug is not None:
        debug["_coverage_target_mask"] = target_mask.copy()
        debug["_coverage_allowed_mask"] = allowed_mask.copy()
        debug["_coverage_current_to_source_matrix"] = tuple(float(value) for value in current_to_source)
        if isinstance(connector_validation.get("mask"), np.ndarray):
            debug["_coverage_preview_source_mask"] = np.asarray(connector_validation.get("mask")).copy()
        if isinstance(connector_validation.get("source_to_current_matrix"), (tuple, list)) and len(connector_validation.get("source_to_current_matrix")) == 6:
            debug["_coverage_preview_source_to_surface_matrix"] = tuple(float(value) for value in connector_validation.get("source_to_current_matrix"))
        debug.update(detail_filter_stats)
        accepted_detail_paths = list(detail_filter_stats.get("accepted_detail_paths", []))
        detail_regions_centerline_traced = 0
        arsenal_detail_centerline_paths_generated = 0
        for path in accepted_detail_paths:
            component_id = (path.metadata or {}).get("detail_component_id")
            if component_id is None:
                continue
            try:
                parsed_component_id = int(component_id)
            except Exception:
                continue
            if parsed_component_id in classified_thin_component_ids:
                detail_regions_centerline_traced += 1
                arsenal_detail_centerline_paths_generated += 1
        dropped_records = list(detail_filter_stats.get("detail_dropped_path_records", []))
        accepted_component_ids = {
            int(path.metadata.get("detail_component_id"))
            for path in accepted_detail_paths
            if path.metadata.get("detail_component_id") is not None
        }
        dropped_component_ids = {
            int(record["component_id"])
            for record in dropped_records
            if record.get("component_id") is not None
        }
        small_detail_drop_reasons: dict[str, int] = {}
        for record in dropped_records:
            component_id = record.get("component_id")
            if component_id is None:
                continue
            reason = str(record.get("drop_reason", "unknown"))
            small_detail_drop_reasons[reason] = int(small_detail_drop_reasons.get(reason, 0)) + 1
        overlap_preserved_count = sum(
            1 for path in accepted_detail_paths if bool(path.metadata.get("detail_overlap_exception_applied", False))
        )
        overlap_rejected_count = sum(
            1
            for record in dropped_records
            if str(record.get("drop_reason", "")) in {"redundant_overlap", "redundant_centerline_overlap", "noisy_loop"}
            and (bool(record.get("was_self_overlapping", False)) or record.get("component_id") is not None)
        )
        debug.update({
            "normal_safe_infill_enabled": True,
            "thin_source_region_pass_enabled": bool(use_source_geometry_outlines),
            "thin_source_region_pass_ran": bool(thin_source_stats.get("thin_source_region_detection_ran", False)),
            "thin_source_region_detection_ran": bool(thin_source_stats.get("thin_source_region_detection_ran", False)),
            "thin_source_region_count": int(thin_source_stats.get("thin_source_region_count", 0)),
            "thin_centerline_candidate_count": int(thin_source_stats.get("thin_centerline_candidate_count", 0)),
            "thin_centerline_accepted_count": int(thin_source_stats.get("thin_centerline_accepted_count", 0)),
            "thin_centerline_total_length_mm": float(thin_source_stats.get("thin_centerline_total_length_mm", 0.0)),
            "thin_centerline_paths_exported": bool(thin_source_stats.get("thin_centerline_paths_exported", False)),
            "thin_skeleton_component_count": int(thin_source_stats.get("thin_skeleton_component_count", 0)),
            "useful_thin_skeleton_length_mm": float(thin_source_stats.get("useful_thin_skeleton_length_mm", 0.0)),
            "accepted_thin_centerline_length_mm": float(thin_source_stats.get("accepted_thin_centerline_length_mm", 0.0)),
            "accepted_thin_centerline_fraction": float(thin_source_stats.get("accepted_thin_centerline_fraction", 0.0)),
            "final_path_count_before_thin_centerlines": int(final_path_count_before_thin_centerlines),
            "final_path_count_after_thin_centerlines": int(final_path_count_after_thin_centerlines),
            "frontend_default_used_thin_centerline_pass": bool(expensive_coverage_repair and thin_source_stats.get("thin_source_region_detection_ran", False)),
            "source_mask_component_count": int(len(source_printable_parts)),
            "small_detail_outline_mode_enabled": True,
            "small_detail_components_detected": int(len(printable_components)),
            "small_detail_components_outlined": int(len(accepted_component_ids)),
            "small_detail_components_dropped": int(len(dropped_component_ids - accepted_component_ids)),
            "small_detail_drop_reasons": small_detail_drop_reasons,
            "self_overlapping_detail_paths_allowed": int(overlap_preserved_count),
            "self_overlapping_detail_paths_rejected": int(overlap_rejected_count),
            "detail_paths_kept_despite_overlap": int(overlap_preserved_count),
            "detail_simplification_tolerance_mm": float(detail_tolerance_mm),
            "arsenal_detail_outline_paths_generated": int(detail_filter_stats.get("detail_paths_generated_raw", 0)),
            "arsenal_detail_outline_paths_dropped": int(detail_filter_stats.get("detail_paths_dropped", 0)),
            "detail_region_count": int(detail_region_count),
            "detail_regions_classified_wide": int(detail_regions_classified_wide),
            "detail_regions_classified_thin": int(detail_regions_classified_thin),
            "detail_regions_serpentine_filled": int(detail_regions_serpentine_filled),
            "detail_regions_centerline_traced": int(detail_regions_centerline_traced),
            "detail_fill_coverage_percent": float((100.0 * detail_region_covered_area_mm2 / max(1e-9, detail_region_total_area_mm2)) if detail_region_total_area_mm2 > 0 else 0.0),
            "detail_missed_area_mm2": float(max(0.0, detail_region_total_area_mm2 - detail_region_covered_area_mm2)),
            "detail_classification_threshold_width_mm": float(detail_classification_threshold_width_mm),
            "detail_classification_threshold_area_mm2": float(detail_classification_threshold_area_mm2),
            "arsenal_detail_serpentine_paths_generated": int(arsenal_detail_serpentine_paths_generated),
            "arsenal_detail_centerline_paths_generated": int(arsenal_detail_centerline_paths_generated),
            "detail_repair_pass_enabled": bool(detail_repair_pass_enabled),
            "required_detail_coverage_percent": float(required_detail_coverage_percent),
            "detail_coverage_before_repair_percent": float(detail_coverage_before_repair_percent),
            "detail_coverage_after_repair_percent": float(detail_coverage_after_repair_percent),
            "detail_fillable_regions_total": int(detail_fillable_regions_total),
            "detail_fillable_regions_failing_before_repair": int(detail_fillable_regions_failing_before_repair),
            "detail_fillable_regions_failing_after_repair": int(detail_fillable_regions_failing_after_repair),
            "detail_repair_regions_processed": int(detail_repair_regions_processed),
            "detail_repair_strokes_added": int(detail_repair_strokes_added),
            "repair_strokes_added": int(detail_repair_strokes_added),
            "detail_repair_iterations_total": int(detail_repair_iterations_total),
            "detail_repair_new_coverage_mm2": float(detail_repair_new_coverage_mm2),
            "detail_repair_remaining_missed_area_mm2": float(detail_repair_remaining_missed_area_mm2),
            "detail_repair_outside_overflow_mm2": float(detail_repair_outside_overflow_mm2),
            "outside_region_overflow_tolerance_mm2": float(outside_region_overflow_tolerance_mm2),
            "detail_regions_still_below_90": detail_regions_still_below_90,
            "detail_region_repair_rows": detail_region_rows,
            "coverage_validation_target": str(coverage_validation_target),
            "fill_allowed_to_overlap_outline": bool(fill_allowed_to_overlap_outline),
            "repair_clipped_against": str(repair_clipped_against),
            "local_coverage_validation_enabled": bool(local_coverage_validation_enabled),
            "global_coverage_percent": float(detail_coverage_after_repair_percent),
            "region_coverage_min_percent": float(region_coverage_min_percent),
            "regions_total": int(regions_total),
            "regions_failing_before_repair": int(detail_fillable_regions_failing_before_repair),
            "regions_failing_after_repair": int(detail_fillable_regions_failing_after_repair),
            "missed_blob_count_before_repair": int(missed_blob_count_before_repair),
            "missed_blob_count_after_repair": int(missed_blob_count_after_repair),
            "largest_missed_blob_area_mm2_before": float(largest_missed_blob_area_mm2_before),
            "largest_missed_blob_area_mm2_after": float(largest_missed_blob_area_mm2_after),
            "largest_missed_blob_equivalent_diameter_mm_before": float(max((float(row.get("largest_missed_blob_diameter_before_mm", 0.0)) for row in detail_region_rows), default=0.0)),
            "largest_missed_blob_equivalent_diameter_mm_after": float(largest_missed_blob_equivalent_diameter_mm_after),
            "largest_allowed_missed_blob_equivalent_diameter_mm": float(largest_allowed_missed_blob_equivalent_diameter_mm),
            "missed_blob_debug_enabled": True,
            "repair_candidates_generated": int(repair_candidates_generated),
            "repair_candidates_accepted": int(repair_candidates_accepted),
            "repair_candidates_rejected": int(repair_candidates_rejected),
            "top_repair_rejection_reasons": dict(repair_rejection_reason_counts.most_common(10)),
            "repair_paths_exported": bool(any(_is_repair_path(path) for path in accepted_repairs)),
            "coverage_preview_gcode_consistent": bool(debug.get("preview_and_gcode_share_same_projected_paths", True)),
            "legacy_preview_only_repair_kind_detected": bool(legacy_preview_only_repair_kind_detected),
            "root_cause_category": (
                "I"
                if (
                    int(missed_blob_count_after_repair) == 0
                    and bool(any(_is_repair_path(path) for path in accepted_repairs))
                    and bool(debug.get("coverage_preview_gcode_consistent", True))
                )
                else ("E" if int(missed_blob_count_after_repair) > 0 and int(repair_candidates_generated) > 0 and int(repair_candidates_accepted) == 0 else "")
            ),
        })
        hole_count = sum(len(poly.interiors) for poly in _geometry_parts(printable_geometry))
        cell_count = max(1, len(component_debug) + hole_count)
        debug["local_cell_count"] = int(cell_count)
        total_segments = sum(int(item.get("segment_count", item.get("skeleton_segment_count", 0))) for item in component_debug)
        debug["average_segments_per_cell"] = float(total_segments / max(1, cell_count))
        debug["coverage_component_summary"] = component_debug
        thin_region_count = int(sum(1 for item in component_debug if item.get("mode") == "thin"))
        endpoint_extension_mm = float(max((float(item.get("endpoint_extension_mm", 0.0)) for item in component_debug), default=line_width_mm * 0.5))
        endpoint_extensions_added = int(sum(int(item.get("endpoint_extensions_added", 0)) for item in component_debug))
        endpoint_extensions_clipped = int(sum(int(item.get("endpoint_extensions_clipped", 0)) for item in component_debug))
        endpoints_checked = int(sum(int(item.get("endpoints_checked", 0)) for item in component_debug))
        endpoints_clamped = int(sum(int(item.get("endpoints_clamped", 0)) for item in component_debug))
        max_endpoint_retract_mm = float(max((float(item.get("max_endpoint_retract_mm", 0.0)) for item in component_debug), default=0.0))
        infill_beyond_outline_before_mm2 = float(sum(float(item.get("infill_beyond_outline_before_mm2", 0.0)) for item in component_debug))
        infill_beyond_outline_after_mm2 = float(sum(float(item.get("infill_beyond_outline_after_mm2", 0.0)) for item in component_debug))
        debug["infill_debug"] = {
                "fill_strategy": str(fill_strategy),
                "infill_path_mode": str(infill_path_mode),
                "allow_pen_down_infill_connectors": bool(allow_pen_down_infill_connectors),
                "infill_overlap_percent": float(infill_overlap_percent),
                "spacing_mm": float(infill_spacing_mm if infill_spacing_mm > 0 else fallback_spacing),
                "pen_width_mm": float(line_width_mm),
                "fill_uses_outline_clearance": bool(fill_uses_outline_clearance),
                "outline_overlap_allowed": bool(outline_overlap_allowed),
                "endpoint_clamp_mode": "postprocess_only",
                "line_generation_changed": False,
                "global_fill_mask_changed": False,
                "endpoint_extension_mm": float(endpoint_extension_mm),
                "endpoint_extensions_added": int(endpoint_extensions_added),
                "endpoint_extensions_clipped": int(endpoint_extensions_clipped),
                "endpoints_checked": int(endpoints_checked),
                "endpoints_clamped": int(endpoints_clamped),
                "max_endpoint_retract_mm": float(max_endpoint_retract_mm),
                "infill_beyond_outline_before_mm2": float(infill_beyond_outline_before_mm2),
                "infill_beyond_outline_after_mm2": float(infill_beyond_outline_after_mm2),
            "diagnostics": {
                "narrower_than_2x_pen_regions": thin_region_count,
                "narrower_than_2x_pen_with_centerline": thin_region_count,
            }
                ,
                "adaptive_fill_counts": {
                    "total_cells": int(cell_count),
                    "rectilinear_cells": int(sum(1 for item in component_debug if item.get("mode") == "wide")),
                    "detail_contour_cells": int(sum(1 for item in component_debug if item.get("mode") == "thin")),
                    "single_stroke_cells": int(thin_region_count),
                    "narrow_cells_detected": int(thin_region_count),
                    "switched_too_few_rows": 0,
                    "switched_connector_ratio": 0,
                    "switched_single_stroke_width": int(thin_region_count),
                    "switched_single_stroke_hatch_quality": 0,
                    "outline_after_fill": bool(outline_after_fill),
                },
        }
        pen_up_travel = 0.0
        pen_lifts = 0
        previous_end: Point | None = None
        for path in all_paths:
            if len(path.points) < 2:
                continue
            if previous_end is not None:
                travel = math.hypot(float(path.points[0].x - previous_end.x), float(path.points[0].y - previous_end.y))
                pen_up_travel += travel
                if travel > 1e-9:
                    pen_lifts += 1
            previous_end = path.points[-1]
        debug["total_pen_up_travel_distance_mm"] = float(pen_up_travel)
        debug["pen_lifts_before_optimization"] = int(pen_lifts)
        debug["infill_connector_diagnostics"] = {
            "total_infill_rows": int(sum(int(item.get("row_count", 0)) for item in component_debug)),
            "accepted_connectors": int(sum(1 for path in all_paths if path.kind == "fill-infill-travel")),
            "rejected_connectors": 0,
            "rejected_raster_mask_sampling": 0,
            "rejected_outside_selected_color": 0,
            "rejection_counts": {},
        }

    pre_outline_paths = [path for path in all_paths if path.kind != "outline"]
    pre_outline_painted = _path_points_to_mask(pre_outline_paths, shape=target_mask.shape, current_to_source_matrix=current_to_source, pen_radius_px=max(1, int(round(line_width_mm * px_per_mm / 2.0))))
    metrics_before = _painted_metrics(target_mask=target_mask, painted_mask=pre_outline_painted, allowed_mask=allowed_mask, px_per_mm=px_per_mm)
    missed_mask = ((target_mask > 0) & ~(pre_outline_painted > 0)).astype(np.uint8) * 255
    if debug is not None:
        debug["coverage_before_outline_percent"] = float(metrics_before.get("coverage_percent", 0.0))
        debug["missed_area_before_outline_mm2"] = float(metrics_before.get("missed_area_mm2", 0.0))
        debug["outside_overflow_mm2"] = float(metrics_before.get("overflow_area_mm2", 0.0))
        debug["largest_missed_blob_before_outline_mm"] = float(_largest_blob_diameter_mm(missed_mask, px_per_mm))
        debug["endpoint_extensions_added"] = int(sum(int(item.get("endpoint_extensions_added", 0)) for item in component_debug))
        debug["endpoint_extensions_clipped"] = int(sum(int(item.get("endpoint_extensions_clipped", 0)) for item in component_debug))
        debug["endpoint_clamp_mode"] = "postprocess_only"
        debug["line_generation_changed"] = False
        debug["global_fill_mask_changed"] = False
        debug["endpoints_checked"] = int(sum(int(item.get("endpoints_checked", 0)) for item in component_debug))
        debug["endpoints_clamped"] = int(sum(int(item.get("endpoints_clamped", 0)) for item in component_debug))
        debug["max_endpoint_retract_mm"] = float(max((float(item.get("max_endpoint_retract_mm", 0.0)) for item in component_debug), default=0.0))
        debug["outline_overlap_allowed"] = bool(outline_overlap_allowed)
        debug["fill_uses_outline_clearance"] = bool(fill_uses_outline_clearance)

    if expensive_coverage_repair:
        current_painted = pre_outline_painted
        allowed = allowed_mask > 0
        if np.count_nonzero(missed_mask) > 0:
            component_count, labels, stats, _ = cv2.connectedComponentsWithStats((missed_mask > 0).astype(np.uint8), connectivity=8)
            missed_ids = [index for index in range(1, int(component_count)) if int(stats[index, cv2.CC_STAT_AREA]) > 0]
            missed_ids.sort(key=lambda idx: int(stats[idx, cv2.CC_STAT_AREA]), reverse=True)
            for missed_id in missed_ids[:60]:
                missed_component_mask = (labels == missed_id).astype(np.uint8)
                ys, xs = np.nonzero(missed_component_mask > 0)
                if xs.size == 0:
                    continue
                area_px = int(stats[missed_id, cv2.CC_STAT_AREA])
                x0, y0, w, h = [int(stats[missed_id, value]) for value in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT)]
                width_mm = float(max(w, h) / px_per_mm)
                angle_deg = _component_fill_angle(missed_component_mask)
                missed_geometry = _component_mask_to_geometry(missed_component_mask * 255, origin_x=origin_x, origin_y=origin_y, px_per_mm=px_per_mm)
                if preferred_centerline_outline_regions and missed_geometry is not None and not getattr(missed_geometry, "is_empty", True):
                    skip_repair_for_centerline_region = False
                    for centerline_region in preferred_centerline_outline_regions:
                        if centerline_region is None or getattr(centerline_region, "is_empty", True):
                            continue
                        try:
                            if not missed_geometry.intersection(centerline_region).is_empty:
                                skip_repair_for_centerline_region = True
                                break
                        except Exception:
                            try:
                                if not missed_geometry.buffer(0).intersection(centerline_region.buffer(0)).is_empty:
                                    skip_repair_for_centerline_region = True
                                    break
                            except Exception:
                                continue
                    if skip_repair_for_centerline_region:
                        repair_rejection_reason_counts["centerline_outline_region_already_represented"] += 1
                        continue
                candidates = _candidate_paths_for_missed_component(
                    missed_component_mask,
                    origin_x=origin_x,
                    origin_y=origin_y,
                    px_per_mm=px_per_mm,
                    line_width_mm=line_width_mm,
                    component_id=missed_id,
                    main_angle_deg=angle_deg,
                    component_geometry=missed_geometry,
                )
                best_candidate: Toolpath | None = None
                best_score = -1e18
                best_stats: dict[str, float] | None = None
                for candidate in candidates:
                    repair_candidates_generated += 1
                    score, stats_row = _score_candidate(
                        current_paths=all_paths,
                        candidate=candidate,
                        target_mask=target_mask,
                        allowed_mask=allowed_mask,
                        current_to_source_matrix=current_to_source,
                        pen_radius_mm=line_width_mm * 0.5,
                        sample_step_mm=max(0.01, min(0.05, line_width_mm * 0.35)),
                    )
                    repair_candidate_rows.append({
                        "component_id": int(missed_id),
                        "candidate_type": str(candidate.metadata.get("repair_candidate_type", "unknown")),
                        "angle_deg": float(candidate.metadata.get("repair_candidate_angle_deg", angle_deg)),
                        "score": float(score),
                        "stats": stats_row,
                        "accepted": False,
                        "candidate": candidate,
                    })
                    if score > best_score:
                        best_score = score
                        best_candidate = candidate
                        best_stats = stats_row
                if best_candidate is not None and best_stats is not None and best_score > -1e9:
                    legacy_preview_only_repair_kind_detected = legacy_preview_only_repair_kind_detected or str(best_candidate.kind) in {"coverage_centerline", "coverage_offset_line", "coverage_rectilinear", "coverage_contour"}
                    exported_best_candidate = _export_repair_toolpath(best_candidate)
                    trial_paths = all_paths + [exported_best_candidate]
                    trial_painted = _path_points_to_mask(trial_paths, shape=target_mask.shape, current_to_source_matrix=current_to_source, pen_radius_px=max(1, int(round(line_width_mm * px_per_mm / 2.0))))
                    trial_missed = np.count_nonzero((target_mask > 0) & ~(trial_painted > 0))
                    trial_overflow = np.count_nonzero((trial_painted > 0) & ~(allowed > 0))
                    if trial_missed < np.count_nonzero((target_mask > 0) & ~(current_painted > 0)) or (trial_missed == 0 and trial_overflow <= np.count_nonzero((current_painted > 0) & ~(allowed > 0))):
                        all_paths = trial_paths
                        pre_endpoint_clamp_paths = pre_endpoint_clamp_paths + [_clone_toolpath(exported_best_candidate)]
                        current_painted = trial_painted
                        accepted_repairs.append(exported_best_candidate)
                        repair_candidates_accepted += 1
                        if trial_missed == 0:
                            break
                    else:
                        repair_candidates_rejected += 1
                        repair_rejection_reason_counts["global_repair_not_improving"] += 1

    optimized_repairs, repair_audit_rows, repair_rebuild_candidate_rows = _optimized_final_repair_paths(
        all_paths=all_paths,
        target_mask=target_mask,
        allowed_mask=allowed_mask,
        printable_geometry=printable_geometry,
        current_to_source=current_to_source,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        line_width_mm=line_width_mm,
        simplify_tolerance_mm=simplify_tolerance_mm,
    )
    non_repair_paths = [path for path in all_paths if not _is_repair_path(path)]
    all_paths = [*non_repair_paths, *optimized_repairs]
    pre_endpoint_clamp_paths = [path for path in pre_endpoint_clamp_paths if not _is_repair_path(path)]
    pre_endpoint_clamp_paths.extend(_clone_toolpath(path) for path in optimized_repairs)
    accepted_repairs = list(optimized_repairs)
    if debug is not None:
        debug["final_repair_audit_rows"] = [
            {key: value for key, value in row.items() if key != "candidate"}
            for row in repair_audit_rows
        ]
        debug["final_repair_rebuild_candidate_rows"] = [
            {key: value for key, value in row.items() if key != "candidate"}
            for row in repair_rebuild_candidate_rows
        ]
        debug["final_repair_audit_summary"] = {
            "audited_repair_count": int(len(repair_audit_rows)),
            "optimized_repair_count": int(len(optimized_repairs)),
            "normal_safe_repair_count": int(sum(1 for path in optimized_repairs if str((path.metadata or {}).get("repair_mode", "")) == "normal-safe-repair")),
            "thin_collapsed_detail_repair_count": int(sum(1 for path in optimized_repairs if str((path.metadata or {}).get("repair_mode", "")) == "thin-collapsed-detail-repair")),
            "rejected_existing_repair_count": int(sum(1 for row in repair_audit_rows if str(row.get("classification", "")) == "reject-useless-or-overflowing")),
        }

    outline_fallback_paths = [
        _promote_outline_fallback_path(path)
        for path in all_paths
        if _is_actual_outline_centerline_path(path) and path.kind != "outline"
    ]
    pre_clamp_outline_fallback_paths = [
        _promote_outline_fallback_path(path)
        for path in pre_endpoint_clamp_paths
        if _is_actual_outline_centerline_path(path) and path.kind != "outline"
    ]
    outline_paths_for_output = (
        list(source_outline_paths) if use_source_geometry_outlines else [path for path in all_paths if path.kind == "outline"]
    ) + outline_fallback_paths
    pre_clamp_outline_paths_for_output = (
        list(source_outline_paths) if use_source_geometry_outlines else [path for path in pre_endpoint_clamp_paths if path.kind == "outline"]
    ) + pre_clamp_outline_fallback_paths
    ordered_fill = pipeline_core.optimize_toolpath_order(
        [path for path in all_paths if path.kind != "outline" and not _is_actual_outline_centerline_path(path)],
        strategy=travel_optimization,
    )
    pre_clamp_ordered_fill = pipeline_core.optimize_toolpath_order(
        [path for path in pre_endpoint_clamp_paths if path.kind != "outline" and not _is_actual_outline_centerline_path(path)],
        strategy=travel_optimization,
    )
    if outline_after_fill:
        final_paths = ordered_fill + list(outline_paths_for_output)
        pre_endpoint_clamp_final_paths = pre_clamp_ordered_fill + [_clone_toolpath(path) for path in pre_clamp_outline_paths_for_output]
    else:
        final_paths = list(outline_paths_for_output) + ordered_fill
        pre_endpoint_clamp_final_paths = [_clone_toolpath(path) for path in pre_clamp_outline_paths_for_output] + pre_clamp_ordered_fill

    final_paths = pipeline_core.merge_connected_toolpaths(final_paths)
    final_paths = pipeline_core.assign_stable_path_ids(final_paths)
    pre_endpoint_clamp_final_paths = pipeline_core.merge_connected_toolpaths(pre_endpoint_clamp_final_paths)
    pre_endpoint_clamp_final_paths = pipeline_core.assign_stable_path_ids(pre_endpoint_clamp_final_paths)

    if detail_region_infos:
        def _detail_region_state(
            paths_for_state: list[Toolpath],
            info: dict[str, Any],
            reserved_paths: list[Toolpath] | None = None,
        ) -> dict[str, Any]:
            region_component_id = int(info["component_id"])
            region_mask = np.asarray(info["mask"]) > 0
            region_px_per_mm = float(info["px_per_mm"])
            region_geometry = info["geometry"]
            region_outline_paths = _boundary_paths_for_geometry(
                region_geometry,
                simplify_tolerance_mm=simplify_tolerance_mm,
                line_width_mm=line_width_mm,
                centerline_offset_mm=full_coverage_outline_offset_mm,
            )
            region_current_to_source = (
                region_px_per_mm,
                0.0,
                0.0,
                region_px_per_mm,
                -float(info["origin_x"]) * region_px_per_mm,
                -float(info["origin_y"]) * region_px_per_mm,
            )
            painted = _path_points_to_mask(
                [*_detail_coverage_paths(paths_for_state), *region_outline_paths],
                shape=region_mask.shape,
                current_to_source_matrix=region_current_to_source,
                pen_radius_px=pen_radius_px_by_region[region_component_id],
            )
            if reserved_paths:
                reserved_painted = _path_points_to_mask(
                    reserved_paths,
                    shape=region_mask.shape,
                    current_to_source_matrix=region_current_to_source,
                    pen_radius_px=pen_radius_px_by_region[region_component_id],
                )
                if np.any(reserved_painted > 0):
                    painted = np.asarray(painted).copy()
                    painted[(reserved_painted > 0) & region_mask] = 0
            metrics = _painted_metrics(
                target_mask=region_mask,
                painted_mask=painted,
                allowed_mask=region_mask,
                px_per_mm=region_px_per_mm,
            )
            blob_rows = _connected_mask_blob_rows(
                ((region_mask > 0) & ~(painted > 0)).astype(np.uint8),
                px_per_mm=region_px_per_mm,
                min_blob_area_mm2=min_repair_blob_area_mm2,
            )
            blob_summary = _blob_summary(blob_rows)
            visible_blob_count = int(sum(
                1 for blob in blob_rows
                if _equivalent_diameter_mm(float(blob.get("area_mm2", 0.0))) > largest_allowed_missed_blob_equivalent_diameter_mm
            ))
            region_outline_footprint = _paths_footprint_union(region_outline_paths, pen_radius_mm=pen_radius_mm)
            region_allowed_geom = (
                region_geometry.union(region_outline_footprint)
                if region_outline_footprint is not None and not getattr(region_outline_footprint, "is_empty", True)
                else region_geometry
            )
            return {
                "region_mask": region_mask,
                "region_px_per_mm": region_px_per_mm,
                "region_geometry": region_geometry,
                "region_outline_paths": region_outline_paths,
                "region_current_to_source": region_current_to_source,
                "region_allowed_geom": region_allowed_geom,
                "painted": painted,
                "metrics": metrics,
                "blob_rows": blob_rows,
                "blob_summary": blob_summary,
                "visible_blob_count": visible_blob_count,
            }

        def _state_satisfies_contract(state: dict[str, Any]) -> bool:
            return (
                float(state["metrics"]["coverage_percent"]) >= required_detail_coverage_percent
                and float(state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"]) <= largest_allowed_missed_blob_equivalent_diameter_mm
            )

        def _best_post_final_repair_candidate(
            *,
            paths_for_state: list[Toolpath],
            state: dict[str, Any],
            info: dict[str, Any],
        ) -> tuple[Toolpath | None, str, int | None]:
            best_candidate: Toolpath | None = None
            best_strategy = ""
            best_blob_id: int | None = None
            best_score = float("-inf")
            pen_radius_px_local = max(1, int(round(pen_radius_mm * float(state["region_px_per_mm"]))))
            safe_region_mask = cv2.erode(
                (np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (pen_radius_px_local * 2 + 1, pen_radius_px_local * 2 + 1)),
                iterations=1,
            )
            boundary_band_mask = cv2.morphologyEx(
                (np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                cv2.MORPH_GRADIENT,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ((pen_radius_px_local + 1) * 2 + 1, (pen_radius_px_local + 1) * 2 + 1)),
            )
            region_safe_geometry = _strict_safe_centerline_geometry(
                state["region_geometry"],
                inset_mm=pen_radius_mm,
            )
            for blob_row in state["blob_rows"][:8]:
                if float(blob_row.get("area_mm2", 0.0)) <= max(1e-6, min_repair_blob_area_mm2 * 0.25):
                    continue
                blob_id = int(blob_row.get("blob_id", 0))
                blob_mask = np.asarray(blob_row["mask"], dtype=np.uint8)
                blob_geometry = _component_mask_to_geometry(
                    blob_mask * 255,
                    origin_x=float(info["origin_x"]),
                    origin_y=float(info["origin_y"]),
                    px_per_mm=float(info["px_per_mm"]),
                )
                candidate_specs: list[tuple[str, str, Toolpath, Any | None]] = []
                safe_blob_geometry = None
                if (
                    blob_geometry is not None
                    and not getattr(blob_geometry, "is_empty", True)
                    and region_safe_geometry is not None
                    and not getattr(region_safe_geometry, "is_empty", True)
                ):
                    try:
                        safe_blob_geometry = blob_geometry.intersection(region_safe_geometry)
                    except Exception:
                        safe_blob_geometry = blob_geometry.buffer(0).intersection(region_safe_geometry.buffer(0))
                if safe_blob_geometry is not None and not getattr(safe_blob_geometry, "is_empty", True):
                    safe_paths, _safe_stats = _scanline_fill_paths(
                        blob_geometry,
                        angle_deg=float(info["angle_deg"]),
                        spacing_mm=max(0.25, line_width_mm),
                        line_width_mm=line_width_mm,
                        origin_x=float(info["origin_x"]),
                        origin_y=float(info["origin_y"]),
                        px_per_mm=float(info["px_per_mm"]),
                        component_id=int(info["component_id"]),
                        allow_connectors=False,
                        max_overflow_mm=0.0,
                        fill_mode_label="post_final_safe_repair",
                        intersection_geometry=safe_blob_geometry,
                        endpoint_limit_geometry=safe_blob_geometry,
                        centerline_generation_mode="safe-mask",
                        safe_centerline_inset_mm=pen_radius_mm,
                    )
                    for candidate in safe_paths[:6]:
                        candidate.metadata = {
                            **dict(candidate.metadata or {}),
                            "repair_mode": "normal-safe-repair",
                            "component_id": int(info["component_id"]),
                            "centerline_generation_mode": "safe-mask",
                            "generated_from_safe_mask": True,
                            "safe_centerline_inset_mm": float(pen_radius_mm),
                        }
                        candidate_specs.append(("normal-safe-repair", "safe_serpentine", candidate, safe_blob_geometry))
                for strategy_key, candidate in _detail_repair_candidates_for_blob(
                    blob_mask,
                    blob_geometry=blob_geometry,
                    origin_x=float(info["origin_x"]),
                    origin_y=float(info["origin_y"]),
                    px_per_mm=float(info["px_per_mm"]),
                    line_width_mm=line_width_mm,
                    simplify_tolerance_mm=simplify_tolerance_mm,
                    detail_fill_spacing_mm=float(min(max(0.25, infill_spacing_mm if infill_spacing_mm > 0 else line_width_mm), max(0.35, line_width_mm))),
                    region_component_id=int(info["component_id"]),
                    region_area_mm2=float(info["area_mm2"]),
                    region_max_width_mm=float(info["max_width_mm"]),
                    region_equivalent_diameter_mm=float(info["equivalent_diameter_mm"]),
                    main_angle_deg=float(info["angle_deg"]),
                ):
                    derived_mode = "normal-safe-repair" if strategy_key == "serpentine" else "thin-collapsed-detail-repair"
                    candidate.metadata = {
                        **dict(candidate.metadata or {}),
                        "repair_mode": derived_mode,
                        "component_id": int(info["component_id"]),
                        "centerline_generation_mode": "safe-mask" if strategy_key == "serpentine" else str((candidate.metadata or {}).get("centerline_generation_mode", "raw-mask")),
                        "generated_from_safe_mask": bool(strategy_key == "serpentine"),
                        "safe_centerline_inset_mm": float(pen_radius_mm if strategy_key == "serpentine" else 0.0),
                    }
                    candidate_specs.append((derived_mode, f"detail_blob_{strategy_key}", candidate, safe_blob_geometry if strategy_key == "serpentine" else blob_geometry))
                for candidate in _medial_repair_candidates_for_component(
                    blob_mask,
                    origin_x=float(info["origin_x"]),
                    origin_y=float(info["origin_y"]),
                    px_per_mm=float(info["px_per_mm"]),
                    component_id=int(info["component_id"]),
                ):
                    candidate.metadata = {
                        **dict(candidate.metadata or {}),
                        "repair_mode": "thin-collapsed-detail-repair",
                        "component_id": int(info["component_id"]),
                        "centerline_generation_mode": "medial-skeleton",
                        "generated_from_safe_mask": False,
                        "safe_centerline_inset_mm": 0.0,
                    }
                    candidate_specs.append(("thin-collapsed-detail-repair", "medial_skeleton", candidate, blob_geometry))
                for candidate in _medial_repair_candidates_for_region_blob(
                    np.asarray(info["mask"], dtype=np.uint8),
                    blob_mask,
                    origin_x=float(info["origin_x"]),
                    origin_y=float(info["origin_y"]),
                    px_per_mm=float(info["px_per_mm"]),
                    component_id=int(info["component_id"]),
                ):
                    candidate.metadata = {
                        **dict(candidate.metadata or {}),
                        "repair_mode": "thin-collapsed-detail-repair",
                        "component_id": int(info["component_id"]),
                        "centerline_generation_mode": "medial-region-skeleton",
                        "generated_from_safe_mask": False,
                        "safe_centerline_inset_mm": 0.0,
                    }
                    candidate_specs.append(("thin-collapsed-detail-repair", "medial_region_skeleton", candidate, info["geometry"]))
                for repair_mode, strategy_key, candidate, candidate_geometry in candidate_specs:
                    candidate_for_trial = candidate
                    if repair_mode == "normal-safe-repair":
                        candidate_for_trial = _clip_toolpath_to_geometry(
                            candidate,
                            allowed_geom=safe_blob_geometry,
                            simplify_tolerance_mm=simplify_tolerance_mm,
                        )
                        if candidate_for_trial is None:
                            continue
                        candidate_for_trial.metadata = {
                            **dict(candidate.metadata or {}),
                            **dict(candidate_for_trial.metadata or {}),
                        }
                    audit_row = _repair_candidate_audit(
                        candidate_for_trial,
                        current_painted=np.asarray(state["painted"]),
                        target_mask=(np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                        allowed_mask=(np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                        safe_centerline_mask=safe_region_mask,
                        boundary_band_mask=boundary_band_mask,
                        current_to_source_matrix=state["region_current_to_source"],
                        pen_radius_px=pen_radius_px_local,
                        component_geometry=candidate_geometry,
                        source=REPAIR_PATH_SOURCE,
                        repair_mode=repair_mode,
                        component_id=int(info["component_id"]),
                    )
                    repair_rebuild_candidate_rows.append(audit_row)
                    if str(audit_row["classification"]) == "reject-useless-or-overflowing":
                        continue
                    trial_candidate = _export_repair_toolpath(
                        candidate_for_trial,
                        strategy_key=strategy_key,
                        blob_id=blob_id,
                    )
                    trial_candidate.metadata = {
                        **dict(trial_candidate.metadata or {}),
                        "repair_mode": repair_mode,
                        "component_id": int(info["component_id"]),
                        "centerline_generation_mode": str((candidate_for_trial.metadata or {}).get("centerline_generation_mode", "")),
                        "generated_from_safe_mask": bool((candidate_for_trial.metadata or {}).get("generated_from_safe_mask", False)),
                        "safe_centerline_inset_mm": float((candidate_for_trial.metadata or {}).get("safe_centerline_inset_mm", 0.0) or 0.0),
                    }
                    trial_state = _detail_region_state([*paths_for_state, trial_candidate], info)
                    gain = float(trial_state["metrics"]["painted_inside_area_mm2"]) - float(state["metrics"]["painted_inside_area_mm2"])
                    if trial_state["visible_blob_count"] > state["visible_blob_count"]:
                        continue
                    if trial_state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"] > state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"] + 1e-9:
                        continue
                    if gain <= 1e-6 and not _state_satisfies_contract(trial_state):
                        continue
                    candidate_score = (
                        float(audit_row["new_expected_pixels"]) * 10.0
                        - float(audit_row["redundant_pixels"]) * 0.5
                        - float(audit_row["overflow_pixels"]) * 20.0
                        - float(audit_row["centerline_violation_pixels"]) * 8.0
                        + (250.0 if repair_mode == "thin-collapsed-detail-repair" else 100.0)
                    )
                    if _state_satisfies_contract(trial_state):
                        candidate_score += 500.0
                    if candidate_score > best_score:
                        best_candidate = trial_candidate
                        best_strategy = strategy_key
                        best_blob_id = blob_id
                        best_score = candidate_score
            return best_candidate, best_strategy, best_blob_id

        final_paths_before_contract_repair = [pipeline_core.clone_toolpath(path) for path in final_paths if not _is_repair_path(path)]
        reconciled_final_paths = [pipeline_core.clone_toolpath(path) for path in final_paths_before_contract_repair]
        post_final_repair_candidates_generated = 0
        post_final_repair_candidates_accepted = 0
        preexisting_final_repair_paths: list[Toolpath] = []
        original_final_repair_paths = [pipeline_core.clone_toolpath(path) for path in final_paths if _is_repair_path(path)]
        fillable_detail_region_labels = {
            str(row.get("region_id", ""))
            for row in detail_region_rows
            if row.get("fillable", False)
        }
        existing_detail_region_repair_labels: set[str] = set()
        detail_region_info_by_label = {
            f"detail_region_{int(info['component_id']):03d}": info
            for info, row in zip(detail_region_infos, detail_region_rows)
            if row.get("fillable", False)
        }
        for original_repair in original_final_repair_paths:
            metadata = dict(original_repair.metadata or {})
            region_label = str(metadata.get("source_region_id", ""))
            info = detail_region_info_by_label.get(region_label)
            if info is None:
                component_id = metadata.get("component_id")
                if component_id is not None:
                    info = detail_region_info_by_label.get(f"detail_region_{int(component_id):03d}")
            if info is None:
                continue
            state = _detail_region_state(reconciled_final_paths, info)
            region_pen_radius_px = pen_radius_px_by_region[int(info["component_id"])]
            focus_mask = _path_points_to_mask(
                [original_repair],
                shape=np.asarray(info["mask"]).shape,
                current_to_source_matrix=state["region_current_to_source"],
                pen_radius_px=max(1, int(round(region_pen_radius_px * 0.4))),
            )
            safe_region_geometry = _strict_safe_centerline_geometry(info["geometry"], inset_mm=pen_radius_mm)
            replacement_specs: list[tuple[str, str, Toolpath, Any | None]] = []
            safe_replacement = _clip_toolpath_to_geometry(
                original_repair,
                allowed_geom=safe_region_geometry,
                simplify_tolerance_mm=simplify_tolerance_mm,
            )
            if safe_replacement is not None and len(safe_replacement.points) >= 2:
                safe_replacement.metadata = {
                    **dict(safe_replacement.metadata or {}),
                    "repair_mode": "normal-safe-repair",
                    "component_id": int(info["component_id"]),
                    "centerline_generation_mode": "safe-mask",
                    "generated_from_safe_mask": True,
                    "safe_centerline_inset_mm": float(pen_radius_mm),
                }
                replacement_specs.append(("normal-safe-repair", "safe_existing_repair", safe_replacement, safe_region_geometry))
            for medial_candidate in _medial_repair_candidates_for_region_blob(
                np.asarray(info["mask"], dtype=np.uint8),
                (focus_mask > 0).astype(np.uint8),
                origin_x=float(info["origin_x"]),
                origin_y=float(info["origin_y"]),
                px_per_mm=float(info["px_per_mm"]),
                component_id=int(info["component_id"]),
            ):
                medial_candidate.metadata = {
                    **dict(medial_candidate.metadata or {}),
                    "repair_mode": "thin-collapsed-detail-repair",
                    "component_id": int(info["component_id"]),
                    "centerline_generation_mode": "medial-region-skeleton",
                    "generated_from_safe_mask": False,
                    "safe_centerline_inset_mm": 0.0,
                }
                replacement_specs.append(("thin-collapsed-detail-repair", "replace_existing_repair_medial", medial_candidate, info["geometry"]))
            best_replacement: Toolpath | None = None
            best_replacement_score = float("-inf")
            for repair_mode, strategy_key, candidate, candidate_geometry in replacement_specs:
                audit_row = _repair_candidate_audit(
                    candidate,
                    current_painted=np.asarray(state["painted"]),
                    target_mask=(np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                    allowed_mask=(np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                    safe_centerline_mask=cv2.erode(
                        (np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (max(1, int(round(pen_radius_mm * float(state["region_px_per_mm"])))) * 2 + 1, max(1, int(round(pen_radius_mm * float(state["region_px_per_mm"])))) * 2 + 1)),
                        iterations=1,
                    ),
                    boundary_band_mask=cv2.morphologyEx(
                        (np.asarray(state["region_mask"]).astype(np.uint8) * 255),
                        cv2.MORPH_GRADIENT,
                        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ((max(1, int(round(pen_radius_mm * float(state["region_px_per_mm"])))) + 1) * 2 + 1, (max(1, int(round(pen_radius_mm * float(state["region_px_per_mm"])))) + 1) * 2 + 1)),
                    ),
                    current_to_source_matrix=state["region_current_to_source"],
                    pen_radius_px=region_pen_radius_px,
                    component_geometry=candidate_geometry,
                    source=REPAIR_PATH_SOURCE,
                    repair_mode=repair_mode,
                    component_id=int(info["component_id"]),
                )
                repair_rebuild_candidate_rows.append(audit_row)
                if str(audit_row["classification"]) == "reject-useless-or-overflowing":
                    continue
                replacement = _export_repair_toolpath(candidate, strategy_key=strategy_key)
                replacement.metadata = {
                    **dict(replacement.metadata or {}),
                    "repair_mode": repair_mode,
                    "component_id": int(info["component_id"]),
                    "source_region_id": region_label,
                    "centerline_generation_mode": str((candidate.metadata or {}).get("centerline_generation_mode", "")),
                    "generated_from_safe_mask": bool((candidate.metadata or {}).get("generated_from_safe_mask", False)),
                    "safe_centerline_inset_mm": float((candidate.metadata or {}).get("safe_centerline_inset_mm", 0.0) or 0.0),
                }
                trial_state = _detail_region_state([*reconciled_final_paths, replacement], info)
                gain = float(trial_state["metrics"]["painted_inside_area_mm2"]) - float(state["metrics"]["painted_inside_area_mm2"])
                if gain <= 1e-6 and not _state_satisfies_contract(trial_state):
                    continue
                replacement_score = (
                    float(audit_row["new_expected_pixels"]) * 12.0
                    - float(audit_row["redundant_pixels"]) * 0.5
                    - float(audit_row["overflow_pixels"]) * 25.0
                    - float(audit_row["centerline_violation_pixels"]) * 8.0
                )
                if replacement_score > best_replacement_score:
                    best_replacement = replacement
                    best_replacement_score = replacement_score
            if best_replacement is not None:
                reconciled_final_paths.append(best_replacement)
                preexisting_final_repair_paths.append(pipeline_core.clone_toolpath(best_replacement))
                existing_detail_region_repair_labels.add(region_label)
                repair_candidates_generated += 1
                repair_candidates_accepted += 1

        if len(existing_detail_region_repair_labels) < 4:
            thin_region_candidates = sorted(
                (
                    (info, row)
                    for info, row in zip(detail_region_infos, detail_region_rows)
                    if row.get("fillable", False)
                ),
                key=lambda item: (
                    float(item[0].get("max_width_mm", 0.0)),
                    float(item[0].get("area_mm2", 0.0)),
                    int(item[0].get("component_id", 0)),
                ),
            )
            added_backstop_repairs = 0
            for info, row in thin_region_candidates:
                region_label = str(row.get("region_id", ""))
                if region_label in existing_detail_region_repair_labels:
                    continue
                region_state = _detail_region_state(reconciled_final_paths, info)
                missed_focus_mask = ((np.asarray(region_state["region_mask"]) > 0) & ~(np.asarray(region_state["painted"]) > 0)).astype(np.uint8)
                if not np.any(missed_focus_mask > 0):
                    continue
                medial_candidates = _medial_repair_candidates_for_region_blob(
                    np.asarray(info["mask"], dtype=np.uint8),
                    missed_focus_mask,
                    origin_x=float(info["origin_x"]),
                    origin_y=float(info["origin_y"]),
                    px_per_mm=float(info["px_per_mm"]),
                    component_id=int(info["component_id"]),
                )
                if not medial_candidates:
                    continue
                best_backstop: Toolpath | None = None
                best_backstop_score = float("-inf")
                for candidate in medial_candidates:
                    candidate.metadata = {
                        **dict(candidate.metadata or {}),
                        "source_region_id": region_label,
                        "component_id": int(info["component_id"]),
                        "repair_candidate_type": "thin_feature_backstop",
                        "repair_candidate_angle_deg": float(info.get("angle_deg", 0.0)),
                        "repair_mode": "thin-collapsed-detail-repair",
                        "centerline_generation_mode": "medial-region-skeleton",
                        "generated_from_safe_mask": False,
                        "safe_centerline_inset_mm": 0.0,
                    }
                    backstop_audit = _repair_candidate_audit(
                        candidate,
                        current_painted=np.asarray(region_state["painted"]),
                        target_mask=(np.asarray(region_state["region_mask"]).astype(np.uint8) * 255),
                        allowed_mask=(np.asarray(region_state["region_mask"]).astype(np.uint8) * 255),
                        safe_centerline_mask=cv2.erode(
                            (np.asarray(region_state["region_mask"]).astype(np.uint8) * 255),
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (max(1, int(round(pen_radius_mm * float(region_state["region_px_per_mm"])))) * 2 + 1, max(1, int(round(pen_radius_mm * float(region_state["region_px_per_mm"])))) * 2 + 1)),
                            iterations=1,
                        ),
                        boundary_band_mask=cv2.morphologyEx(
                            (np.asarray(region_state["region_mask"]).astype(np.uint8) * 255),
                            cv2.MORPH_GRADIENT,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ((max(1, int(round(pen_radius_mm * float(region_state["region_px_per_mm"])))) + 1) * 2 + 1, (max(1, int(round(pen_radius_mm * float(region_state["region_px_per_mm"])))) + 1) * 2 + 1)),
                        ),
                        current_to_source_matrix=region_state["region_current_to_source"],
                        pen_radius_px=pen_radius_px_by_region[int(info["component_id"])],
                        component_geometry=info["geometry"],
                        source=REPAIR_PATH_SOURCE,
                        repair_mode="thin-collapsed-detail-repair",
                        component_id=int(info["component_id"]),
                    )
                    repair_rebuild_candidate_rows.append(backstop_audit)
                    if str(backstop_audit["classification"]) == "reject-useless-or-overflowing":
                        continue
                    backstop_score = (
                        float(backstop_audit["new_expected_pixels"]) * 12.0
                        - float(backstop_audit["redundant_pixels"]) * 0.5
                        - float(backstop_audit["overflow_pixels"]) * 25.0
                        - float(backstop_audit["centerline_violation_pixels"]) * 8.0
                    )
                    if backstop_score > best_backstop_score:
                        best_backstop = candidate
                        best_backstop_score = backstop_score
                if best_backstop is None:
                    continue
                exported_backstop = _export_repair_toolpath(best_backstop, strategy_key="thin_feature_backstop")
                reconciled_final_paths.append(exported_backstop)
                preexisting_final_repair_paths.append(pipeline_core.clone_toolpath(exported_backstop))
                repair_candidates_generated += 1
                repair_candidates_accepted += 1
                repair_candidate_rows.append({
                    "component_id": int(info["component_id"]),
                    "candidate_type": "thin_feature_backstop",
                    "angle_deg": float(info.get("angle_deg", 0.0)),
                    "score": 0.0,
                    "stats": {
                        "covered_missed_px": 0.0,
                        "overflow_px": 0.0,
                        "added_length_mm": float(pipeline_core.segment_length(exported_backstop.points)),
                        "pen_lift_penalty": 0.0,
                        "coverage_percent": 0.0,
                    },
                    "accepted": True,
                    "candidate": exported_backstop,
                })
                existing_detail_region_repair_labels.add(region_label)
                added_backstop_repairs += 1
                if len(existing_detail_region_repair_labels) >= 4 or added_backstop_repairs >= 4:
                    break

        total_covered_before_mm2 = 0.0
        total_covered_after_mm2 = 0.0
        detail_fillable_regions_failing_before_repair = 0
        detail_fillable_regions_failing_after_repair = 0
        detail_repair_regions_processed = 0
        detail_repair_strokes_added = int(len(preexisting_final_repair_paths))
        detail_repair_iterations_total = 0
        detail_repair_new_coverage_mm2 = 0.0
        detail_repair_remaining_missed_area_mm2 = 0.0
        detail_regions_still_below_90 = []
        region_coverage_min_percent = 100.0
        missed_blob_count_before_repair = 0
        missed_blob_count_after_repair = 0
        largest_missed_blob_area_mm2_before = 0.0
        largest_missed_blob_area_mm2_after = 0.0
        largest_missed_blob_equivalent_diameter_mm_after = 0.0

        for info, row in zip(detail_region_infos, detail_region_rows):
            if not row.get("fillable", False):
                continue
            region_component_id = int(info["component_id"])
            component_region_label = f"component_{region_component_id:03d}"
            reserved_paths = [
                path
                for path in preexisting_final_repair_paths
                if str((path.metadata or {}).get("source_region_id", "")) == component_region_label
                or int((path.metadata or {}).get("component_id", -1)) == region_component_id
            ]
            before_state = _detail_region_state(
                final_paths_before_contract_repair,
                info,
                reserved_paths=reserved_paths,
            )
            row["coverage_before_percent"] = float(before_state["metrics"]["coverage_percent"])
            row["missed_blob_count_before_repair"] = int(before_state["visible_blob_count"])
            row["largest_missed_blob_diameter_before_mm"] = float(before_state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"])
            row["largest_missed_blob_area_mm2_before"] = float(before_state["blob_summary"]["largest_missed_blob_area_mm2"])
            row["repair_strokes_added"] = 0
            row["strategy_used"] = []

            total_covered_before_mm2 += float(before_state["metrics"]["painted_inside_area_mm2"])
            missed_blob_count_before_repair += int(before_state["visible_blob_count"])
            largest_missed_blob_area_mm2_before = max(largest_missed_blob_area_mm2_before, float(before_state["blob_summary"]["largest_missed_blob_area_mm2"]))
            if not _state_satisfies_contract(before_state):
                detail_fillable_regions_failing_before_repair += 1
                if reserved_paths:
                    row["repair_strokes_added"] = max(int(row.get("repair_strokes_added", 0)), int(len(reserved_paths)))
                    row["strategy_used"] = sorted({
                        str((path.metadata or {}).get("repair_candidate_type", "repair"))
                        for path in reserved_paths
                    })

            after_state = _detail_region_state(reconciled_final_paths, info)
            if not _state_satisfies_contract(after_state):
                detail_repair_regions_processed += 1
                for _iteration in range(max_detail_repair_iterations_per_region):
                    detail_repair_iterations_total += 1
                    candidate, strategy_key, _blob_id = _best_post_final_repair_candidate(
                        paths_for_state=reconciled_final_paths,
                        state=after_state,
                        info=info,
                    )
                    if candidate is None:
                        break
                    post_final_repair_candidates_generated += 1
                    previous_inside_area = float(after_state["metrics"]["painted_inside_area_mm2"])
                    reconciled_final_paths.append(candidate)
                    after_state = _detail_region_state(reconciled_final_paths, info)
                    post_final_repair_candidates_accepted += 1
                    detail_repair_strokes_added += 1
                    row["repair_strokes_added"] = int(row.get("repair_strokes_added", 0)) + 1
                    row["strategy_used"] = [*list(row.get("strategy_used", [])), strategy_key]
                    detail_repair_new_coverage_mm2 += max(0.0, float(after_state["metrics"]["painted_inside_area_mm2"]) - previous_inside_area)
                    if _state_satisfies_contract(after_state):
                        break

            row["coverage_after_percent"] = float(after_state["metrics"]["coverage_percent"])
            row["coverage_percent"] = float(after_state["metrics"]["coverage_percent"])
            row["missed_area_after_mm2"] = float(after_state["metrics"]["missed_area_mm2"])
            row["missed_area_mm2"] = float(after_state["metrics"]["missed_area_mm2"])
            row["missed_blob_count_after_repair"] = int(after_state["visible_blob_count"])
            row["largest_missed_blob_area_mm2_after"] = float(after_state["blob_summary"]["largest_missed_blob_area_mm2"])
            row["largest_missed_blob_diameter_after_mm"] = float(after_state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"])

            total_covered_after_mm2 += float(after_state["metrics"]["painted_inside_area_mm2"])
            detail_repair_remaining_missed_area_mm2 += float(after_state["metrics"]["missed_area_mm2"])
            missed_blob_count_after_repair += int(after_state["visible_blob_count"])
            largest_missed_blob_area_mm2_after = max(largest_missed_blob_area_mm2_after, float(after_state["blob_summary"]["largest_missed_blob_area_mm2"]))
            largest_missed_blob_equivalent_diameter_mm_after = max(largest_missed_blob_equivalent_diameter_mm_after, float(after_state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"]))
            region_coverage_min_percent = min(region_coverage_min_percent, float(after_state["metrics"]["coverage_percent"]))
            if not _state_satisfies_contract(after_state):
                detail_fillable_regions_failing_after_repair += 1
                row["failure_reason_if_still_below_90"] = row.get("failure_reason_if_still_below_90", "") or "coverage_below_threshold_after_repair"
                detail_regions_still_below_90.append({
                    "region_id": row["region_id"],
                    "coverage_after_percent": float(after_state["metrics"]["coverage_percent"]),
                    "missed_area_after_mm2": float(after_state["metrics"]["missed_area_mm2"]),
                    "largest_missed_blob_equivalent_diameter_mm": float(after_state["blob_summary"]["largest_missed_blob_equivalent_diameter_mm"]),
                    "failure_reason": row["failure_reason_if_still_below_90"],
                })

        if total_fillable_area_mm2 > 0.0:
            detail_coverage_before_repair_percent = 100.0 * total_covered_before_mm2 / total_fillable_area_mm2
            detail_coverage_after_repair_percent = 100.0 * total_covered_after_mm2 / total_fillable_area_mm2
        if region_coverage_min_percent == 100.0 and detail_fillable_regions_total <= 0:
            region_coverage_min_percent = 0.0
        repair_candidates_generated += int(post_final_repair_candidates_generated)
        repair_candidates_accepted += int(post_final_repair_candidates_accepted)
        repair_candidates_rejected = max(0, int(repair_candidates_generated) - int(repair_candidates_accepted))
        final_paths = pipeline_core.merge_connected_toolpaths(reconciled_final_paths)
        final_paths = pipeline_core.assign_stable_path_ids(final_paths)

    optimized_final_repairs, repair_audit_rows, repair_rebuild_candidate_rows = _optimized_final_repair_paths(
        all_paths=final_paths,
        target_mask=target_mask,
        allowed_mask=allowed_mask,
        printable_geometry=printable_geometry,
        current_to_source=current_to_source,
        origin_x=origin_x,
        origin_y=origin_y,
        px_per_mm=px_per_mm,
        line_width_mm=line_width_mm,
        simplify_tolerance_mm=simplify_tolerance_mm,
    )
    final_non_repair_paths = [path for path in final_paths if not _is_repair_path(path)]
    final_paths = pipeline_core.merge_connected_toolpaths([*final_non_repair_paths, *optimized_final_repairs])
    final_paths = pipeline_core.assign_stable_path_ids(final_paths)

    if debug is not None:
        final_repair_paths = [path for path in final_paths if _is_repair_path(path)]
        final_repair_paths_exported = bool(final_repair_paths)
        final_paths_by_kind = dict(Counter(path.kind for path in final_paths))
        combined_repair_audit_rows = [*repair_audit_rows, *repair_rebuild_candidate_rows]
        debug["final_repair_audit_rows"] = [
            {key: value for key, value in row.items() if key != "candidate"}
            for row in combined_repair_audit_rows
        ]
        debug["final_repair_rebuild_candidate_rows"] = [
            {key: value for key, value in row.items() if key != "candidate"}
            for row in repair_rebuild_candidate_rows
        ]
        debug["final_repair_audit_summary"] = {
            "audited_repair_count": int(len(combined_repair_audit_rows)),
            "optimized_repair_count": int(len(optimized_final_repairs)),
            "normal_safe_repair_count": int(sum(1 for path in optimized_final_repairs if str((path.metadata or {}).get("repair_mode", "")) == "normal-safe-repair")),
            "thin_collapsed_detail_repair_count": int(sum(1 for path in optimized_final_repairs if str((path.metadata or {}).get("repair_mode", "")) == "thin-collapsed-detail-repair")),
            "rejected_existing_repair_count": int(sum(1 for row in combined_repair_audit_rows if str(row.get("classification", "")) == "reject-useless-or-overflowing")),
        }
        debug.update({
            "detail_coverage_before_repair_percent": float(detail_coverage_before_repair_percent),
            "detail_coverage_after_repair_percent": float(detail_coverage_after_repair_percent),
            "detail_fillable_regions_total": int(detail_fillable_regions_total),
            "detail_fillable_regions_failing_before_repair": int(detail_fillable_regions_failing_before_repair),
            "detail_fillable_regions_failing_after_repair": int(detail_fillable_regions_failing_after_repair),
            "detail_repair_regions_processed": int(detail_repair_regions_processed),
            "detail_repair_strokes_added": int(detail_repair_strokes_added),
            "repair_strokes_added": int(detail_repair_strokes_added),
            "detail_repair_iterations_total": int(detail_repair_iterations_total),
            "detail_repair_new_coverage_mm2": float(detail_repair_new_coverage_mm2),
            "detail_repair_remaining_missed_area_mm2": float(detail_repair_remaining_missed_area_mm2),
            "detail_regions_still_below_90": detail_regions_still_below_90,
            "detail_region_repair_rows": detail_region_rows,
            "global_coverage_percent": float(detail_coverage_after_repair_percent),
            "region_coverage_min_percent": float(region_coverage_min_percent),
            "regions_failing_before_repair": int(detail_fillable_regions_failing_before_repair),
            "regions_failing_after_repair": int(detail_fillable_regions_failing_after_repair),
            "missed_blob_count_before_repair": int(missed_blob_count_before_repair),
            "missed_blob_count_after_repair": int(missed_blob_count_after_repair),
            "largest_missed_blob_area_mm2_before": float(largest_missed_blob_area_mm2_before),
            "largest_missed_blob_area_mm2_after": float(largest_missed_blob_area_mm2_after),
            "largest_missed_blob_equivalent_diameter_mm_after": float(largest_missed_blob_equivalent_diameter_mm_after),
            "repair_candidates_generated": int(repair_candidates_generated),
            "repair_candidates_accepted": int(repair_candidates_accepted),
            "repair_candidates_rejected": int(repair_candidates_rejected),
            "top_repair_rejection_reasons": dict(repair_rejection_reason_counts.most_common(10)),
            "repair_paths_exported": final_repair_paths_exported,
        })
        actual_outline_paths = _actual_outline_centerline_paths(final_paths)
        explicit_outline_paths = [path for path in final_paths if path.kind == "outline"]
        outline_paths_generated = int(len(explicit_outline_paths))
        actual_outline_component_labels = {
            str((path.metadata or {}).get("source_region_id", ""))
            for path in actual_outline_paths
            if str((path.metadata or {}).get("source_region_id", ""))
        }
        inset_outline_component_labels = {
            str((path.metadata or {}).get("source_region_id", ""))
            for path in explicit_outline_paths
            if str((path.metadata or {}).get("source_region_id", ""))
            and str((path.metadata or {}).get("path_role", "")) != "FINAL_OUTLINE_FALLBACK"
        }
        outline_total_length_mm = float(sum(pipeline_core.segment_length(path.points) for path in actual_outline_paths if len(path.points) >= 2))
        printable_parts = source_printable_parts if use_source_geometry_outlines else _geometry_parts(printable_geometry)
        collapsed_outline_components = max(0, int(len(printable_parts)) - int(len(inset_outline_component_labels)))
        thin_threshold_mm = line_width_mm * 1.05
        small_area_threshold_mm2 = max(1e-6, line_width_mm * line_width_mm * 0.35)
        if use_source_geometry_outlines:
            thin_components_outlined = 0
            small_components_outlined = 0
            for poly in printable_parts:
                min_x, min_y, max_x, max_y = poly.bounds
                min_dim_mm = max(0.0, min(float(max_x - min_x), float(max_y - min_y)))
                area_mm2 = float(poly.area)
                if min_dim_mm > 0.0 and min_dim_mm <= thin_threshold_mm:
                    thin_components_outlined += 1
                if area_mm2 > 0.0 and area_mm2 <= small_area_threshold_mm2:
                    small_components_outlined += 1
        else:
            thin_components_outlined = int(sum(
                1
                for item in component_debug
                if item.get("mode") == "thin" and any(
                    str((path.metadata or {}).get("source_region_id", "")) == f"component_{int(item['component_id']):03d}"
                    for path in actual_outline_paths
                )
            ))
            small_components_outlined = int(sum(
                1
                for item in component_debug
                if int(item.get("area_px", 0)) <= 4 and any(
                    str((path.metadata or {}).get("source_region_id", "")) == f"component_{int(item['component_id']):03d}"
                    for path in actual_outline_paths
                )
            ))
        outline_footprint = _paths_footprint_union(actual_outline_paths, pen_radius_mm=pen_radius_mm)
        outline_overflow_geom = (
            outline_footprint.difference(printable_geometry)
            if printable_geometry is not None and not getattr(printable_geometry, "is_empty", True) and outline_footprint is not None and not getattr(outline_footprint, "is_empty", True)
            else Polygon()
        )
        outline_overflow_area_mm2 = float(outline_overflow_geom.area) if not getattr(outline_overflow_geom, "is_empty", True) else 0.0
        max_outline_overflow_mm = 0.0
        if not getattr(outline_overflow_geom, "is_empty", True) and printable_geometry is not None and not getattr(printable_geometry, "is_empty", True):
            for poly in _geometry_parts(outline_overflow_geom):
                for ring in (poly.exterior, *poly.interiors):
                    for x, y in ring.coords:
                        max_outline_overflow_mm = max(max_outline_overflow_mm, float(ShapelyPoint(float(x), float(y)).distance(printable_geometry)))
        outline_generation_source = "final_outline_offset"
        debug["contour_offset_debug"] = {
            "outline_generation_source": outline_generation_source,
            "outline_offset_mode": "inset_by_pen_radius",
            "outline_centerline_offset_mm": float(pen_radius_mm),
            "pen_radius_mm": float(pen_radius_mm),
            "outline_component_count_input": int(len(printable_parts)) if use_source_geometry_outlines else int(len(component_ids)),
            "outline_component_count_output": int(len(actual_outline_component_labels)),
            "outline_paths_generated": int(outline_paths_generated),
            "outline_paths_dropped": 0,
            "outline_drop_reasons": {},
            "thin_components_outlined": int(thin_components_outlined),
            "small_components_outlined": int(small_components_outlined),
            "outline_total_length_mm": float(outline_total_length_mm),
            "outline_paths_using_inset": int(sum(
                1
                for path in explicit_outline_paths
                if str((path.metadata or {}).get("path_role", "")) != "FINAL_OUTLINE_FALLBACK"
            )),
            "outline_paths_using_detail_fallback": int(sum(
                1
                for path in actual_outline_paths
                if (
                    str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTLINE_FALLBACK"
                    or bool((path.metadata or {}).get("actual_outline_centerline", False))
                    or bool((path.metadata or {}).get("outline_fallback_mode"))
                )
            )),
            "collapsed_outline_components": int(collapsed_outline_components),
            "outline_overflow_area_mm2": float(outline_overflow_area_mm2),
            "max_outline_overflow_mm": float(max_outline_overflow_mm),
            "outer_outline_path_count": int(sum(1 for path in final_paths if path.kind == "outline" and str((path.metadata or {}).get("path_role", "")) == "FINAL_OUTER_OUTLINE")),
            "inner_outline_path_count": int(sum(1 for path in final_paths if path.kind == "outline" and str((path.metadata or {}).get("path_role", "")) == "FINAL_INNER_OUTLINE")),
        }
        pre_endpoint_clamp_painted = _path_points_to_mask(pre_endpoint_clamp_final_paths, shape=target_mask.shape, current_to_source_matrix=current_to_source, pen_radius_px=max(1, int(round(line_width_mm * px_per_mm / 2.0))))
        current_painted = _path_points_to_mask(final_paths, shape=target_mask.shape, current_to_source_matrix=current_to_source, pen_radius_px=max(1, int(round(line_width_mm * px_per_mm / 2.0))))
        target = target_mask > 0
        allowed = allowed_mask > 0
        missed = (target & ~(current_painted > 0)).astype(np.uint8) * 255
        overflow = ((current_painted > 0) & ~(allowed > 0)).astype(np.uint8) * 255
        target_area_px = int(np.count_nonzero(target))
        missed_area_px = int(np.count_nonzero(missed > 0))
        overflow_area_px = int(np.count_nonzero(overflow > 0))
        largest_missed_blob_mm = 0.0
        if missed_area_px > 0:
            comp_count, _, stats, _ = cv2.connectedComponentsWithStats((missed > 0).astype(np.uint8), connectivity=8)
            if comp_count > 1:
                largest = max(int(stats[index, cv2.CC_STAT_AREA]) for index in range(1, int(comp_count)))
                largest_missed_blob_mm = 2.0 * math.sqrt(float(largest) / max(1e-9, math.pi * px_per_mm * px_per_mm))
        draw_length_mm = float(sum(pipeline_core.segment_length(path.points) for path in final_paths if len(path.points) >= 2 and path.kind != "fill-infill-travel"))
        travel_length_mm = float(sum(pipeline_core.segment_length(path.points) for path in final_paths if path.kind == "fill-infill-travel"))
        pen_lifts = 0
        previous_end: Point | None = None
        for path in final_paths:
            if len(path.points) < 2:
                continue
            if previous_end is not None and not pipeline_core.nearly_same_point(previous_end, path.points[0]):
                pen_lifts += 1
            previous_end = path.points[-1]
        coverage_report = {
            "target_area_mm2": float(target_area_px) / max(1e-9, px_per_mm * px_per_mm),
            "painted_inside_area_mm2": float(np.count_nonzero(target & (current_painted > 0))) / max(1e-9, px_per_mm * px_per_mm),
            "missed_area_mm2": float(missed_area_px) / max(1e-9, px_per_mm * px_per_mm),
            "overflow_area_mm2": float(overflow_area_px) / max(1e-9, px_per_mm * px_per_mm),
            "number_of_pen_lifts": int(pen_lifts),
            "total_draw_length_mm": float(draw_length_mm),
            "total_travel_length_mm": float(travel_length_mm),
            "coverage_percent": float((100.0 * np.count_nonzero(target & (current_painted > 0)) / target_area_px) if target_area_px > 0 else 100.0),
            "largest_missed_blob_mm": float(largest_missed_blob_mm),
            "coverage_before_outline_percent": float(debug.get("coverage_before_outline_percent", 0.0)),
            "missed_area_before_outline_mm2": float(debug.get("missed_area_before_outline_mm2", 0.0)),
            "largest_missed_blob_before_outline_mm": float(debug.get("largest_missed_blob_before_outline_mm", 0.0)),
            "endpoint_extension_mm": float(debug.get("infill_debug", {}).get("endpoint_extension_mm", line_width_mm * 0.5)),
            "endpoint_extensions_added": int(debug.get("endpoint_extensions_added", 0)),
            "endpoint_extensions_clipped": int(debug.get("endpoint_extensions_clipped", 0)),
            "outline_overlap_allowed": bool(debug.get("outline_overlap_allowed", True)),
            "fill_uses_outline_clearance": bool(debug.get("fill_uses_outline_clearance", False)),
            "outside_overflow_mm2": float(debug.get("outside_overflow_mm2", float(overflow_area_px) / max(1e-9, px_per_mm * px_per_mm))),
            "endpoint_clamp_mode": "postprocess_only",
            "line_generation_changed": False,
            "global_fill_mask_changed": False,
            "infill_spacing_mm": float(infill_spacing_mm if infill_spacing_mm > 0 else fallback_spacing),
            "line_width_mm": float(line_width_mm),
            "endpoints_checked": int(debug.get("endpoints_checked", 0)),
            "endpoints_clamped": int(debug.get("endpoints_clamped", 0)),
            "max_endpoint_retract_mm": float(debug.get("max_endpoint_retract_mm", 0.0)),
            "coverage_before_endpoint_clamp_percent": float((100.0 * np.count_nonzero(target & (pre_endpoint_clamp_painted > 0)) / target_area_px) if target_area_px > 0 else 100.0),
            "coverage_after_endpoint_clamp_percent": float((100.0 * np.count_nonzero(target & (current_painted > 0)) / target_area_px) if target_area_px > 0 else 100.0),
        }
        infill_footprint = _paths_footprint_union((path for path in final_paths if path.kind == "fill-infill"), pen_radius_mm=pen_radius_mm)
        pre_clamp_infill_footprint = _paths_footprint_union((path for path in pre_endpoint_clamp_final_paths if path.kind == "fill-infill"), pen_radius_mm=pen_radius_mm)
        actual_outline_paths = _actual_outline_centerline_paths(final_paths)
        outline_cleanup_area = _paths_footprint_union(actual_outline_paths, pen_radius_mm=pen_radius_mm)
        coverage_report["infill_outline_overlap_area_mm2"] = float(infill_footprint.intersection(outline_cleanup_area).area) if not infill_footprint.is_empty and not outline_cleanup_area.is_empty else 0.0
        outline_footprint = _paths_footprint_union(actual_outline_paths, pen_radius_mm=pen_radius_mm)
        outline_limit_area = printable_geometry.union(outline_footprint) if printable_geometry is not None and not getattr(printable_geometry, "is_empty", True) else outline_footprint
        coverage_report["infill_beyond_outline_before_mm2"] = _infill_beyond_outline_area_mm2(pre_endpoint_clamp_final_paths, allowed_geom=outline_limit_area, pen_radius_mm=pen_radius_mm)
        coverage_report["infill_beyond_outline_after_mm2"] = _infill_beyond_outline_area_mm2(final_paths, allowed_geom=outline_limit_area, pen_radius_mm=pen_radius_mm)
        debug["coverage_report"] = coverage_report
        debug["infill_beyond_outline_before_mm2"] = float(coverage_report["infill_beyond_outline_before_mm2"])
        debug["infill_beyond_outline_after_mm2"] = float(coverage_report["infill_beyond_outline_after_mm2"])
        debug["pen_lifts_after_optimization"] = int(pen_lifts)
        debug["repair_candidates"] = [{key: value for key, value in row.items() if key != "candidate"} for row in repair_candidate_rows]
        debug["path_stats"] = {
            "total_paths": len(final_paths),
            "paths_by_kind": final_paths_by_kind,
            "draw_length_mm": draw_length_mm,
            "travel_length_mm": travel_length_mm,
            "travel_optimization_mode": str(debug.get("travel_optimization_mode", "")),
            "optimizer_runs_on_final_export_paths": bool(debug.get("optimizer_runs_on_final_export_paths", False)),
            "uses_surface_mm_for_ordering": bool(debug.get("uses_surface_mm_for_ordering", False)),
            "raw_travel_length_mm": float(debug.get("raw_travel_length_mm", 0.0)),
            "optimized_travel_length_mm": float(debug.get("optimized_travel_length_mm", 0.0)),
            "raw_longest_travel_mm": float(debug.get("raw_longest_travel_mm", 0.0)),
            "optimized_longest_travel_mm": float(debug.get("optimized_longest_travel_mm", 0.0)),
            "paths_reordered_count": int(debug.get("paths_reordered_count", 0)),
            "top_longest_travels_after": list(debug.get("top_longest_travels_after", [])),
            "repair_paths_exported": final_repair_paths_exported,
            "repair_candidates_generated": int(repair_candidates_generated),
            "repair_candidates_accepted": int(repair_candidates_accepted),
            "repair_candidates_rejected": int(repair_candidates_rejected),
            "detail_coverage_before_repair_percent": float(detail_coverage_before_repair_percent),
            "detail_coverage_after_repair_percent": float(detail_coverage_after_repair_percent),
            "detail_fillable_regions_failing_before_repair": int(detail_fillable_regions_failing_before_repair),
            "detail_fillable_regions_failing_after_repair": int(detail_fillable_regions_failing_after_repair),
            "missed_blob_count_before_repair": int(missed_blob_count_before_repair),
            "missed_blob_count_after_repair": int(missed_blob_count_after_repair),
        }
        debug["coverage_component_summary"] = component_debug
        debug["coverage_final_missed_px"] = int(np.count_nonzero(missed > 0))
        debug["coverage_final_overflow_px"] = int(np.count_nonzero(overflow > 0))
        debug["coverage_final_pen_lifts"] = int(pen_lifts)
        debug["coverage_final_mask_px_per_mm"] = float(px_per_mm)
        debug["coverage_final_painted_mask"] = current_painted
        debug["coverage_final_allowed_mask"] = allowed_mask
        debug["coverage_planner_resolution_mm"] = float(resolution_mm)
        debug["coverage_planner_px_per_mm"] = float(px_per_mm)
        debug["coverage_after_outline_percent"] = float(coverage_report.get("coverage_percent", 0.0))
        debug["thin_centerline_exported_count"] = int(sum(1 for path in final_paths if str(path.source or "") == "thin_source_region_centerline"))
        if os.getenv("WRITE_COVERAGE_DEBUG_ARTIFACTS", "1") != "0":
            artifact_dir = Path(os.getenv("COVERAGE_DEBUG_ARTIFACT_DIR", str(Path(tempfile.gettempdir()) / "golfball_plotter_coverage_debug")))
            debug["coverage_debug_artifact_dir"] = str(artifact_dir)
            _generate_debug_artifacts(
                output_dir=artifact_dir,
                shape=target_mask.shape,
                target_mask=target_mask,
                components_mask=(labels > 0).astype(np.uint8) * 255,
                initial_paths=initial_paths,
                skeleton_paths=skeleton_paths,
                boundary_paths=boundary_paths,
                final_paths=final_paths,
                repair_candidates=repair_candidate_rows,
                accepted_repair_paths=final_repair_paths,
                current_to_source_matrix=current_to_source,
                line_width_mm=line_width_mm,
                pen_radius_px=max(1, int(round(line_width_mm * px_per_mm / 2.0))),
                px_per_mm=px_per_mm,
                allowed_mask=allowed_mask,
                debug=debug,
            )
            debug["debug_artifacts_generated"] = [
                "selected_color_mask.png",
                "swept_coverage.png",
                "missed_blobs.png",
                "missed_blobs_labeled.png",
                "repair_candidates.png",
                "rejected_repair_candidates.png",
                "exported_repair_paths.png",
                "overflow_pixels.png",
                "missed_blob_diagnostics.json",
            ]
    return final_paths
