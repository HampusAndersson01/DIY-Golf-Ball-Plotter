from __future__ import annotations

import math
import re
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from shapely.geometry import LineString

from . import pipeline_core


_FEATURE_INFILL_TOKENS = ("infill", "internal infill", "solid infill")
_FEATURE_OUTLINE_TOKENS = ("perimeter", "wall", "external perimeter")
_NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"


@dataclass
class InfillGeometrySummary:
    angle_deg: float
    spacing_mm: float
    line_count: int
    segment_count: int
    bbox: tuple[float, float, float, float] | None
    lines: list[LineString]


@dataclass
class ReferenceGcodeParseResult:
    source_path: Path
    selected_layer_z: float | None
    infill: InfillGeometrySummary
    outline_segments: int
    travel_segments: int


def _normalize_undirected_angle_deg(angle_deg: float) -> float:
    normalized = angle_deg % 180.0
    if normalized < 0:
        normalized += 180.0
    return normalized


def _segment_angle_deg(line: LineString) -> float:
    (x1, y1), (x2, y2) = list(line.coords)
    return _normalize_undirected_angle_deg(math.degrees(math.atan2(y2 - y1, x2 - x1)))


def _segment_length(line: LineString) -> float:
    return float(line.length)


def _spacing_from_parallel_lines(lines: list[LineString], angle_deg: float) -> float:
    if len(lines) < 2:
        return 0.0
    normal_angle = math.radians((angle_deg + 90.0) % 180.0)
    nx, ny = math.cos(normal_angle), math.sin(normal_angle)
    projections = []
    for line in lines:
        cx, cy = line.centroid.coords[0]
        projections.append((cx * nx) + (cy * ny))
    projections.sort()
    deltas = [b - a for a, b in zip(projections, projections[1:]) if (b - a) > 1e-6]
    if not deltas:
        return 0.0
    return float(sorted(deltas)[len(deltas) // 2])


def summarize_line_family(lines: Iterable[LineString]) -> InfillGeometrySummary:
    usable = [line for line in lines if len(line.coords) == 2 and _segment_length(line) > 1e-6]
    if not usable:
        return InfillGeometrySummary(0.0, 0.0, 0, 0, None, [])
    angles = sorted(_segment_angle_deg(line) for line in usable)
    angle = float(angles[len(angles) // 2])
    spacing = _spacing_from_parallel_lines(usable, angle)
    min_x = min(min(x for x, _ in line.coords) for line in usable)
    min_y = min(min(y for _, y in line.coords) for line in usable)
    max_x = max(max(x for x, _ in line.coords) for line in usable)
    max_y = max(max(y for _, y in line.coords) for line in usable)
    return InfillGeometrySummary(
        angle_deg=angle,
        spacing_mm=spacing,
        line_count=len(usable),
        segment_count=len(usable),
        bbox=(float(min_x), float(min_y), float(max_x), float(max_y)),
        lines=usable,
    )


def extract_gcode_from_3mf(archive_path: Path, output_dir: Path) -> tuple[Path | None, list[str]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    inventory: list[str] = []
    selected: tuple[str, bytes] | None = None
    with zipfile.ZipFile(archive_path, "r") as zf:
        for name in zf.namelist():
            inventory.append(name)
            lowered = name.lower()
            if lowered.endswith((".gcode", ".bgcode", ".gco")) and selected is None:
                selected = (name, zf.read(name))
    if selected is None:
        return None, inventory
    member_name, payload = selected
    extracted_name = f"{archive_path.stem}_{Path(member_name).name}"
    extracted_path = output_dir / extracted_name
    extracted_path.write_bytes(payload)
    return extracted_path, inventory


def parse_reference_gcode(gcode_path: Path) -> ReferenceGcodeParseResult:
    lines = gcode_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    current_feature = ""
    x = y = z = 0.0
    e = 0.0
    has_e = False
    layer_segments: dict[float, list[tuple[LineString, str]]] = {}
    travel_segments = 0

    for raw in lines:
        stripped = raw.strip()
        if not stripped:
            continue
        if stripped.startswith(";"):
            comment = stripped[1:].strip().lower()
            if comment.startswith(("type:", "feature", "line_type")):
                current_feature = comment
            continue
        code = stripped.split(";", 1)[0].strip()
        if not code:
            continue
        if code.startswith("G0") or code.startswith("G1"):
            start_x, start_y, start_z, start_e = x, y, z, e
            x_match = re.search(rf"\bX({_NUM})", code)
            y_match = re.search(rf"\bY({_NUM})", code)
            z_match = re.search(rf"\bZ({_NUM})", code)
            e_match = re.search(rf"\bE({_NUM})", code)
            if x_match:
                x = float(x_match.group(1))
            if y_match:
                y = float(y_match.group(1))
            if z_match:
                z = float(z_match.group(1))
            if e_match:
                new_e = float(e_match.group(1))
                has_e = True
                e = new_e
            moved_xy = (abs(x - start_x) > 1e-9) or (abs(y - start_y) > 1e-9)
            if not moved_xy:
                continue
            extrusion_move = has_e and ((e - start_e) > 1e-9)
            if not extrusion_move:
                travel_segments += 1
                continue
            layer_key = round(start_z if abs(start_z) > 1e-9 else z, 5)
            seg = LineString([(start_x, start_y), (x, y)])
            layer_segments.setdefault(layer_key, []).append((seg, current_feature))
        elif code.startswith("G92"):
            e_match = re.search(rf"\bE({_NUM})", code)
            if e_match:
                e = float(e_match.group(1))
                has_e = True

    if not layer_segments:
        return ReferenceGcodeParseResult(
            source_path=gcode_path,
            selected_layer_z=None,
            infill=summarize_line_family([]),
            outline_segments=0,
            travel_segments=travel_segments,
        )

    def _layer_rank(item: tuple[float, list[tuple[LineString, str]]]) -> tuple[float, float]:
        _, segs = item
        infill_len = sum(
            float(seg.length)
            for seg, feature in segs
            if any(token in feature.lower() for token in _FEATURE_INFILL_TOKENS)
        )
        total_len = sum(float(seg.length) for seg, _ in segs)
        return infill_len, total_len

    best_layer = max(layer_segments.items(), key=_layer_rank)[0]
    segments = layer_segments[best_layer]
    infill_lines: list[LineString] = []
    outline_count = 0
    for seg, feature in segments:
        feature_lower = feature.lower()
        if any(token in feature_lower for token in _FEATURE_INFILL_TOKENS):
            infill_lines.append(seg)
        elif any(token in feature_lower for token in _FEATURE_OUTLINE_TOKENS):
            outline_count += 1
    if not infill_lines:
        # Fallback: choose dominant undirected angle family as infill-like hatch.
        by_bucket: dict[int, list[LineString]] = {}
        for seg, _ in segments:
            bucket = int(round(_segment_angle_deg(seg))) % 180
            by_bucket.setdefault(bucket, []).append(seg)
        dominant = max(by_bucket.values(), key=len)
        infill_lines = dominant
    return ReferenceGcodeParseResult(
        source_path=gcode_path,
        selected_layer_z=best_layer,
        infill=summarize_line_family(infill_lines),
        outline_segments=outline_count,
        travel_segments=travel_segments,
    )


def parse_motion_paths_for_kind(gcode: list[str], *, pen_up_s: int, pen_down_s: int, kind: str) -> list[pipeline_core.Toolpath]:
    return [path for path in pipeline_core.parse_gcode_machine_motion_paths(gcode, pen_up_s=pen_up_s, pen_down_s=pen_down_s) if path.kind == kind]
